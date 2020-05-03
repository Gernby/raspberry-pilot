#!/usr/bin/env python
import os
import zmq
import time
import json
from tensorflow.python.keras.models import load_model #, Model  #, Sequential
import joblib
import numpy as np
from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from enum import Enum
from cereal import log, car
from setproctitle import setproctitle
from common.params import Params
setproctitle('transcoderd')

#import sys
#sys.stderr = open('../laterald.txt', 'w')


INPUTS = 69
OUTPUTS = 6
MODEL_VERSION = '029'

HISTORY_ROWS = 5
OUTPUT_ROWS = 15
BATCH_SIZE = 3

output_scaler = joblib.load(os.path.expanduser('models/GRU_MinMax_tanh_%d_outputs_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
model = load_model(os.path.expanduser('models/cpu-model-%s.hdf5' % MODEL_VERSION))
model_input = np.zeros((BATCH_SIZE,HISTORY_ROWS, INPUTS))
print(model.summary())

def dump_sock(sock, wait_for_one=False):
  if wait_for_one:
    sock.recv()
  while 1:
    try:
      sock.recv(zmq.NOBLOCK)
    except zmq.error.Again:
      break

def pub_sock(port, addr="*"):
  context = zmq.Context.instance()
  sock = context.socket(zmq.PUB)
  sock.bind("tcp://%s:%d" % (addr, port))
  return sock

def sub_sock(port, poller=None, addr="127.0.0.1", conflate=False, timeout=None):
  context = zmq.Context.instance()
  sock = context.socket(zmq.SUB)
  if conflate:
    sock.setsockopt(zmq.CONFLATE, 1)
  sock.connect("tcp://%s:%d" % (addr, port))
  sock.setsockopt(zmq.SUBSCRIBE, b"")

  if timeout is not None:
    sock.RCVTIMEO = timeout

  if poller is not None:
    poller.register(sock, zmq.POLLIN)
  return sock

def project_error(error):
  error_start = error[0,0]
  error -= error_start
  error_max = np.argmax(error)
  error_min = np.argmin(error)
  if min(error_min, error_max) < 5:
    error[max(error_min, error_max):,0] = error[max(error_min, error_max),0]
  else:
    error[min(error_min, error_max):,0] = error[min(error_min, error_max),0]
  return error_start + error


gernPath = pub_sock(service_list['pathPlan'].port)
gernModelInputs = sub_sock(service_list['model'].port, conflate=True)

frame_count = 1
dashboard_count = 0
lane_width = 0
half_width = 0
width_trim = 0
angle_bias = 0.0
path_send = log.Event.new_message()
path_send.init('pathPlan')
advanceSteer = 1
one_deg_per_sec = np.ones((OUTPUT_ROWS,1)) / 15
left_center = np.zeros((OUTPUT_ROWS,1))
right_center = np.zeros((OUTPUT_ROWS,1))
calc_center = np.zeros((OUTPUT_ROWS,1))
projected_center = np.zeros((OUTPUT_ROWS,1))
left_probs = np.zeros((OUTPUT_ROWS,1))
right_probs = np.zeros((OUTPUT_ROWS,1))
clipped_angle = np.zeros((OUTPUT_ROWS,1))
requested_angle = clipped_angle
center_limit = np.reshape(0.5 * np.arange(OUTPUT_ROWS) + 10,(OUTPUT_ROWS,1))
accel_counter = 0   
upper_limit = 0
lower_limit = 0
lr_prob_prev = 0
lr_prob_prev_prev = 0
center_rate_prev = 0
calc_center_prev = calc_center
angle_factor = 1.0
all_inputs = []

execution_time_avg = 0.0
time_factor = 1.0

fingerprint = np.zeros((BATCH_SIZE,HISTORY_ROWS,7))
kegman = kegman_conf()  
if int(kegman.conf['fingerprint']) >= 0: 
  fingerprint[:,:,int(kegman.conf['fingerprint'])] = 1

all_inputs = [model_input[:,:,:9], fingerprint, model_input[:,:,9:24], model_input[:,:,24:39], model_input[:,:,39:54], model_input[:,:,54:69]]
new_inputs = [model_input[-1:,:,:9], fingerprint[-1,:,:], model_input[-1:,:,9:24], model_input[-1:,:,24:39], model_input[-1:,:,39:54], model_input[-1:,:,54:69]]

model_output = None
start_time = time.time()

model_output = model.predict(all_inputs)

frame = 0
dump_sock(gernModelInputs, True)
diverging = False

while 1:
  try:
    cs = car.CarState.from_bytes(gernModelInputs.recv())
    
    model_input = np.asarray(cs.modelData).reshape(1, HISTORY_ROWS, INPUTS)
    new_inputs = [model_input[-1:,:,:9], fingerprint[-1:,:,:], model_input[-1:,:,9:24], model_input[-1:,:,24:39], model_input[-1:,:,39:54], model_input[-1:,:,54:69]]
    for i in range(len(all_inputs)):
      all_inputs[i][:-1] = all_inputs[i][1:]
      all_inputs[i][-1:] = new_inputs[i]

    start_time = time.time()  
    model_output = model.predict_on_batch(all_inputs)
    execution_time_avg += max(0.0001, time_factor) * ((time.time() - start_time) - execution_time_avg)
    time_factor *= 0.96

    frame += 1

    descaled_output = output_scaler.inverse_transform(model_output[-1])
    
    l_prob = min(1, max(0, cs.camLeft.parm4 / 127))
    r_prob = min(1, max(0, cs.camRight.parm4 / 127))

    diverging = False
    if False:
      if descaled_output[-1:, 1:2] > descaled_output[-1:, 2:3]:
        if cs.camLeft.solid > 0 and cs.camRight.solid <= 0:
          #pass
          diverging = True
          l_prob *= 0.2
          print("      Diverging Left", l_prob)
        if cs.camLeft.solid <= 0 and cs.camRight.solid > 0:
          diverging = True
          #pass
          r_prob *= 0.2
          print("      Diverging Right", r_prob)

    max_width_step = 0.05 * cs.vEgo * l_prob * r_prob
    lane_width = max(570, lane_width - max_width_step * 2, min(1200, lane_width + max_width_step, max(0, cs.camLeft.parm2) - min(0, cs.camRight.parm2)))
    
    lr_prob = (l_prob + r_prob) - l_prob * r_prob
    
    left = descaled_output[:,3:4]
    right = descaled_output[:,5:6]
    center = descaled_output[:,1:2]
    abs_left = np.sum(np.absolute(left)) 
    abs_right = np.sum(np.absolute(right)) 
    left_center = l_prob * left + (1-l_prob) * center
    right_center = r_prob * right + (1-r_prob) * center
    calc_center = project_error(abs_left * right_center + abs_right * left_center) / (abs_left + abs_right)

    if cs.vEgo > 10 and l_prob > 0 and r_prob > 0:
      if left[0,0] <= right[0,0]:
        width_trim -= 1
      else:
        width_trim += 1

    if abs(cs.steeringTorque) < 1200 and abs(cs.adjustedAngle) < 30:
      clipped_angle[:-1] = clipped_angle[1:]
      upper_limit = one_deg_per_sec * cs.vEgo * (max(2, min(5, abs(cs.steeringRate))) + accel_counter)
      lower_limit = -upper_limit
      if cs.torqueRequest >= 1:
        upper_limit = one_deg_per_sec * cs.steeringRate
        lower_limit = clipped_angle + lower_limit
      elif cs.torqueRequest <= -1:
        lower_limit = one_deg_per_sec * cs.steeringRate
        upper_limit = clipped_angle + upper_limit
      else:
        upper_limit = upper_limit + clipped_angle
        lower_limit = lower_limit + clipped_angle
      if l_prob + r_prob > 0:
        accel_counter = max(0, min(2, accel_counter - 1))
        model_angle = (descaled_output[:,0:1] - descaled_output[0,0:1]) * (1 + advanceSteer)
        clipped_angle = np.clip(model_angle, lower_limit, upper_limit)
        requested_angle = np.clip(model_angle, clipped_angle - 0.5, clipped_angle + 0.5)
      else:
        accel_counter = max(0, min(2, accel_counter + 1))
        requested_angle *= 0.9
        clipped_angle = requested_angle
    else:
      requested_angle = one_deg_per_sec * cs.steeringRate
      clipped_angle = requested_angle

    if abs(cs.steeringRate) < 5 and abs(cs.adjustedAngle) < 3 and cs.torqueRequest != 0:
      if calc_center[-1,0] < 0:
        angle_bias += (0.00001 * cs.vEgo)
      elif calc_center[-1,0] > 0:
        angle_bias -= (0.00001 * cs.vEgo)
      
    total_offset = cs.adjustedAngle - cs.steeringAngle

    path_send.pathPlan.angleSteers = float(requested_angle[5] + cs.steeringAngle - angle_bias)
    path_send.pathPlan.mpcAngles = [float(x) for x in (requested_angle + cs.steeringAngle - angle_bias)]  
    path_send.pathPlan.laneWidth = float(lane_width + width_trim)
    path_send.pathPlan.angleOffset = total_offset
    path_send.pathPlan.lPoly = [float(x) for x in (left_center[:,0] + 0.5 * lane_width)]
    path_send.pathPlan.rPoly = [float(x) for x in (right_center[:,0] - 0.5 * lane_width)]
    path_send.pathPlan.cPoly = [float(x) for x in (calc_center[:,0])]
    path_send.pathPlan.lProb = float(l_prob)
    path_send.pathPlan.rProb = float(r_prob)
    path_send.pathPlan.cProb = float(lr_prob)
    path_send.pathPlan.canTime = cs.canTime
    gernPath.send(path_send.to_bytes())

    path_send = log.Event.new_message()
    path_send.init('pathPlan')
    if frame % 30 == 0:
      print('lane_width: %0.1f center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  total_offset:  %0.2f  angle_bias:  %0.2f  model_angle:  %0.2f  model_center_offset:  %0.2f  model exec time:  %0.4fs' % (lane_width, calc_center[-1], l_prob, r_prob, total_offset, angle_bias, descaled_output[1,0], descaled_output[1,1], execution_time_avg), fingerprint[-1:,-1:])
      
    # TODO: replace kegman_conf with params!
    if frame % 100 == 0:
      kegman = kegman_conf()  
      advanceSteer = max(0, float(kegman.conf['advanceSteer']))
      angle_factor = float(kegman.conf['angleFactor'])
      use_bias = float(kegman.conf['angleBias'])
      use_angle_offset = float(kegman.conf['angleOffset'])
      use_lateral_offset = float(kegman.conf['lateralOffset'])
  except:
    pass
