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


INPUTS = 52
OUTPUTS = 5
MODEL_VERSION = '022'
HISTORY_ROWS = 5
OUTPUT_ROWS = 15

output_scaler = joblib.load(os.path.expanduser('models/GRU_MinMax_tanh_%d_outputs_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
model = load_model(os.path.expanduser('models/cpu-model-%s.hdf5' % MODEL_VERSION))
model_input = np.zeros((HISTORY_ROWS, INPUTS))
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

gernPath = pub_sock(service_list['pathPlan'].port)
gernModelInputs = sub_sock(service_list['model'].port, conflate=True)

frame_count = 1
dashboard_count = 0
lane_width = 0
half_width = 0
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
angle = np.zeros((OUTPUT_ROWS,1))
accel_counter = 0
upper_limit = 0
lower_limit = 0
lr_prob_prev = 0
lr_prob_prev_prev = 0
center_rate_prev = 0
calc_center_prev = 0
angle_factor = 1.0

execution_time_avg = 0.0
time_factor = 1.0

fingerprint = np.array([[0,0,0,0,0,0,0]])
kegman = kegman_conf()  
if int(kegman.conf['fingerprint']) >= 0: 
  fingerprint[0,int(kegman.conf['fingerprint'])] = 1

                        # vehicle_inputs,       steer_inputs,           error_indicators,          vehicle_center,           camera_flags,        fingerprints,    camera_offset_inputs,      camera_left_inputs,      camera_right_inputs
model_output = model.predict_on_batch([[model_input[:,:6]], [model_input[:,6:9]], [model_input[-1:,9:11]], [model_input[-1:,11:12]], [model_input[-1:,12:20]], [fingerprint], [model_input[:,20:28]], [model_input[:,-24:-12]], [model_input[:,-12:]]])
descaled_output = output_scaler.inverse_transform(model_output[0])
print(np.round(descaled_output,2))

frame = 0
dump_sock(gernModelInputs, True)
diverging = False

while 1:
  cs = car.CarState.from_bytes(gernModelInputs.recv())
  
  model_input = np.asarray(cs.modelData).reshape(HISTORY_ROWS, INPUTS)

  all_inputs = [[model_input[:,:6]], [model_input[:,6:9]], [model_input[-1:,9:11]], [model_input[-1:,11:12]], [model_input[-1:,12:20]], [fingerprint], [model_input[:,20:28]], [model_input[:,-24:-12]], [model_input[:,-12:]]]

  start_time = time.time()  
  model_output = model.predict_on_batch(all_inputs)
  execution_time_avg += max(0.0001, time_factor) * ((time.time() - start_time) - execution_time_avg)
  time_factor *= 0.96

  if frame % 30 == 0:
    print(fingerprint, frame, start_time, execution_time_avg)
  
  frame += 1

  descaled_output = output_scaler.inverse_transform(model_output[0])
  
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
  
  if l_prob > 0 and r_prob > 0 and not diverging:
    if lane_width > 0:
      lane_width += 0.01 * (min(1100, max(570, cs.camLeft.parm2 -  cs.camRight.parm2) - lane_width))
    else:
      lane_width = min(1100, max(570, cs.camLeft.parm2 -  cs.camRight.parm2) - lane_width)
      half_width = lane_width / 2
    half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.48))
  else:
    half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.47))

  lr_prob = (l_prob + r_prob) - l_prob * r_prob
  
  #left_center = min(0.8, max(0.1, l_prob)) * (descaled_output[:,1:2] + half_width) + max(0.2, min(0.9, 1-l_prob)) * left_center
  #right_center = min(0.8, max(0.1, r_prob)) * (descaled_output[:,2:3] - half_width) + max(0.2, min(0.9, 1-r_prob)) * right_center
  #calc_center = min(0.8, max(0.1, lr_prob)) * ((l_prob * descaled_output[:,1:2] + r_prob * descaled_output[:,2:3]) / (l_prob + r_prob + 0.0005)) + max(0.2, min(0.9, 1-lr_prob)) * calc_center
  left_center = descaled_output[:,1:2]
  right_center = descaled_output[:,2:3]
  #calc_center = (l_prob * descaled_output[:,1:2] + r_prob * descaled_output[:,2:3]) / (l_prob + r_prob + 0.0005)
  calc_center = descaled_output[:,4:5] #* lr_prob + (1-lr_prob) * calc_center

  if abs(cs.steeringTorque) < 1200 and abs(cs.adjustedAngle) < 30:
    upper_limit = one_deg_per_sec * cs.vEgo * (max(2, min(5, abs(cs.steeringRate))) + accel_counter)
    lower_limit = -upper_limit
    if cs.torqueRequest >= 1:
      upper_limit = one_deg_per_sec * cs.steeringRate
      lower_limit = angle + lower_limit
    elif cs.torqueRequest <= -1:
      lower_limit = one_deg_per_sec * cs.steeringRate
      upper_limit = angle + upper_limit
    else:
      upper_limit = upper_limit + angle
      lower_limit = lower_limit + angle
    if l_prob + r_prob > 0:
      accel_counter = max(0, min(2, accel_counter - 1))
      angle = np.clip((descaled_output[:,3:4] - descaled_output[0,3:4]) * (1 + advanceSteer), lower_limit, upper_limit)
      #angle = np.clip((descaled_output[:,3:4] - descaled_output[0,3:4]) * (1 + advanceSteer), lower_limit, upper_limit)
    else:
      accel_counter = max(0, min(2, accel_counter + 1))
      angle *= 0.9
  else:
    angle = one_deg_per_sec * cs.steeringRate

  if abs(cs.steeringRate) < 5 and abs(cs.adjustedAngle) < 3 and cs.torqueRequest != 0:
    if calc_center[-1,0] < 0:
      angle_bias += (0.00001 * cs.vEgo)
    elif calc_center[-1,0] > 0:
      angle_bias -= (0.00001 * cs.vEgo)
    
  total_offset = cs.adjustedAngle - cs.steeringAngle

  path_send.pathPlan.angleSteers = float(angle[5] + cs.steeringAngle  - angle_bias)
  #path_send.pathPlan.mpcAngles = [float(x) for x in (angle_factor * (angle[:] + descaled_output[0,3:4]) - total_offset - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  path_send.pathPlan.mpcAngles = [float(x) for x in (angle + cs.steeringAngle - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  #path_send.pathPlan.mpcAngles = [float(x) for x in (angle_factor * (angle + descaled_output[0,0:1]) - total_offset - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  path_send.pathPlan.laneWidth = float(lane_width)
  path_send.pathPlan.angleOffset = total_offset
  path_send.pathPlan.lPoly = [float(x) for x in (left_center[:,0] + half_width)]
  path_send.pathPlan.rPoly = [float(x) for x in (right_center[:,0] - half_width)]
  path_send.pathPlan.cPoly = [float(x) for x in (calc_center[:,0])]
  path_send.pathPlan.lProb = float(l_prob)
  path_send.pathPlan.rProb = float(r_prob)
  path_send.pathPlan.cProb = float(lr_prob)
  path_send.pathPlan.canTime = cs.canTime
  gernPath.send(path_send.to_bytes())

  path_send = log.Event.new_message()
  path_send.init('pathPlan')
  if frame % 30 == 0:
    #try:
    print('half_width: %0.1f center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  total_offset:  %0.2f  angle_bias:  %0.2f  model_lane_width:  %0.2f  model_center_offset:  %0.2f' % (half_width, calc_center[-1], l_prob, r_prob, total_offset, angle_bias, descaled_output[1,1], descaled_output[1,2]))
    #print(np.round(projected_center[:,0] - projected_center[0,0],1), np.round(projected_center[:,0] - calc_center[0,0],1))
    
# TODO: replace kegman_conf with params!
if frame % 100 == 0:
#try:
  kegman = kegman_conf()  
  advanceSteer = max(0, float(kegman.conf['advanceSteer']))
  angle_factor = float(kegman.conf['angleFactor'])
  use_bias = float(kegman.conf['angleBias'])
  use_angle_offset = float(kegman.conf['angleOffset'])
  use_lateral_offset = float(kegman.conf['lateralOffset'])

#if frame % 1000 == 2:
#  params.put("LateralParams", json.dumps({'angle_offset': angle_offset, 'angle_bias': angle_bias, 'lateral_offset': lateral_offset}))
