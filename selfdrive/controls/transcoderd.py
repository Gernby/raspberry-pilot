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
MODEL_VERSION = '025'
HISTORY_ROWS = 3
OUTPUT_ROWS = 15
BATCH_SIZE = 1

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

                                        # vehicle_inputs,    fingerprints,  left_inputs,                 far_left_inputs,             right_inputs,          far_right_inputs
all_inputs = [model_input[:,:,:9], fingerprint, model_input[:,:,9:24], model_input[:,:,24:39], model_input[:,:,39:54], model_input[:,:,54:69]]
new_inputs = [model_input[-1:,:,:9], fingerprint[-1,:,:], model_input[-1:,:,9:24], model_input[-1:,:,24:39], model_input[-1:,:,39:54], model_input[-1:,:,54:69]]

model_output = None
start_time = time.time()
if False:
  for n in range(100):
    for i in range(len(all_inputs)):
      all_inputs[i][:-1] = all_inputs[i][1:]
      all_inputs[i][-1:] = new_inputs[i] + (n / 200)
    previous_output = model_output
    model_output = model.predict_on_batch(all_inputs)
    #model_output = model.predict(all_inputs)
    time.sleep(0.03)
  end_time = time.time()
  print("batch size: %d   exec time per batch: %0.5f" % (BATCH_SIZE, (end_time - start_time - 3)/100))

model_output = model.predict(all_inputs)

frame = 0
dump_sock(gernModelInputs, True)
diverging = False

print(fingerprint)
while 1:
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
  lane_width = max(570, lane_width - max_width_step * 2, min(1200, lane_width + max_width_step, cs.camLeft.parm2 - cs.camRight.parm2))
  
  #if False and l_prob > 0 and r_prob > 0 and not diverging:
  #  if lane_width > 0:
  #    lane_width = max(lane_width - 0.1 * cs.vEgo, min(lane_width + 0.1 * cs.vEgo, lane_width + 0.01 * l_prob * r_prob * cs.vEgo * (min(1100, max(570, cs.camLeft.parm2 -  cs.camRight.parm2)) - lane_width)))
      #lane_width = lane_width + 0.01 * l_prob * r_prob * cs.vEgo * (min(1100, max(570, cs.camLeft.parm2 -  cs.camRight.parm2)) - lane_width)
  #  else:
  #    lane_width = min(1100, max(570, cs.camLeft.parm2 -  cs.camRight.parm2))
  #  half_width = lane_width / 2
    #half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.48))
  #else:
  #  half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.47))

  lr_prob = (l_prob + r_prob) - l_prob * r_prob
  
  #left_center = min(0.8, max(0.1, l_prob)) * (descaled_output[:,1:2] + half_width) + max(0.2, min(0.9, 1-l_prob)) * left_center
  #right_center = min(0.8, max(0.1, r_prob)) * (descaled_output[:,2:3] - half_width) + max(0.2, min(0.9, 1-r_prob)) * right_center
  #calc_center = min(0.8, max(0.1, lr_prob)) * ((l_prob * descaled_output[:,1:2] + r_prob * descaled_output[:,2:3]) / (l_prob + r_prob + 0.0005)) + max(0.2, min(0.9, 1-lr_prob)) * calc_center
  #calc_center = (l_prob * left_center + r_prob * right_center) / (l_prob + r_prob + 0.0005)
  #calc_center = (1-lr_prob) * descaled_output[:,1:2] + lr_prob * calc_center
  left = descaled_output[:,3:4]
  right = descaled_output[:,5:6]
  center = descaled_output[:,1:2]
  abs_left = np.sum(np.absolute(left)) 
  abs_right = np.sum(np.absolute(right)) 
  left_center = l_prob * left + (1-l_prob) * center
  right_center = r_prob * right + (1-r_prob) * center
  calc_center = (abs_left * right_center + abs_right * left_center) / (abs_left + abs_right)
  
  calc_center_limit = lr_prob * center_limit * cs.vEgo * 5
  calc_center = np.clip(calc_center, calc_center_prev - calc_center_limit, calc_center_prev + calc_center_limit)
  calc_center_prev = calc_center
  #  calc_center = center 
  #elif  > np.sum(np.absolute(right_center)):
  #  calc_center = r_prob * right_center + (1-r_prob) * center
  #else:
  #  calc_center = l_prob * left_center + (1-l_prob) * center
  if cs.vEgo > 10 and l_prob > 0 and r_prob > 0:
    if left[0,0] <= right[0,0]:
      width_trim -= 1
    else:
      width_trim += 1


  #calc_center = 0.5 * (l_prob * descaled_output[:,3:4] + (1-l_prob) * descaled_output[:,1:2]) + 0.5 * (r_prob * descaled_output[:,5:6] + (1-r_prob) * descaled_output[:,1:2])
  #calc_center = (lr_prob * new_calc_center + lr_prob_prev * calc_center) / (lr_prob + lr_prob_prev + 0.0001)
  #lr_prob_prev = min(0.2, lr_prob)

  #left_s = descaled_output[:,2:3]
  #right_s = descaled_output[:,4:5]
  #center_s = descaled_output[:,0:1]
  #abs_left_s = np.sum(np.absolute(left_s)) 
  #abs_right_s = np.sum(np.absolute(right_s)) 
  #left_center_s = l_prob * left_s + (1-l_prob) * center_s
  #right_center_s = r_prob * right_s + (1-r_prob) * center_s
  #calc_center_s = (abs_left * right_center_s + abs_right * left_center_s) / (abs_left + abs_right)

  if abs(cs.steeringTorque) < 1200 and abs(cs.adjustedAngle) < 30:
    clipped_angle[:-1] = clipped_angle[1:]
    upper_limit = one_deg_per_sec * cs.vEgo * (max(1, min(2, abs(cs.steeringRate))) + accel_counter)
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
      #angle = np.clip((calc_center_s[:,0:1] - calc_center_s[0,0:1]) * (1 + advanceSteer), lower_limit, upper_limit)
      #angle = np.clip((descaled_output[:,3:4] - descaled_output[0,3:4]) * (1 + advanceSteer), lower_limit, upper_limit)
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
  #path_send.pathPlan.mpcAngles = [float(x) for x in (angle_factor * (clipped_angle[:] + descaled_output[0,3:4]) - total_offset - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  #path_send.pathPlan.mpcAngles = [float(x) for x in (angle_factor * calc_center_s - total_offset - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  path_send.pathPlan.mpcAngles = [float(x) for x in (requested_angle + cs.steeringAngle - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  #path_send.pathPlan.mpcAngles = [float(x) for x in (angle_factor * (angle + descaled_output[0,0:1]) - total_offset - angle_bias)]   #angle_steers.pop(output_list[-1]))]
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
    #try:
    print('lane_width: %0.1f center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  total_offset:  %0.2f  angle_bias:  %0.2f  model_angle:  %0.2f  model_center_offset:  %0.2f  model exec time:  %0.4fs' % (lane_width, calc_center[-1], l_prob, r_prob, total_offset, angle_bias, descaled_output[1,0], descaled_output[1,1], execution_time_avg), fingerprint[-1:,-1:])
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
