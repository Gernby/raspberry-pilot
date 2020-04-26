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


INPUTS = 69
OUTPUTS = 6
MODEL_VERSION = '024'
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

fingerprint = np.zeros((5,7))
kegman = kegman_conf()  
if int(kegman.conf['fingerprint']) >= 0: 
  fingerprint[:,int(kegman.conf['fingerprint'])] = 1

                                        # vehicle_inputs,    fingerprints,  left_inputs,                 far_left_inputs,             right_inputs,          far_right_inputs
all_inputs = [[model_input[:,:9]], [fingerprint], [model_input[:,9:24]], [model_input[:,24:39]], [model_input[:,39:54]], [model_input[:,54:69]]]

start_time = time.time()
for i in range(100):
  model_output = model.predict(all_inputs)
  time.sleep(0.03)
end_time = time.time()
descaled_output = output_scaler.inverse_transform(model_output[-1])

frame = 0
dump_sock(gernModelInputs, True)
diverging = False

while 1:
  cs = car.CarState.from_bytes(gernModelInputs.recv())
  
  model_input = np.asarray(cs.modelData).reshape(HISTORY_ROWS, INPUTS)

  all_inputs = [[model_input[:,:9]], [fingerprint], [model_input[:,9:24]], [model_input[:,24:39]], [model_input[:,39:54]], [model_input[:,54:69]]]

  start_time = time.time()  
  model_output = model.predict_on_batch(all_inputs)
  execution_time_avg += max(0.0001, time_factor) * ((time.time() - start_time) - execution_time_avg)
  time_factor *= 0.96

  frame += 1

  descaled_output = output_scaler.inverse_transform(model_output[0])
  
  l_prob = min(1, max(0, cs.camLeft.parm4 / 127))
  r_prob = min(1, max(0, cs.camRight.parm4 / 127))

  max_width_step = 0.05 * cs.vEgo * l_prob * r_prob
  lane_width = max(570, lane_width - max_width_step * 2, min(1200, lane_width + max_width_step, cs.camLeft.parm2 - cs.camRight.parm2))
 
  lr_prob = (l_prob + r_prob) - l_prob * r_prob
  
  left = descaled_output[:,3:4]
  right = descaled_output[:,5:6]
  center = descaled_output[:,1:2]
  abs_left = np.sum(np.absolute(left)) 
  abs_right = np.sum(np.absolute(right)) 
  left_center = l_prob * left + (1-l_prob) * center
  right_center = r_prob * right + (1-r_prob) * center
  calc_center = (abs_left * right_center + abs_right * left_center) / (abs_left + abs_right)

  left_s = descaled_output[:,2:3]
  right_s = descaled_output[:,4:5]
  center_s = descaled_output[:,0:1]
  left_center_s = l_prob * left_s + (1-l_prob) * center_s
  right_center_s = r_prob * right_s + (1-r_prob) * center_s
  calc_center_s = (abs_left * right_center_s + abs_right * left_center_s) / (abs_left + abs_right)

  if abs(cs.steeringTorque) < 1200 and abs(cs.adjustedAngle) < 30:
    angle[:-1] = angle[1:]
    upper_limit = one_deg_per_sec * cs.vEgo * (max(50, min(50, abs(cs.steeringRate))) + accel_counter)
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
      angle = np.clip((calc_center_s[:,0:1] - calc_center_s[0,0:1]) * (1 + advanceSteer), lower_limit, upper_limit)
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

  path_send.pathPlan.angleSteers = float(angle[5] + cs.steeringAngle - angle_bias)
  path_send.pathPlan.mpcAngles = [float(x) for x in (angle + cs.steeringAngle - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  path_send.pathPlan.laneWidth = float(lane_width)
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
    print('lane_width: %0.1f center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  total_offset:  %0.2f  angle_bias:  %0.2f  model_angle:  %0.2f  model_center_offset:  %0.2f  model exec time:  %0.4fs' % (lane_width, calc_center[-1], l_prob, r_prob, total_offset, angle_bias, descaled_output[1,0], descaled_output[1,1], round(execution_time_avg,2)), fingerprint[-1:,:])
    
# TODO: replace kegman_conf with params!
if frame % 100 == 0:
#try:
  kegman = kegman_conf()  
  advanceSteer = max(0, float(kegman.conf['advanceSteer']))
  angle_factor = float(kegman.conf['angleFactor'])
  use_bias = float(kegman.conf['angleBias'])
  use_angle_offset = float(kegman.conf['angleOffset'])
  use_lateral_offset = float(kegman.conf['lateralOffset'])
