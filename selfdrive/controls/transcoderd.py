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


INPUTS = 48
OUTPUTS = 5
MODEL_VERSION = '015'
HISTORY_ROWS = 10

output_scaler = joblib.load(os.path.expanduser('models/GRU_MinMax_tanh_%d_outputs_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
model = load_model(os.path.expanduser('models/cpu-model-%s.hdf5' % MODEL_VERSION))
model_input = np.zeros((HISTORY_ROWS, INPUTS))


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
gernModelInputs = sub_sock(service_list['model'].port, conflate=False)

frame_count = 1
dashboard_count = 0
scaler_padding = None 
lane_width = 0
half_width = 0
angle_bias = 0.0
path_send = log.Event.new_message()
path_send.init('pathPlan')
advanceSteer = 1
one_deg_per_sec = np.ones((30,1)) / 15
accel_counter = 0
upper_limit = 0
lower_limit = 0
lr_prob_prev = 0
lr_prob_prev_prev = 0
center_rate_prev = 0
calc_center_prev = 0
row_count = 0
column_count = 0
angle_factor = 1.0
fingerprint = np.array([[0,0,0,0,0]])
kegman = kegman_conf()  
if int(kegman.conf['fingerprint']) >= 0: 
  fingerprint[0,int(kegman.conf['fingerprint'])] = 1

model.predict_on_batch([[model_input[:,:8]], [model_input[-1:,-40:-32]], [fingerprint], [model_input[:,-32:-24]], [model_input[:,-24:-12]], [model_input[:,-12:]]])
frame = 0
dump_sock(gernModelInputs, True)

while 1:
  cs = car.CarState.from_bytes(gernModelInputs.recv())
  
  model_input = np.asarray(cs.modelData).reshape(HISTORY_ROWS, INPUTS)

  all_inputs = [[model_input[:,:8]], [model_input[-1:,-40:-32]], [fingerprint], [model_input[:,-32:-24]], [model_input[:,-24:-12]], [model_input[:,-12:]]]

  model_output = list(model.predict_on_batch(all_inputs)[0].astype('float'))
  #model_output.append(input_list[-1])
  #gernModelOutputs.send_json(model_output)
  if frame % 30 == 0:
    print(fingerprint, frame, time.time())
  
  frame += 1

  #  output_list = list(socket.recv_json())
  model_output = np.asarray(model_output)
  if scaler_padding is None:
    column_count = OUTPUTS
    row_count = len(model_output)//column_count
    scaler_padding = [np.zeros((row_count,OUTPUTS)), np.zeros((row_count,OUTPUTS))]
    left_center = np.zeros((row_count,1))
    right_center = np.zeros((row_count,1))
    calc_center = np.zeros((row_count,1))
    projected_center = np.zeros((row_count,1))
    left_probs = np.zeros((row_count,1))
    right_probs = np.zeros((row_count,1))
    angle = np.zeros((row_count,1))

  model_output = model_output.reshape(row_count,column_count)

  scaler_padding[0] = np.asarray(model_output)
  descaled_output = [output_scaler.inverse_transform(scaler_padding[0]), output_scaler.inverse_transform(scaler_padding[1])]

  l_prob = cs.camLeft.parm4 / 127
  r_prob = cs.camRight.parm4 / 127

  if l_prob < 0 and r_prob > 0 and descaled_output[0][-1:, 1:2] > -descaled_output[0][-1:, 2:3] * 1.5:
    #pass
    #l_prob *= 0.2
    print("      Diverging Left", l_prob)
  elif r_prob < 0 and l_prob > 0 and descaled_output[0][-1:, 1:2] * 1.5 < -descaled_output[0][-1:,2:3]:
    #pass
    #r_prob *= 0.2
    print("      Diverging Right", r_prob)
  elif abs(l_prob) > 0 and abs(r_prob) > 0:
    if lane_width > 0:
      lane_width += 0.01 * (min(700, max(570, cs.camLeft.parm2 -  cs.camRight.parm2) - lane_width))
    else:
      lane_width = min(700, max(570, cs.camLeft.parm2 -  cs.camRight.parm2) - lane_width)
      half_width = lane_width / 2
    half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.48))
  else:
    half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.47))

  l_prob = abs(l_prob)
  r_prob = abs(r_prob)
  lr_prob = (l_prob + r_prob) - l_prob * r_prob
  a_prob = 1 

  left_probs[:,0] =     l_prob * np.clip(model_output[:,3], 0, 1) + (1 - l_prob) * left_probs[:,0]
  right_probs[:,0] =    r_prob * np.clip(model_output[:,4], 0, 1) + (1 - r_prob) * right_probs[:,0]
  left_center[:,0:]  =  l_prob *   (descaled_output[0][:,1:2] - half_width) + (1 - l_prob) * left_center[:, 0:]
  right_center[:,0:]  = r_prob *   (descaled_output[0][:,2:3] + half_width) + (1 - r_prob) * right_center[:, 0:] 
  left_center =         l_prob * left_center + (1 - l_prob) * calc_center
  right_center =        r_prob * right_center + (1 - r_prob) * calc_center 
  
  calc_center = (l_prob * left_center + r_prob * right_center) / (l_prob + r_prob + 0.0005) 

  center_rate = calc_center - calc_center_prev
  center_accel = center_rate - center_rate_prev
  projected_center = calc_center + (2 * lr_prob * lr_prob_prev * center_rate) + (2 * lr_prob * lr_prob_prev * lr_prob_prev_prev * center_accel)
  
  #if recv_frames % 30 == 0:
  #  print(np.round(calc_center[::5,0], 1), np.round(center_rate[::5,0], 1), np.round(center_accel[::5,0],1),np.round(projected_center[::5,0], 1))

  lr_prob_prev_prev = lr_prob_prev
  lr_prob_prev = lr_prob
  calc_center_prev = calc_center
  center_rate_prev = center_rate

  if abs(cs.steeringTorque) < 1200 and abs(cs.adjustedAngle) < 30:
    upper_limit = one_deg_per_sec * cs.vEgo # * (1 + 0.1 * max(0, abs(cs.adjustedAngle)-4) + accel_counter)
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
      angle = np.clip((descaled_output[0][:,0:1] - descaled_output[0][0,0:1]) * (1 + advanceSteer), lower_limit, upper_limit)
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
  #lateral_offset *= use_lateral_offset

  path_send.pathPlan.angleSteers = float(angle[5] + cs.steeringAngle  - angle_bias)
  path_send.pathPlan.mpcAngles = [float(x) for x in (angle_factor * (angle[:] + descaled_output[0][0,0:1]) - total_offset - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  #path_send.pathPlan.mpcAngles = [float(x) for x in (angle + cs.steeringAngle  - angle_bias)]   #angle_steers.pop(output_list[-1]))]
  path_send.pathPlan.laneWidth = float(lane_width)
  path_send.pathPlan.angleOffset = total_offset
  #path_send.pathPlan.lateralOffset = float(lateral_offset)      
  path_send.pathPlan.lPoly = [float(x) for x in (left_center[:,0] + half_width)]
  path_send.pathPlan.rPoly = [float(x) for x in (right_center[:,0] - half_width)]
  path_send.pathPlan.cPoly = [float(x) for x in (projected_center[:,0])]
  path_send.pathPlan.lProb = float(l_prob)
  path_send.pathPlan.rProb = float(r_prob)
  path_send.pathPlan.cProb = float(lr_prob)
  path_send.pathPlan.canTime = cs.canTime
  gernPath.send(path_send.to_bytes())

  path_send = log.Event.new_message()
  path_send.init('pathPlan')
  if frame % 30 == 0:
    #try:
    print('half_width: %0.1f center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  total_offset:  %0.2f  angle_bias:  %0.2f' % (half_width, calc_center[-1], l_prob, r_prob, total_offset, angle_bias))
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
