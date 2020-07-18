#!/usr/bin/env python
import os
import zmq
import time
import json
import joblib
import numpy as np

INPUTS = 77
OUTPUTS = 9
MODEL_VERSION = 'F'
MODEL_NAME = ''
output_standard = joblib.load(os.path.expanduser('models/GRU_Stand_%d_output_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
output_scaler = joblib.load(os.path.expanduser('models/GRU_MaxAbs_%d_output_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
vehicle_standard = joblib.load(os.path.expanduser('models/GRU_Stand_%d_vehicle_%s.scaler' % (11, MODEL_VERSION)))
vehicle_scaler = joblib.load(os.path.expanduser('models/GRU_MaxAbs_%d_vehicle_%s.scaler' % (11, MODEL_VERSION)))
camera_standard = joblib.load(os.path.expanduser('models/GRU_Stand_%d_camera_%s.scaler' % (32, MODEL_VERSION)))
camera_scaler = joblib.load(os.path.expanduser('models/GRU_MaxAbs_%d_camera_%s.scaler' % (32, MODEL_VERSION)))

from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from selfdrive.car.honda.values import CAR
from enum import Enum
from cereal import log, car
from setproctitle import setproctitle
from common.params import Params 
from tensorflow.python.keras.models import load_model 

setproctitle('transcoderd')

params = Params()
#kegman = kegman_conf()  

#import sys
#sys.stderr = open('../laterald.txt', 'w')
for filename in os.listdir('models/'):
  if filename[-5:] == '.hdf5':
    if MODEL_NAME == '':
      MODEL_NAME = filename
    else:
      print("\n\n   More than one model found!  Exiting!\n\n")
      exit()
     #[0]  #'Model-100-2-2-3-3'
print('loading model: %s' % MODEL_NAME)
HISTORY_ROWS = 5
OUTPUT_ROWS = 15
BATCH_SIZE = 1
MAX_CENTER_OPPOSE = np.reshape(np.arange(15) * 200, (OUTPUT_ROWS,1))

model = load_model(os.path.expanduser('models/%s' % (MODEL_NAME)))
new_input = np.zeros((BATCH_SIZE,HISTORY_ROWS, INPUTS))
model_input = new_input
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

def tri_blend(l_prob, r_prob, lr_prob, tri_value, steer, angle, prev_center, minimize=False, optimize=False):
  left = l_prob * tri_value[:,1:2] + (1 - l_prob) * tri_value[:,0:1]
  right = r_prob * tri_value[:,2:3] + (1 - r_prob) * tri_value[:,0:1]
  center = tri_value[:,0:1]
  if minimize:
    abs_left = np.sum(np.absolute(left))
    abs_right = np.sum(np.absolute(right))
  else:
    abs_left = 1
    abs_right = 1     

  if optimize and abs(angle) < 5:
    weighted_center = [(abs_right * l_prob * left + abs_left * r_prob * right) / (abs_right * l_prob + abs_left * r_prob + 0.0001), left, right]
    if steer > 0:
      weighted_center[0] = np.maximum(weighted_center[0], center)
    elif steer < 0:
      weighted_center[0] = np.minimum(weighted_center[0], center)
  else:
    weighted_center = [((lr_prob * (abs_right * left + abs_left * right) / (abs_right + abs_left + 0.0001)) + ((1-lr_prob) * center)), left, right]
    #weighted_center = [(lr_prob * (abs_right * l_prob * left + abs_left * r_prob * right) / (abs_right * l_prob + abs_left * r_prob + 0.0001) + (1-lr_prob) * center), left, right]
  return weighted_center


gernPath = pub_sock(service_list['pathPlan'].port)
gernModelInputs = sub_sock(service_list['model'].port, conflate=True)

frame_count = 1
dashboard_count = 0
lane_width = 0
half_width = 0
width_trim = 0
angle_bias = 0.0
total_offset = 0.0
path_send = log.Event.new_message()
path_send.init('pathPlan')
advanceSteer = 1
one_deg_per_sec = np.ones((OUTPUT_ROWS,1)) / 15
left_center = np.zeros((OUTPUT_ROWS,1))
right_center = np.zeros((OUTPUT_ROWS,1))
calc_center = np.zeros((3,OUTPUT_ROWS,1))
projected_center = np.zeros((OUTPUT_ROWS,1))
left_probs = np.zeros((OUTPUT_ROWS,1))
right_probs = np.zeros((OUTPUT_ROWS,1))
fast_angles = np.zeros((OUTPUT_ROWS,1))
center_limit = np.reshape(0.5 * np.arange(OUTPUT_ROWS) + 10,(OUTPUT_ROWS,1))
accel_counter = 0   
upper_limit = 0
lower_limit = 0
lr_prob_prev = 0
lr_prob_prev_prev = 0
center_rate_prev = 0
calc_center_prev = calc_center
angle_factor = 1.0
angle_speed = 3
use_discrete_angle = True
use_optimize = True
use_minimize = False

execution_time_avg = 0.0
time_factor = 1.0
lateral_offset = 0
calibration_factor = 1.0

model_output = None
start_time = time.time()

#['Civic','CRV_5G','Accord_15','Insight', 'Accord']
fingerprint = np.zeros((1, 10), dtype=np.int)
model_output = model.predict_on_batch([new_input[  :,:,:5], new_input[  :,:,5:-16],new_input[  :,:,-16:-8], new_input[  :,:,-8:], fingerprint])

print(model_output.shape)
while model_output.shape[2] > output_scaler.max_abs_.shape[0]:
  print("adding column")
  print(output_standard.mean_.shape, output_scaler.scale_.shape) 
  output_standard.n_features_in_ += 1
  output_scaler.max_abs_ = np.concatenate((output_scaler.max_abs_[:1], output_scaler.max_abs_),axis=0)
  output_scaler.scale_ = np.concatenate((output_scaler.scale_[:1], output_scaler.scale_),axis=0)
  output_standard.mean_ = np.concatenate((output_standard.mean_[:1], output_standard.mean_),axis=0)
  output_standard.scale_ = np.concatenate((output_standard.scale_[:1], output_standard.scale_),axis=0)
  output_standard.var_ = np.concatenate((output_standard.var_[:1], output_standard.var_),axis=0)

descaled_output = output_standard.transform(output_scaler.inverse_transform(model_output[-1]))
print(descaled_output)

car_params = car.CarParams.from_bytes(params.get('CarParams', True))

if car_params.carFingerprint == CAR.CIVIC_BOSCH:
  index_finger = 0
elif car_params.carFingerprint in [CAR.CRV_5G, CAR.CRV_HYBRID]:
  index_finger = 1
elif car_params.carFingerprint in [CAR.ACCORD_15, CAR.ACCORD, CAR.ACCORDH]:
  index_finger = 2
elif car_params.carFingerprint == CAR.INSIGHT:
  index_finger = 3

fingerprint[:,index_finger] = 1
if not os.path.exists(os.path.expanduser('~/vehicle_option.json')):
  with open(os.path.expanduser('~/vehicle_option.json'), 'w') as f:
    json.dump({'vehicle_option': 6}, f, indent=2, sort_keys=False)
with open(os.path.expanduser('~/vehicle_option.json'), 'r') as f:
  vehicle_option = json.load(f)
  fingerprint[:,3+vehicle_option['vehicle_option']] = 1

print(fingerprint, vehicle_option)
model_output = model.predict_on_batch([new_input[  :,:,:5], new_input[  :,:,5:-16],new_input[  :,:,-16:-8], new_input[  :,:,-8:], fingerprint])
print(model_output)

l_prob = 0.0
r_prob = 0.0
lateral_adjust = 0
frame = 0
dump_sock(gernModelInputs, True)

calibration_items = ['angle_steers','lateral_accelleration','angle_rate_eps', 'yaw_rate_can','angle_steers2','lateral_accelleration2','yaw_rate_can2','far_left_1','far_left_7','far_left_9','far_right_1','far_right_7','far_right_9','left_1','left_7','left_9','right_1','right_7','right_9']
all_items = ['v_ego','angle_steers','lateral_accelleration','angle_rate_eps', 'yaw_rate_can','v_ego','long_accel', 'lane_width','angle_steers2','lateral_accelleration2','yaw_rate_can2','l_blinker','r_blinker',
            'left_missing','l6b_6','l6b_6','l6b_6','l6b_6','l6b_6','l6b_6','l8b_8',
            'far_left_missing','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl8b_8',
            'right_missing','r6b_6','r6b_6','r6b_6','r6b_6','r6b_6','r6b_6','r8b_8',
            'far_right_missing','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr8b_8',
            'far_left_10', 'far_left_2',  'far_left_1',  'far_left_3',  'far_left_4',  'far_left_5',  'far_left_7',  'far_left_9',  
            'far_right_10','far_right_2', 'far_right_1', 'far_right_3', 'far_right_4', 'far_right_5', 'far_right_7', 'far_right_9', 
            'left_10',     'left_2',      'left_1',      'left_3',      'left_4',      'left_5',      'left_7',      'left_9',      
            'right_10',    'right_2',     'right_1',     'right_3',     'right_4',     'right_5',     'right_7',     'right_9']     
cal_col = np.zeros((len(calibration_items)),dtype=np.int)
cal_factor = np.zeros((len(calibration_items)),dtype=np.int)
for col in range(len(calibration_items)):
  cal_col[col] = all_items.index(calibration_items[col])
print(cal_col)

adj_items =  ['far_left_2','far_right_2','left_2','right_2']
adj_col = np.zeros((len(adj_items)),dtype=np.int)
for col in range(len(adj_items)):
  adj_col[col] = all_items.index(adj_items[col])
print(adj_col)
#             [46, 54, 62, 70]
kegtime_prev = 0
angle_speed_count = model_output.shape[2] - 7
#(mode, ino, dev, nlink, uid, gid, size, atime, mtime, kegtime_prev) = os.stat(os.path.expanduser('~/kegman.json'))


'''for i in range(0, 9, 3):
  print("mean: ", output_standard.mean_[i])
  output_standard.mean_[i] += 100
  print("mean: ", output_standard.mean_[i])'''

try:
  car_params = params.get('CarParams')
  print(car_params)
  #if 'Accord' in car_params['fingerprint']:
    
  calibrated = True
  calibration_data = params.get("CalibrationParams")
  calibration_data =  json.loads(calibration_data)
  calibration = np.array(calibration_data['calibration'])
  if len(calibration) != len(cal_col):
    with open(os.path.expanduser('~/calibration.json'), 'r') as f:
      calibration_data = json.load(f)
      calibration = np.array(calibration_data['calibration'])
      if len(calibration) != len(cal_col):
        calibration = np.zeros(len(cal_col))
        calibrated = False
        print("resetting calibration")
  lane_width = calibration_data['lane_width']
  angle_bias = calibration_data['angle_bias']
  print(calibration)
except:
  # TODO: Remove this after user's calibrations have been moved to params
  try:
    with open(os.path.expanduser('~/calibration.json'), 'r') as f:
      calibration_data = json.load(f)
      calibration = np.array(calibration_data['calibration'])
      if len(calibration) != len(cal_col):
        calibration = np.zeros(len(cal_col))
        calibrated = False
        print("resetting calibration")
    lane_width = calibration_data['lane_width']
    angle_bias = calibration_data['angle_bias']
  except:
    calibrated = False
    print("resetting calibration")
    calibration = np.zeros(len(cal_col))

while 1:
  cs = car.CarState.from_bytes(gernModelInputs.recv())
  start_time = time.time()  
  new_input = np.asarray(cs.modelData).reshape(1, HISTORY_ROWS, INPUTS)
  l_prob = min(1, max(0, cs.camLeft.parm4 / 127))
  r_prob = min(1, max(0, cs.camRight.parm4 / 127))
  lr_prob = (l_prob + r_prob) - l_prob * r_prob

  if cs.vEgo > 10 and abs(cs.steeringAngle - calibration[0]) <= 3 and abs(cs.steeringRate) < 3 and l_prob > 0 and r_prob > 0:
    cal_speed = cs.vEgo * 0.00001
    far_left_factor = min(cal_speed, cs.camFarLeft.parm4)
    far_right_factor = min(cal_speed, cs.camFarRight.parm4)
    left_factor = min(cal_speed, cs.camLeft.parm4)
    right_factor = min(cal_speed, cs.camRight.parm4)
    cal_factor = [cal_speed,cal_speed,cal_speed,cal_speed,cal_speed,cal_speed,cal_speed,far_left_factor,far_left_factor,far_left_factor,far_right_factor,far_right_factor,far_right_factor,left_factor,left_factor,left_factor,right_factor,right_factor,right_factor]
    for i in range(len(cal_col)):
      calibration[i] += (cal_factor[i] * (new_input[0][-1:,cal_col[i]] - calibration[i]))
  for i in range(len(cal_col)):
    if cal_factor[i] > 0:
      new_input[-1,:,cal_col[i]] -= calibration[i]
  for i in range(4):
    if [cs.camFarLeft.parm4, cs.camFarRight.parm4, cs.camLeft.parm4, cs.camRight.parm4][i] > 0:
      new_input[-1:,:,adj_col[i]] += lateral_adjust

  new_input[-1,:,:11] = vehicle_scaler.transform(vehicle_standard.transform(new_input[-1,:,:11]))
  new_input[-1,-1:,-32:] = camera_scaler.transform(camera_standard.transform(new_input[-1,-1:,-32:]))
  new_input[-1,:-1,5:] = model_input[-1,1:,5:]
  model_input = new_input

  model_output = model.predict_on_batch([model_input[  :,:,:5], model_input[  :,:,5:-16],model_input[  :,:,-16:-8], model_input[  :,:,-8:], fingerprint])

  time.sleep(0.00001)
  descaled_output = output_standard.inverse_transform(output_scaler.inverse_transform(model_output[-1])) 
  
  max_width_step = 0.05 * cs.vEgo * l_prob * r_prob
  lane_width = max(570, lane_width - max_width_step * 2, min(1200, lane_width + max_width_step, cs.camLeft.parm2 - cs.camRight.parm2))
  
  calc_center = tri_blend(l_prob, r_prob, lr_prob, descaled_output[:,angle_speed_count::3], cs.torqueRequest, cs.steeringAngle - calibration[0], calc_center[0], minimize=use_minimize, optimize=use_optimize)

  if cs.vEgo > 10 and l_prob > 0 and r_prob > 0:
    if calc_center[1][0,0] > calc_center[2][0,0]:
      width_trim += 1
    else:
      width_trim -= 1
    width_trim = max(-100, min(width_trim, 0))
  
  if abs(cs.steeringRate) < 3 and abs(cs.steeringAngle - calibration[0]) < 3 and l_prob > 0 and r_prob > 0 and cs.vEgo > 10 and (cs.torqueRequest != 0 or not calibrated):
    if calc_center[0][0,0] > 0:
      angle_bias -= (0.000001 * cs.vEgo)
    elif calc_center[0][0,0] < 0:
      angle_bias += (0.000001 * cs.vEgo)

  fast_angles = []
  for i in range(angle_speed_count):
    if use_discrete_angle:
      fast_angles.append(angle_factor * descaled_output[:,i:i+1] + calibration[0] - angle_bias)
    else:
      fast_angles.append(angle_factor * advanceSteer * (descaled_output[:,i:i+1] - descaled_output[0,i:i+1]) + cs.steeringAngle - angle_bias)

  path_send.pathPlan.angleSteers = float(fast_angles[0][5])
  path_send.pathPlan.fastAngles = [[float(x) for x in y] for y in fast_angles]
  path_send.pathPlan.laneWidth = float(lane_width + width_trim)
  path_send.pathPlan.angleOffset = float(calibration[0])
  path_send.pathPlan.angleBias = angle_bias
  path_send.pathPlan.paramsValid = calibrated
  path_send.pathPlan.cPoly = [float(x) for x in (calc_center[0][:,0] + lateral_adjust)]
  path_send.pathPlan.lPoly = [float(x) for x in (calc_center[1][:,0] + 0.5 * lane_width)]
  path_send.pathPlan.rPoly = [float(x) for x in (calc_center[2][:,0] - 0.5 * lane_width)]
  path_send.pathPlan.lProb = float(l_prob)
  path_send.pathPlan.rProb = float(r_prob)
  path_send.pathPlan.cProb = float(lr_prob)
  path_send.pathPlan.canTime = cs.canTime
  path_send.pathPlan.sysTime = cs.sysTime
  gernPath.send(path_send.to_bytes())

  frame += 1

  path_send = log.Event.new_message()
  path_send.init('pathPlan')
  if frame % 60 == 0:
    #print(calibration_factor, np.round(calibration, 2))
    print('lane_width: %0.1f angle bias: %0.2f  width_trim: %0.1f  lateral_offset:  %d   center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  l_offset:  %0.2f  r_offset:  %0.2f  model_angle:  %0.2f  model_center_offset:  %0.2f  model exec time:  %0.4fs  angle_speed:  %0.1f' % (lane_width, angle_bias, width_trim, lateral_adjust, calc_center[0][-1], l_prob, r_prob, cs.camLeft.parm2, cs.camRight.parm2, descaled_output[1,0], descaled_output[1,1], execution_time_avg, angle_speed))

  if frame % 6000 == 0:
    print(np.round(calibration,2))
    params.put("CalibrationParams", json.dumps({'calibration': list(calibration),'lane_width': lane_width,'angle_bias': angle_bias}))
    #os.remove(os.path.expanduser('~/calibration.json'))

  # TODO: replace kegman_conf with params!
  if frame % 100 == 0:
    (mode, ino, dev, nlink, uid, gid, size, atime, mtime, kegtime) = os.stat(os.path.expanduser('~/kegman.json'))
    if kegtime != kegtime_prev:
      kegtime_prev = kegtime
      kegman = kegman_conf()  
      advanceSteer = 1.0 + max(0, float(kegman.conf['advanceSteer']))
      angle_factor = float(kegman.conf['angleFactor'])
      use_bias = float(kegman.conf['angleBias'])
      angle_speed = min(5, max(0, int(10 * float(kegman.conf['polyReact']))))
      use_angle_offset = float(kegman.conf['angleOffset'])
      lateral_offset = float(kegman.conf['lateralOffset'])
      use_discrete_angle = True if kegman.conf['useDiscreteAngle'] == "1" else False
      use_optimize = True if kegman.conf['useOptimize'] == '1' else False
      use_minimize = True if kegman.conf['useMinimize'] == '1' else False
      
  execution_time_avg += max(0.0001, time_factor) * ((time.time() - start_time) - execution_time_avg)
  time_factor *= 0.96
