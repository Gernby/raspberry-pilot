#!/usr/bin/env python
import os
import zmq
import time
import json
import joblib
import numpy as np

INPUTS = 78
OUTPUTS = 9
MODEL_VERSION = 'F'
MODEL_NAME = ''
output_standard = joblib.load(os.path.expanduser('models/GRU_Stand_%d_output_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
output_scaler = joblib.load(os.path.expanduser('models/GRU_MaxAbs_%d_output_%s.scaler' % (OUTPUTS, MODEL_VERSION)))
vehicle_standard = joblib.load(os.path.expanduser('models/GRU_Stand_%d_vehicle_%s.scaler' % (12, MODEL_VERSION)))
vehicle_scaler = joblib.load(os.path.expanduser('models/GRU_MaxAbs_%d_vehicle_%s.scaler' % (12, MODEL_VERSION)))
camera_standard = joblib.load(os.path.expanduser('models/GRU_Stand_%d_camera_%s.scaler' % (32, MODEL_VERSION)))
camera_scaler = joblib.load(os.path.expanduser('models/GRU_MaxAbs_%d_camera_%s.scaler' % (32, MODEL_VERSION)))

from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from selfdrive.car.honda.values import CAR
from enum import Enum
from cereal import log, car
from setproctitle import setproctitle
from common.params import Params, put_nonblocking
from common.profiler import Profiler
from tensorflow.python.keras.models import load_model 

setproctitle('transcoderd')

params = Params()
profiler = Profiler(False, 'transcoder')
#kegman = kegman_conf()  

BIT_MASK = [1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8] 

#import sys
#sys.stderr = open('../laterald.txt', 'w')
history_rows = []
for filename in os.listdir('models/'):
  if filename[-5:] == '.hdf5':
    if os.path.exists('models/models.json'):
      models = []
      with open('models/models.json', 'r') as f:
        models = []
        for md in json.load(f)['models']:
          models.append(load_model(os.path.expanduser('models/%s' % md)))
          history_rows.append(models[-1].layers[0].input.shape[1])
          print("loaded %s" % md)
      break
    elif MODEL_NAME == '':
      MODEL_NAME = filename
      models = [load_model(os.path.expanduser('models/%s' % (MODEL_NAME)))]
      history_rows = [models[-1].layers[0].input.shape[1]]
    else:
      print("\n\n   More than one model found!  Exiting!\n\n")
      exit()
     #[0]  #'Model-100-2-2-3-3'
print('loading model: %s' % MODEL_NAME)
models[-1].summary()

#HISTORY_ROWS = models[-1].layers[0].input.shape[1]
OUTPUT_ROWS = 15
BATCH_SIZE = 1
MAX_CENTER_OPPOSE = np.reshape(np.arange(15) * 200, (OUTPUT_ROWS,1))

lo_res_data = np.zeros((BATCH_SIZE,history_rows[-1], INPUTS-6))

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

def project_error(cpoly):
  peaks = np.sort([np.argmin(cpoly[:,0]), np.argmax(cpoly[:,0])])
  if cpoly[peaks[0],0] < cpoly[peaks[1],0]:
    peak_slope = peaks[0] + np.argmax(np.diff(cpoly[peaks[0]:peaks[1]+1,0]))
  else:
    peak_slope = peaks[0] + np.argmin(np.diff(cpoly[peaks[0]:peaks[1]+1,0]))
  for i in range(peak_slope+2, len(cpoly)):
    cpoly[i,0] = 2 * cpoly[i-1,0] - cpoly[i-2,0]
  return cpoly

def tri_blend(l_prob, r_prob, lr_prob, tri_value, steer, angle, prev_center, minimize=False, optimize=False, project=False):
  center = tri_value[:,0:1]
  left = l_prob * tri_value[:,1:2] + (1 - l_prob) * center
  right = r_prob * tri_value[:,2:3] + (1 - r_prob) * center
  if minimize:
    abs_left = max(0.00001, np.sum(np.absolute(left)))
    abs_right = max(0.00001,  np.sum(np.absolute(right)))
    centers = [(abs_right * left + abs_left * right) / (abs_left + abs_right), tri_value[:,1:2], tri_value[:,2:3]]
  else:
    centers = [0.5 * left + 0.5 * right, tri_value[:,1:2], tri_value[:,2:3]]
  if project:
    centers[0] = project_error(centers[0])
  return centers


def update_calibration(calibration, inputs, cal_col, cs):
  cal_speed = cs.vEgo * 0.00001
  far_left_factor = min(cal_speed, cs.camFarLeft.parm4)
  far_right_factor = min(cal_speed, cs.camFarRight.parm4)
  left_factor = min(cal_speed, cs.camLeft.parm4)
  right_factor = min(cal_speed, cs.camRight.parm4)
  cal_factor = [cal_speed,cal_speed,cal_speed,cal_speed,cal_speed,cal_speed,cal_speed,far_left_factor,far_left_factor,far_left_factor,far_right_factor,far_right_factor,far_right_factor,left_factor,left_factor,left_factor,right_factor,right_factor,right_factor]
  for i in range(len(cal_col)):
    calibration[i] += (cal_factor[i] * (inputs[cal_col[i]] - calibration[i]))
  return cal_factor

gernPath = pub_sock(service_list['pathPlan'].port)
carState = sub_sock(service_list['carState'].port, conflate=False)

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
angle_limit = 0.0
next_params_put = 3600

model_output = None
start_time = time.time()

#['Civic','CRV_5G','Accord_15','Insight', 'Accord']
fingerprint = np.zeros((1, 10), dtype=np.int)
model_output = models[-1].predict_on_batch([lo_res_data[  :,:,:6], lo_res_data[  :,:,:-16],lo_res_data[  :,:,-16:-8], lo_res_data[  :,:,-8:], fingerprint])

print(model_output.shape)
while model_output.shape[2] > output_scaler.max_abs_.shape[0]:
  print("adding column")
  print(output_standard.mean_.shape, output_scaler.scale_.shape) 
  try:
    output_standard.n_features_in_ += 1
  except:
    output_standard['n_features_in_'] = output_standard.mean_.shape[0] + 1
    pass
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
for md in range(len(models)):
  model_output = models[md].predict_on_batch([lo_res_data[  :,-history_rows[md]:,:6], lo_res_data[  :,-history_rows[md]:,:-16],lo_res_data[  :,-history_rows[md]:,-16:-8], lo_res_data[  :,-history_rows[md]:,-8:], fingerprint])
print(model_output)
print(history_rows)

l_prob = 0.0
r_prob = 0.0
lateral_adjust = 0
frame = 0
dump_sock(carState, True)

calibration_items = ['angle_steers','lateral_accelleration','yaw_rate_can','angle_steers2','lateral_accelleration2','yaw_rate_can2','far_left_1','far_left_7','far_left_9','far_right_1','far_right_7','far_right_9','left_1','left_7','left_9','right_1','right_7','right_9']
all_items = ['v_ego','angle_steers','lateral_accelleration','angle_rate', 'angle_rate_eps', 'yaw_rate_can','v_ego','long_accel', 'lane_width','angle_steers2','lateral_accelleration2','yaw_rate_can2','l_blinker','r_blinker',
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

stock_cam_frame_prev = -1
combine_flags = 1
vehicle_array = []
first_model = 0
last_model = len(models)-1
model_factor = 0.5
model_index = 0
params = None

while 1:
  vehicle_array = list(vehicle_array)
  for _cs in carState.recv_multipart():
    start_time = time.time()  
    profiler.checkpoint('inputs_recv', False)

    cs = log.Event.from_bytes(_cs).carState

    #TO DO: Split hi and low res control scalers
    vehicle_array.append([cs.vEgo, cs.steeringAngle, cs.lateralAccel, cs.steeringRate, cs.steeringTorqueEps, cs.yawRateCAN, cs.vEgo, cs.longAccel,  max(570, lane_width + width_trim), cs.steeringAngle, cs.lateralAccel, cs.yawRateCAN])

    if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
      stock_cam_frame_prev = cs.camLeft.frame

      left_missing = 1 if cs.camLeft.parm4 == 0 else 0
      far_left_missing = 1 if cs.camFarLeft.parm4 == 0 else 0
      right_missing = 1 if cs.camRight.parm4 == 0 else 0
      far_right_missing = 1 if cs.camFarRight.parm4 == 0 else 0
      
      camera_flags = np.bitwise_and([left_missing,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm8, 
                                    far_left_missing,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm8, 
                                    right_missing,     cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm8, 
                                    far_right_missing, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm8], BIT_MASK)
      
      if combine_flags:
        for i in range(2):
          camera_flags[3+i*16] += (camera_flags[2+i*16] + camera_flags[1+i*16])
          camera_flags[1+i*16] == 0
          camera_flags[2+i*16] == 0

      camera_input = np.concatenate(([0, 0], np.minimum(1, camera_flags), 
                                     [cs.camFarLeft.parm10,  cs.camFarLeft.parm2,  cs.camFarLeft.parm1,  cs.camFarLeft.parm3,  cs.camFarLeft.parm4,  cs.camFarLeft.parm5,  cs.camFarLeft.parm7,  cs.camFarLeft.parm9], 
                                     [cs.camFarRight.parm10, cs.camFarRight.parm2, cs.camFarRight.parm1, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm7, cs.camFarRight.parm9],
                                     [cs.camLeft.parm10,     cs.camLeft.parm2,     cs.camLeft.parm1,     cs.camLeft.parm3,     cs.camLeft.parm4,     cs.camLeft.parm5,     cs.camLeft.parm7,     cs.camLeft.parm9],    
                                     [cs.camRight.parm10,    cs.camRight.parm2,    cs.camRight.parm1,    cs.camRight.parm3,    cs.camRight.parm4,    cs.camRight.parm5,    cs.camRight.parm7,    cs.camRight.parm9]),axis=0)
    profiler.checkpoint('process_inputs')

  l_prob =     min(1, max(0, cs.camLeft.parm4 / 127))
  r_prob =     min(1, max(0, cs.camRight.parm4 / 127))
  lr_prob =    (l_prob + r_prob) - l_prob * r_prob

  vehicle_array = np.array(vehicle_array[-history_rows[-1]:])
  #if cs.vEgo > 10 and abs(cs.steeringAngle - calibration[0]) <= 3 and abs(cs.steeringRate) < 3 and l_prob > 0 and r_prob > 0:
  #  cal_factor = update_calibration(calibration, np.concatenate((vehicle_array[-1], camera_input), axis=0), cal_col, cs)

  for i in range(len(cal_col)):
    if cal_factor[i] > 0:
      if cal_col[i] < vehicle_array.shape[1]:
        vehicle_array[:,cal_col[i]] -= calibration[i]
      else:
        camera_input[cal_col[i]-12] -= calibration[i]
        #print(all_items[11:][cal_col[i]-11], calibration_items[i])

      # TO DO: Fix this hack
      if i in [0,4,5]: 
        vehicle_array[:,cal_col[i]] /= angle_factor

  profiler.checkpoint('calibrate')
    
  #try:
  hi_res_data = vehicle_scaler.transform(vehicle_standard.transform(vehicle_array[-history_rows[-1]:]))
  lo_res_data[:-1,:] = lo_res_data[1:,:]
  lo_res_data[-1,:] = np.concatenate(([hi_res_data[-1,6:]], [camera_input[:-32]], camera_scaler.transform(camera_standard.transform([camera_input[-32:]]))), axis=1)
  profiler.checkpoint('scale')
  if lr_prob == 0:
    model_index = last_model
  elif cs.steeringPressed:
    model_index = first_model
  else:
    model_index = max(model_index - 1, first_model, min(model_index + 1, last_model, int(abs(cs.steeringAngle - calibration[0]) * model_factor)))
  model_output = models[model_index].predict_on_batch([np.array([hi_res_data[-history_rows[model_index]:,:6]]), lo_res_data[:,-history_rows[model_index]:,:-16], lo_res_data[:,-history_rows[model_index]:,-16:-8], lo_res_data[:,-history_rows[model_index]:,-8:], fingerprint])
  profiler.checkpoint('predict')

  descaled_output = output_standard.inverse_transform(output_scaler.inverse_transform(model_output[-1])) 
  profiler.checkpoint('scale')

  max_width_step = 0.05 * cs.vEgo * l_prob * r_prob
  lane_width = max(570, lane_width - max_width_step * 2, min(1200, lane_width + max_width_step, cs.camLeft.parm2 - cs.camRight.parm2))
  
  calc_center = tri_blend(l_prob, r_prob, lr_prob, descaled_output[:,angle_speed_count::3], cs.torqueRequest, cs.steeringAngle - calibration[0], calc_center[0], minimize=use_minimize, optimize=use_optimize)
  
  '''if cs.vEgo > 10 and l_prob > 0 and r_prob > 0:	
    if calc_center[1][0,0] > calc_center[2][0,0]:	
      width_trim += 1	
    else:	
      width_trim -= 1	
    width_trim = max(-100, min(width_trim, 0))'''

  fast_angles = []
  if use_discrete_angle:
    fast_angles = angle_factor * descaled_output[:,:angle_speed_count] + calibration[0]
    if angle_limit < 1: 
      relative_angles = angle_factor * advanceSteer * (descaled_output[:,:angle_speed_count] - descaled_output[0,:angle_speed_count]) + cs.steeringAngle
      fast_angles = np.clip(fast_angles, relative_angles - angle_limit, relative_angles + angle_limit)
  else:
    fast_angles = angle_factor * advanceSteer * (descaled_output[:,:angle_speed_count] - descaled_output[0,:angle_speed_count]) + cs.steeringAngle
    if angle_limit < 1 or abs(cs.steeringAngle) > 30: 
      discrete_angles = angle_factor * descaled_output[:,:angle_speed_count] + calibration[0]
      fast_angles = np.clip(fast_angles, discrete_angles - angle_limit, discrete_angles + angle_limit)
  
  fast_angles = np.transpose(fast_angles)

  '''if abs(cs.steeringRate) < 3 and abs(cs.steeringAngle - calibration[0]) < 3 and cs.vEgo > 10 and cs.torqueRequest != 0:
    if fast_angles[0][0] - angle_bias > cs.steeringAngle:
      angle_bias += (0.00001 * cs.vEgo)
    else:
      angle_bias -= (0.00001 * cs.vEgo)'''

  if abs(cs.steeringRate) < 3 and abs(cs.steeringAngle - calibration[0]) < 3 and cs.torqueRequest != 0 and l_prob > 0 and r_prob > 0 and cs.vEgo > 10:
    if calc_center[0][3,0] > 0:
      angle_bias -= (0.00001 * cs.vEgo)
    elif calc_center[0][3,0] < 0:
      angle_bias += (0.00001 * cs.vEgo)

  profiler.checkpoint('process')
  path_send.pathPlan.centerCompensation = 0
  path_send.pathPlan.angleSteers = float(fast_angles[0][5])
  path_send.pathPlan.fastAngles = [[float(x) - angle_bias for x in y] for y in fast_angles]
  path_send.pathPlan.laneWidth = float(lane_width + width_trim)
  path_send.pathPlan.angleOffset = float(calibration[0])
  path_send.pathPlan.angleBias = angle_bias
  path_send.pathPlan.modelIndex = model_index
  path_send.pathPlan.paramsValid = calibrated
  path_send.pathPlan.cPoly = [float(x) for x in (calc_center[0][:,0])]
  path_send.pathPlan.lPoly = [float(x) for x in (calc_center[1][:,0] + 0.5 * lane_width)]
  path_send.pathPlan.rPoly = [float(x) for x in (calc_center[2][:,0] - 0.5 * lane_width)]
  path_send.pathPlan.lProb = float(l_prob)
  path_send.pathPlan.rProb = float(r_prob)
  path_send.pathPlan.cProb = float(lr_prob)
  path_send.pathPlan.canTime = cs.canTime
  path_send.pathPlan.sysTime = cs.sysTime 
  gernPath.send(path_send.to_bytes())
  profiler.checkpoint('send')
  
  frame += 1

  path_send = log.Event.new_message()
  path_send.init('pathPlan')
  profiler.checkpoint('log_init')

  if cs.vEgo > 10 and abs(cs.steeringAngle - calibration[0]) <= 3 and abs(cs.steeringRate) < 3 and l_prob > 0 and r_prob > 0:
    cal_factor = update_calibration(calibration, np.concatenate((vehicle_array[-1], camera_input), axis=0), cal_col, cs)

  if frame % 60 == 0:
    #print(calibration_factor, np.round(calibration, 2))
    print('lane_width: %0.1f angle bias: %0.2f  lateral_offset:  %d   center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  l_offset:  %0.2f  r_offset:  %0.2f  model_angle:  %0.2f  model_center_offset:  %0.2f  model exec time:  %0.4fs  angle_speed:  %0.1f' % (lane_width, angle_bias, lateral_adjust, calc_center[0][-1], l_prob, r_prob, cs.camLeft.parm2, cs.camRight.parm2, descaled_output[1,0], descaled_output[1,1], execution_time_avg, angle_speed))
  elif frame % 15 == 1:
    print("model_index: %d  angle: %0.1f  history_rows: %d" % (model_index, cs.steeringAngle, history_rows[model_index]))

  if frame > next_params_put and ((cs.vEgo < 10 and cs.brakePressed) or not calibrated):
    next_params_put = frame + 3000
    print(np.round(calibration,2))
    put_nonblocking("CalibrationParams", json.dumps({'calibration': list(calibration),'lane_width': lane_width,'angle_bias': angle_bias}))
    #os.remove(os.path.expanduser('~/calibration.json'))
    profiler.checkpoint('save_cal')

  # TODO: replace kegman_conf with params!
  if frame % 100 == 0:
    (mode, ino, dev, nlink, uid, gid, size, atime, mtime, kegtime) = os.stat(os.path.expanduser('~/kegman.json'))
    if kegtime != kegtime_prev:
      kegtime_prev = kegtime
      kegman = kegman_conf()  
      advanceSteer = 1.0 + max(0, float(kegman.conf['advanceSteer']))
      angle_factor = float(kegman.conf['angleFactor'])
      angle_speed = min(5, max(0, int(10 * float(kegman.conf['polyReact']))))
      use_discrete_angle = True if float(kegman.conf['discreteAngle']) > 0 else False
      angle_limit = abs(float(kegman.conf['discreteAngle']))
      use_minimize = True if kegman.conf['useMinimize'] == '1' else False
      first_model = max(0, min(len(models)-1, int(float(kegman.conf['firstModel']))))
      last_model = max(first_model, min(len(models)-1, int(float(kegman.conf['lastModel']))))
      model_factor = abs(float(kegman.conf['modelFactor']))
  
    profiler.checkpoint('kegman')
      
  execution_time_avg += max(0.0001, time_factor) * ((time.time() - start_time) - execution_time_avg)
  time_factor *= 0.96

  if frame % 1000 == 0 and profiler.enabled:
    profiler.display()
    profiler.reset(True)

  #except:
  #  pass
