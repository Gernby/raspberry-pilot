#!/usr/bin/env python
import os
import zmq
import time
import json
import numpy as np

INPUTS = 79
OUTPUTS = 9

try:
  from selfdrive.kegman_conf import kegman_conf
  from selfdrive.services import service_list
  from selfdrive.car.honda.values import CAR
  from cereal import log, car
  from setproctitle import setproctitle
  from common.params import Params, put_nonblocking
  from common.profiler import Profiler

  setproctitle('transcoderd')
  params = Params()
  profiler = Profiler(False, 'transcoder')
except:
  pass

import onnxruntime as ort

options = ort.SessionOptions()
options.intra_op_num_threads = 1
options.inter_op_num_threads = 1
#options.execution_mode = ort.ExecutionMode.ORT_PARALLEL
options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
provider = 'CPUExecutionProvider'

BIT_MASK = [1, 128, 64, 32, 8, 4, 2, 8,
            1, 128, 64, 32, 8, 4, 2, 8,
            1, 128, 64, 32, 8, 4, 2, 8,
            1, 128, 64, 32, 8, 4, 2, 8]
   
history_rows = []
OUTPUT_ROWS = 15
CENTER_POLYS = 5
ANGLE_POLYS = 5

PATH_COUNT = 5

fingerprint = np.zeros((1, 4), dtype='float32')
calc_center = [np.zeros((OUTPUT_ROWS, 4)),np.zeros((OUTPUT_ROWS, 4))]
calc_angles = [np.zeros((PATH_COUNT, OUTPUT_ROWS)), np.zeros((PATH_COUNT, OUTPUT_ROWS))]
angle_plan = np.zeros((1, OUTPUT_ROWS))
projected_rate = np.arange(0., 0.10066667,0.0066666)[1:]
previous_rate = 0.
CENTER_PROFILES = np.array([[  0.000,  0.000,  0.000,  0.000,  0.000],
                            [ -1.000, -1.000,  0.200,  0.100,  0.000],
                            [ -1.000, -1.000, -0.100,  0.050,  0.000],
                            [ -1.000, -1.000,  0.100, -0.050,  0.000],
                            [ -1.000, -1.000, -0.100, -0.050,  0.000]], 'float32') / np.array([1., 1., 25., 25., 1.], 'float32')

center_profiles = np.array(CENTER_PROFILES) * np.array([1., 1., 1., 1., 1.])

accel_profile = np.array([(np.ones((OUTPUT_ROWS), dtype='float32') * 100),
                          (np.ones((OUTPUT_ROWS), dtype='float32') * 100),
                          (np.ones((OUTPUT_ROWS), dtype='float32') * 100),
                          (np.ones((OUTPUT_ROWS), dtype='float32') * 100),
                          (np.ones((OUTPUT_ROWS), dtype='float32') * 100),
                          (np.arange(OUTPUT_ROWS, dtype='float32'))])
prev_angle_plans = np.zeros((6, OUTPUT_ROWS), dtype='float32')

for i in range(2, 7):
  accel_profile[-i,:-i+1] = accel_profile[-i+1,1:17-i] + accel_profile[-1,i-1:] * 0.8**(i-1)
accel_profile[-1:] *= 2
accel_limit = accel_profile

if os.path.exists('models/models.json'):
  with open('models/models.json', 'r') as f:
    models = []
    models_def = json.load(f)
    for md in models_def['models']:
      models.append(ort.InferenceSession(os.path.expanduser('models/%s' % md), options))
      models[-1].set_providers([provider], None)
      history_rows.append(15)
      for i in range(21):
        if i == 1:
          start_time = time.time()
        model_output = [models[-1].run(None, {'vehicle_inputs': np.zeros((1, 100, 13), dtype='float32')[:,-50:,:8], 
                                              'vehicle_inputs2': np.zeros((1, 15, 6), dtype='float32'),
                                              'left_flag_inputs': np.zeros((1, 15, 8), dtype='float32') + 0,
                                              'outer_left_flag_inputs': np.zeros((1, 15, 8), dtype='float32') + 0,
                                              'right_flag_inputs': np.zeros((1, 15, 8), dtype='float32') + 0,
                                              'outer_right_flag_inputs': np.zeros((1, 15, 8), dtype='float32') + 0,
                                              'outer_left_inputs': np.zeros((1, 15, 8), dtype='float32') + 0.01,
                                              'outer_right_inputs': np.zeros((1, 15, 8), dtype='float32') + 0.01,
                                              'left_inputs': np.zeros((1, 15, 8), dtype='float32') + 0.01,
                                              'right_inputs': np.zeros((1, 15, 8), dtype='float32') + 0.01,
                                              'fingerprints': [[[0,0,0,1]]],
                                              'center_profiles': [center_profiles * np.array([1., 1., 25., 25., 1.], 'float32')],
                                              'center_bias': np.array([[[0.21],[0.001],[0.003],[0.003],[-2100.2]]], dtype='float32') * np.array([[[0.00001], [0.00001], [0.00001], [0.00001], [0.00001]]], dtype='float32'),
                                              'model_bias': [np.ones((ANGLE_POLYS,1),dtype='float32') * 0],
                                            })]

        calc_angles[0] = np.transpose(model_output[0][0][0,:,:PATH_COUNT])

        angle_plan = np.clip(calc_angles[0], angle_plan - accel_limit[-1], angle_plan + accel_limit[-1])
        projected_steering = angle_plan[0,0] + (angle_plan[0,1] - angle_plan[0,0]) * projected_rate
        angle_plan = np.clip(angle_plan, projected_steering - accel_limit[-1], projected_steering + accel_limit[-1])
      
      print(len(model_output), len(model_output[0]), len(model_output[0][0]))
      print(np.array(100 * model_output[0][0][0,:,:PATH_COUNT], dtype='int32'))
      print()
      print(np.array(model_output[0][0][0,:,PATH_COUNT:], dtype='int32'))
      print(np.sum(np.absolute(model_output[0][0][0,10:,PATH_COUNT:]), axis=-2))
      print(np.argmin(np.sum(np.absolute(model_output[0][0][0,10:,PATH_COUNT:]), axis=-2)))
      print(time.time()-start_time, md)
      print(len(model_output), len(model_output[0]), len(model_output[0][0]), len(model_output[0][-1]))
      

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

def update_calibration(calibration, inputs, cal_col, cs):
  cal_speed = cs.vEgo * 0.00001
  far_left_factor = min(cal_speed, cs.camFarLeft.parm4)
  far_right_factor = min(cal_speed, cs.camFarRight.parm4)
  left_factor = min(cal_speed, cs.camLeft.parm4)
  right_factor = min(cal_speed, cs.camRight.parm4)
  cal_factor[0][(cal_col[0] == 1)] = [cal_speed,cal_speed,cal_speed]
  cal_factor[1][(cal_col[1] == 1)] = [cal_speed,cal_speed,cal_speed]
  cal_factor[3][(cal_col[3] == 1)] = [far_left_factor,far_left_factor,far_left_factor,far_right_factor,far_right_factor,far_right_factor,left_factor,left_factor,left_factor,right_factor,right_factor,right_factor]
  for i in [0,1]:
    calibration[i] += (cal_factor[i][(cal_col[i] == 1)] * (inputs[0][i][-1][-1][-1][(cal_col[i] == 1)] - calibration[i]))
  for i in [3]:
    calibration[i] += (cal_factor[i][(cal_col[i] == 1)] * (inputs[1][i-2][-1][-1][-1][(cal_col[i] == 1)] - calibration[i]))
  return cal_factor

def send_path_to_controls(model_index, calc_angles, calc_center, angle_plan, projected_steering, path_send, gernPath, cs, angle_bias, lane_width, width_trim, l_prob, r_prob, lr_prob, calibrated, c_poly, d_poly, steer_override_timer, something_masked):
  prev_angle_plans[-1,:] = projected_steering
  if steer_override_timer <= 15:
    angle_plan = np.clip(calc_angles[0], np.amax(prev_angle_plans - accel_limit, axis=-2), np.amin(prev_angle_plans + accel_limit, axis=-2))  
  else:
    angle_plan = np.clip(calc_angles[0], np.amax(prev_angle_plans - (0.5 * accel_limit), axis=-2), np.amin(prev_angle_plans + (0.5 * accel_limit), axis=-2))  

  prev_angle_plans[:-1,:-1] = prev_angle_plans[1:,1:]
  prev_angle_plans[-2,:] = angle_plan[0]
  p_poly = np.polyfit(np.linspace(0., 6., num=7), angle_plan[0][:7] / 54.938633, 4)

  path_send = log.Event.new_message()
  path_send.init('pathPlan')

  path_send.pathPlan.centerCompensation = 0
  path_send.pathPlan.angleSteers = float(angle_plan[0][5])
  path_send.pathPlan.fastAngles = [[float(x) for x in y] for y in angle_plan] 
  path_send.pathPlan.laneWidth = float(lane_width + width_trim)
  path_send.pathPlan.angleOffset = float(calibration[0][0])
  path_send.pathPlan.angleBias = angle_bias
  path_send.pathPlan.modelIndex = float(model_index)
  path_send.pathPlan.paramsValid = calibrated
  path_send.pathPlan.cPoly = [float(x) for x in c_poly]
  path_send.pathPlan.lPoly = [float(x) for x in c_poly]
  path_send.pathPlan.rPoly = [float(x) for x in c_poly]
  path_send.pathPlan.pPoly = [float(x) for x in p_poly]
  path_send.pathPlan.dPoly = [float(x) for x in d_poly]
  path_send.pathPlan.lProb = float(l_prob)
  path_send.pathPlan.rProb = float(r_prob)
  path_send.pathPlan.cProb = float(lr_prob)
  path_send.pathPlan.canTime = cs.canTime
  path_send.pathPlan.sysTime = cs.sysTime
  gernPath.send(path_send.to_bytes())

  return angle_plan

gernPath = pub_sock(service_list['pathPlan'].port)
carState = sub_sock(service_list['carState'].port, conflate=False)

width_trim = 0
angle_bias = 0
execution_time_avg = 0.027
time_factor = 1.0
next_params_distance = 13300.0
distance_driven = 0.0
steer_override_timer = 0
start_time = 0
os.system("taskset -a -cp --cpu-list 2,3 %d" % os.getpid())

path_send = log.Event.new_message()
path_send.init('pathPlan')
gernPath.send(path_send.to_bytes())
path_send = log.Event.new_message()
path_send.init('pathPlan')

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

l_prob = 0.0
r_prob = 0.0
lr_prob = 0.0
lateral_adjust = 0
frame = 0
dump_sock(carState, True)

calibration_items = [['angle_steers','lateral_accelleration','yaw_rate_can'],['angle_steers2','lateral_accelleration2','yaw_rate_can2'],[],['far_left_1','far_left_7','far_left_9','far_right_1','far_right_7','far_right_9','left_1','left_7','left_9','right_1','right_7','right_9']]
all_items = [['v_ego','angle_steers','lateral_accelleration','angle_rate', 'angle_rate_eps', 'yaw_rate_can','steering_torque','request'],['v_ego','long_accel', 'lane_width','angle_steers2','lateral_accelleration2','yaw_rate_can2'],
           ['l_blinker','r_blinker',
            'left_missing','l6b_6','l6b_6','l6b_6','l6b_6','l6b_6','l6b_6','l8b_8',
            'far_left_missing','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl8b_8',
            'right_missing','r6b_6','r6b_6','r6b_6','r6b_6','r6b_6','r6b_6','r8b_8',
            'far_right_missing','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr8b_8'],
           ['far_left_10', 'far_left_2',  'far_left_1',  'far_left_3',  'far_left_4',  'far_left_5',  'far_left_7',  'far_left_9',  
            'far_right_10','far_right_2', 'far_right_1', 'far_right_3', 'far_right_4', 'far_right_5', 'far_right_7', 'far_right_9', 
            'left_10',     'left_2',      'left_1',      'left_3',      'left_4',      'left_5',      'left_7',      'left_9',
            'right_10',    'right_2',     'right_1',     'right_3',     'right_4',     'right_5',     'right_7',     'right_9']]

l_angle_avg = calibration_items[3].index('left_1')
r_angle_avg = calibration_items[3].index('right_1')
fl_angle_avg = calibration_items[3].index('far_left_1')
fr_angle_avg = calibration_items[3].index('far_right_1')

cal_col = [np.zeros((len(all_items[0])),dtype=np.int),np.zeros((len(all_items[1])),dtype=np.int),[], np.zeros((len(all_items[3])),dtype=np.int)]
cal_factor = [np.zeros((len(all_items[0])),dtype='float32'),np.zeros((len(all_items[1])),dtype='float32'),[],np.zeros((len(all_items[3])),dtype='float32')]
for i in range(len(all_items)):
  for col in range(len(all_items[i])):
    #print(i,col)
    if len(cal_col[i]) > col:
      cal_col[i][col] = 1 if len(all_items[i]) > col and all_items[i][col] in calibration_items[i] else 0 

adj_items =  ['far_left_2','far_right_2','left_2','right_2']
adj_col = np.zeros((len(adj_items)),dtype=np.int)
for col in range(len(adj_items)):
  adj_col[col] = all_items[3].index(adj_items[col])

kegtime_prev = 0
model_bias = np.zeros((2,ANGLE_POLYS,1), 'float32')
center_bias = np.zeros((2,CENTER_POLYS,1), 'float32')
path_profile_scale = np.ones(PATH_COUNT-1, 'float32')
path_profile_counter = np.zeros(PATH_COUNT, 'float32')
path_profile_ratio = 1.
lane_width = 800.

calibrated = True
calibration_data = params.get("CalibrationParams")
partial_reset = False
if not calibration_data is None:
  calibration_data =  json.loads(calibration_data)
  calibration = np.array(calibration_data['calibration'], dtype='float32')
  if 'angle_bias' in calibration_data:
    angle_bias = calibration_data['angle_bias']
  if 'lane_width' in calibration_data:
    lane_width = calibration_data['lane_width']
  if not models_def["reset"] or ("models" in calibration_data and calibration_data['models'] == models_def['models']):
    if 'center_bias' in calibration_data and len(calibration_data['center_bias']) == 2 * 1 * CENTER_POLYS:
      center_bias = np.array(np.reshape(calibration_data['center_bias'], (2,CENTER_POLYS,1)), dtype='float32')
    if 'model_bias' in calibration_data and len(calibration_data['model_bias']) == 2 * 1 * ANGLE_POLYS:
      model_bias = np.array(np.reshape(calibration_data['model_bias'], (2,ANGLE_POLYS,1)), dtype='float32')
    if 'path_profile_scale' in calibration_data and len(calibration_data['path_profile_scale']) == (PATH_COUNT - 1):
      path_profile_scale = np.array(calibration_data['path_profile_scale'], 'float32')
  else:
    partial_reset = True
    os.system("cp /data/params/d/CalibrationParams /data/params/d/CalibrationParams%d" % int(time.time()))
    print("New models!  Resetting bias")

if calibration_data is None or len(calibration) != (len(calibration_items[0]) + len(calibration_items[1]) + len(calibration_items[3])):
  next_params_distance *= 10
  calibration = [np.zeros(len(calibration_items[0]), dtype='float32'), np.zeros(len(calibration_items[1]), dtype='float32'), [], np.zeros(len(calibration_items[3]), dtype='float32')]
  calibrated = False
  print("resetting calibration")
  params.delete("CalibrationParams")
else:
  calibration = [np.array(calibration[:len(calibration_items[0])], dtype='float32'), 
                np.array(calibration[len(calibration_items[0]):len(calibration_items[0])+len(calibration_items[1])], dtype='float32'), [], 
                np.array(calibration[len(calibration_items[0])+len(calibration_items[1]):], dtype='float32')]

print(calibration)

stock_cam_frame_prev = -1
vehicle_array = [[],[]]
camera_array = [[],[]]
actual_angles = []
first_model = 0
lane_width = 800
last_model = len(models)-1
model_factor = 0.5
lateral_factor = 1
yaw_factor = 1
speed_factor = 1
steer_factor = 1
width_factor = 1
model_index = 0
steering_torque = 0.
rate_adjustment = 1.0

while 1:
  for _cs in carState.recv_multipart():
    start_time = time.time() * 1000
    profiler.checkpoint('inputs_recv', False)

    cs = log.Event.from_bytes(_cs).carState
    vehicle_array[0].append([cs.vEgo, max(-30, min(30, steer_factor * cs.steeringAngle)), lateral_factor * cs.lateralAccel, 
                            max(-40, min(40, steer_factor * cs.steeringRate)), max(-40, min(40, cs.steeringTorqueEps)), 
                            yaw_factor * cs.yawRateCAN, cs.steeringTorque, max(-1, min(1, cs.torqueRequest))])

    profiler.checkpoint('process_inputs1')

    if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
      stock_cam_frame_prev = cs.camLeft.frame

      left_missing = 1 if cs.camLeft.parm4 == 0 else 1 if cs.camLeft.parm5 != 0 else 0
      far_left_missing = 1 if cs.camFarLeft.parm4 == 0 else 1 if cs.camFarLeft.parm5 != 0 else 0
      right_missing = 1 if cs.camRight.parm4 == 0 else 1 if cs.camRight.parm5 != 0 else 0
      far_right_missing = 1 if cs.camFarRight.parm4 == 0 else 1 if cs.camFarRight.parm5 != 0 else 0

      vehicle_array[1].append([cs.vEgo, cs.longAccel,  width_factor * max(570, lane_width + width_trim), max(-30, min(30, steer_factor * cs.steeringAngle)), lateral_factor * cs.lateralAccel, yaw_factor * cs.yawRateCAN])

      camera_array[0].append(np.clip(np.bitwise_and([left_missing,          cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm8, 
                                                     far_left_missing,      cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm8, 
                                                     right_missing,         cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm8, 
                                                     far_right_missing,     cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm8], BIT_MASK), 0, 1))

      camera_array[1].append([cs.camFarLeft.parm10,  cs.camFarLeft.parm2,  cs.camFarLeft.parm1,  cs.camFarLeft.parm3,  cs.camFarLeft.parm4,  cs.camFarLeft.parm5,  cs.camFarLeft.parm7,  cs.camFarLeft.parm9, 
                              cs.camFarRight.parm10, cs.camFarRight.parm2, cs.camFarRight.parm1, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm7, cs.camFarRight.parm9,
                              cs.camLeft.parm10,     cs.camLeft.parm2,     cs.camLeft.parm1,     cs.camLeft.parm3,     cs.camLeft.parm4,     cs.camLeft.parm5,     cs.camLeft.parm7,     cs.camLeft.parm9,    
                              cs.camRight.parm10,    cs.camRight.parm2,    cs.camRight.parm1,    cs.camRight.parm3,    cs.camRight.parm4,    cs.camRight.parm5,    cs.camRight.parm7,    cs.camRight.parm9])

      profiler.checkpoint('process_inputs2')

  l_prob =     min(1, max(0, cs.camLeft.parm4 / 127))
  r_prob =     min(1, max(0, cs.camRight.parm4 / 127))
  lr_prob =    (l_prob + r_prob) - l_prob * r_prob

  if len(vehicle_array[0]) >= round(history_rows[-1]*6.6666667+7): 

    vehicle_array[0] = vehicle_array[0][-round(history_rows[-1]*6.66666667+7):]
    vehicle_array[1] = vehicle_array[1][-15:]
    vehicle_input = [np.array([[vehicle_array[0]]], dtype='float32'), np.array([[vehicle_array[1]]], dtype='float32')]

    camera_array[0] = camera_array[0][-15:]
    camera_array[1] = camera_array[1][-15:]
    camera_input = [np.array([[camera_array[0]]], dtype='float32'), np.array([[camera_array[1]]], dtype='float32')]

    profiler.checkpoint('process_inputs1')

    vehicle_input[0][:,:,:,(cal_col[0] == 1)] -= calibration[0]
    vehicle_input[1][:,:,:,(cal_col[1] == 1)] -= calibration[1]
    camera_input[1][:,:,:,(cal_col[3] == 1)] -= calibration[3]

    profiler.checkpoint('calibrate')

    if lr_prob > 0:
      rate_adjustment = np.interp(cs.vEgo, [0., 40.], [speed_factor, 1.00])
    else:
      rate_adjustment = 1.0

    model_output = [models[0].run(None, dict({'vehicle_inputs': vehicle_input[0][0,:, -50:] * np.array([[rate_adjustment, 1.0, rate_adjustment, rate_adjustment, rate_adjustment, rate_adjustment, 1.0, 1.0]], dtype='float32'),
                                                        'vehicle_inputs2': vehicle_input[1][0,:, -history_rows[0]:] * np.array([[rate_adjustment, 1.0 , 1.0, 1.0, rate_adjustment, rate_adjustment]], dtype='float32'),
                                                        'left_flag_inputs': camera_input[0][0,:, -history_rows[0]:,:8],
                                                        'outer_left_flag_inputs': camera_input[0][0,:, -history_rows[0]:,8:16],
                                                        'right_flag_inputs': camera_input[0][0,:, -history_rows[0]:,16:24],
                                                        'outer_right_flag_inputs': camera_input[0][0,:, -history_rows[0]:,24:],
                                                        'outer_left_inputs': camera_input[1][0,:, -history_rows[0]:,:8],
                                                        'outer_right_inputs': camera_input[1][0,:, -history_rows[0]:,8:16],
                                                        'left_inputs': camera_input[1][0,:, -history_rows[0]:,16:24],
                                                        'right_inputs': camera_input[1][0,:, -history_rows[0]:,24:],
                                                        'fingerprints': [fingerprint], 
                                                        'center_profiles': [center_profiles * np.array([1., 1., cs.vEgo, cs.vEgo, 1.], 'float32')],
                                                        'center_bias': [center_bias[0,:,:]],
                                                        'model_bias': [model_bias[0,:,:]],
                                                    }))]

    profiler.checkpoint('predict') 
    calc_angles[0] = np.transpose(model_output[0][0][0,:,:PATH_COUNT])
    model_index = np.argmin(np.sum(np.absolute(model_output[0][0][0,-5:,PATH_COUNT:]), axis=-2))
    if lr_prob > 0 and steer_override_timer <= 25 and cs.vEgo >= 10:  
      path_profile_counter[model_index] += 1
    calc_angles[0][:,:] = calc_angles[0][model_index:model_index+1]
    calc_center[0] = model_output[0][0][0,:,PATH_COUNT:PATH_COUNT+1]
    projected_steering = cs.steeringAngle + (cs.steeringRate * projected_rate) + ((cs.steeringRate - previous_rate) * projected_rate / 6.66667)
    previous_rate = cs.steeringRate
    c_poly = model_output[0][PATH_COUNT+1][0]
    d_poly = model_output[0][PATH_COUNT+1+model_index][0]

    something_masked = False

    angle_plan = send_path_to_controls(model_index, calc_angles, calc_center, angle_plan, projected_steering, path_send, gernPath, cs, angle_bias, lane_width, width_trim, l_prob, r_prob, lr_prob, calibrated, c_poly, d_poly, steer_override_timer, something_masked)
    profiler.checkpoint('send')
    time.sleep(0.02)

    if frame % 151 == 0:
      print("best_angles: ", model_index)
      print(path_profile_counter // 1)
      print(path_profile_scale.astype('float'))
      print((100 * np.transpose(angle_plan[0,:]))//10)
      print(np.array(model_output[0][0][0,:,PATH_COUNT+model_index], dtype='int32'))

    max_width_step = 0.005 * cs.vEgo * l_prob * r_prob
    if cs.camLeft.parm2 > 0 and cs.camRight.parm2 < 0 and not something_masked:
      lane_width = max(570, lane_width - max_width_step * 2, min(2100, lane_width + max_width_step, cs.camLeft.parm2 - cs.camRight.parm2))

    steer_override_timer -= 1
    if steer_override_timer < 0 and abs(cs.steeringRate) < 3 and abs(cs.steeringAngle - calibration[0][0]) < 3 and l_prob > 0 and r_prob > 0 and cs.vEgo > 10 and cs.camLeft.parm2 > 0 and cs.camRight.parm2 < 0 and (abs(cs.steeringTorque) < 300 or ((cs.steeringTorque < 0) == (cs.camLeft.parm2 + cs.camRight.parm2 < 0))): # and not something_masked:
      if abs(cs.torqueRequest) > 0:
        if cs.camLeft.parm2 + cs.camRight.parm2 > 0:
          angle_bias += (0.00001 * cs.vEgo * lr_prob)
        else:
          angle_bias -= (0.00001 * cs.vEgo * lr_prob)

      model_bias[0][:,0] += (0.00001 * cs.vEgo * lr_prob * model_output[0][1][0])
      model_bias[0][-1,:] = 0.0

      center_bias[0][0,0] += (0.00002 * cs.vEgo * lr_prob * model_output[0][PATH_COUNT+1][0,0])
      center_bias[0][1,0] += (0.00002 * cs.vEgo * lr_prob * model_output[0][PATH_COUNT+1][0,1])
      center_bias[0][2,0] += (0.00002 * cs.vEgo * lr_prob * model_output[0][PATH_COUNT+1][0,2])
      center_bias[0][3,0] += (0.00002 * cs.vEgo * lr_prob * model_output[0][PATH_COUNT+1][0,3])
      center_bias[0][4,0] += (0.00002 * cs.vEgo * lr_prob * (model_output[0][PATH_COUNT+1][0,4] - ((cs.camLeft.parm2 + cs.camRight.parm2) / 993)))

      #>>>  NOTE TO SELF:  Self, do not enable the next line!  You know you want to, but DON'T!    <<<#
      #center_bias[0][-1,:] = 0.0

      profiler.checkpoint('bias')

    elif abs(cs.steeringTorque) > 600 and (cs.steeringTorque < 0) != (cs.camLeft.parm2 + cs.camRight.parm2 < 0) and abs(cs.torqueRequest) > 0:
      # Prevent angle_bias adjustment for 2 seconds after driver opposes the model
      steer_override_timer = 30
    #except:
    #  pass

    frame += 1
    distance_driven += cs.vEgo

    if distance_driven > 10000 and cs.vEgo > 10 and lr_prob > 0 and steer_override_timer <= 0:
      path_profile_target = path_profile_counter[0] * path_profile_ratio
      for i in range(1,5):
        path_profile_scale[i-1] = max(0.1, path_profile_scale[i-1] + cs.vEgo * lr_prob * (0.0001 if path_profile_counter[i] < path_profile_target else -0.0001))
        center_profiles[i,2:] = CENTER_PROFILES[i,2:] * path_profile_scale[i-1]
    
    if cs.vEgo > 10 and abs(cs.steeringAngle - calibration[0][0]) <= 3 and abs(cs.steeringRate) < 3 and l_prob > 0 and r_prob > 0 and not something_masked:
      cal_factor = update_calibration(calibration, [vehicle_input, camera_input], cal_col, cs)
      profiler.checkpoint('calibrate')

    if frame % 60 == 0:
      print('lane_width: %0.1f angle bias: %0.2f  distance_driven:  %0.2f   center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  l_offset:  %0.2f  r_offset:  %0.2f  model time:  %0.4fs  adj_speed:  %0.1f' % (lane_width, angle_bias, distance_driven, calc_center[0][-1,0], l_prob, r_prob, cs.camLeft.parm2, cs.camRight.parm2, 0.001 * execution_time_avg, max(10, rate_adjustment * cs.vEgo)))

    if ((cs.vEgo < 10 and not cs.cruiseState.enabled) or not calibrated) and distance_driven > next_params_distance:
      next_params_distance = distance_driven + 13300
      print(np.round(calibration[0],2))
      put_nonblocking("CalibrationParams", json.dumps({'models': models_def['models'], 
                                                        'calibration': list(np.concatenate(([float(x) for x in calibration[0]],[float(x) for x in calibration[1]],[float(x) for x in calibration[2]],[float(x) for x in calibration[3]]), axis=0)),
                                                        'lane_width': float(lane_width),
                                                        'angle_bias': float(angle_bias), 
                                                        'center_bias': list(np.reshape(np.array(center_bias, dtype='float'), (2 * 1 * CENTER_POLYS,))), 
                                                        'model_bias': list(np.reshape(np.array(model_bias, dtype='float'), (2 * 1 * ANGLE_POLYS,))),
                                                        'path_profile_counter': list(path_profile_counter.astype('float')),
                                                        'path_profile_scale': list(path_profile_scale.astype('float'))}, indent=2))
      calibrated = True
      profiler.checkpoint('save_cal')

    # TODO: replace kegman_conf with params!
    if frame % 100 == 0:
      (mode, ino, dev, nlink, uid, gid, size, atime, mtime, kegtime) = os.stat(os.path.expanduser('~/kegman.json'))
      if kegtime != kegtime_prev:
        kegtime_prev = kegtime
        kegman = kegman_conf()
        steer_factor = float(kegman.conf['steerFactor'])
        first_model = max(0, min(len(models)-1, int(float(kegman.conf['firstModel']))))
        last_model = max(first_model, min(len(models)-1, int(float(kegman.conf['lastModel']))))
        model_factor = abs(float(kegman.conf['modelFactor']))
        speed_factor = abs(float(kegman.conf['speedFactor']))
        width_factor = abs(float(kegman.conf['widthFactor']))
        accel_limit = accel_profile * max(0, abs(float(kegman.conf['polyAccelLimit']))) * 10
        lateral_factor = abs(float(kegman.conf['lateralFactor']))
        yaw_factor = abs(float(kegman.conf['yawFactor']))
        path_profile_ratio = min(2., max(0, float(kegman.conf['polyAdjust'])))
        path_profile_counter[1:] = path_profile_counter[0] * path_profile_ratio

      profiler.checkpoint('kegman')

    execution_time_avg += (max(0.0001, time_factor) * ((time.time()*1000 - start_time) - execution_time_avg))
    time_factor *= 0.96

    if frame % 100 == 0 and profiler.enabled:
      profiler.display()
      profiler.reset(True)
    profiler.checkpoint('profiling')
