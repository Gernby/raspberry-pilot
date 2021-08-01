#!/usr/bin/env python
import os
from numpy.core.defchararray import center
import zmq
import time
import json
import gc
import numpy as np

INPUTS = 79
OUTPUTS = 9

from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from selfdrive.car.honda.values import CAR
from enum import Enum
from cereal import log, car
from setproctitle import setproctitle
from common.params import Params, put_nonblocking
from common.profiler import Profiler
import onnxruntime as ort

setproctitle('transcoderd')

options = ort.SessionOptions()
#options.intra_op_num_threads = 1
#options.inter_op_num_threads = 1
#options.execution_mode = ort.ExecutionMode.ORT_PARALLEL
options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
provider = 'CPUExecutionProvider'

params = Params()
profiler = Profiler(False, 'transcoder')

BIT_MASK = [0, 0, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8] 

history_rows = []
OUTPUT_ROWS = 15

fingerprint = np.zeros((1, 4), dtype='float32')
left_center = np.zeros((OUTPUT_ROWS))
right_center = np.zeros((OUTPUT_ROWS))
calc_center = [np.zeros((3,OUTPUT_ROWS)),np.zeros((3,OUTPUT_ROWS))]
calc_center_prev = np.zeros((OUTPUT_ROWS))
desired_center = np.zeros((OUTPUT_ROWS,1), dtype='float32')
center_correction = np.ones((OUTPUT_ROWS,1),dtype='float32')
center_correction[5:] = 0.975
center_correction[6:] = 0.9
center_correction[:,0] = np.polyval(np.polyfit(np.arange(15), center_correction[:,0], 6), np.arange(15))
center_correction[:3,0] = center_correction[3,0]
center_correction[-4:,0] = center_correction[-5,0]
super_center = np.zeros((OUTPUT_ROWS))
smooth_center = np.zeros((OUTPUT_ROWS))
fast_angles = np.zeros((OUTPUT_ROWS,1))
accel_limit = np.arange(15, dtype='float32') / 7.5
projected_rate = np.arange(0., 0.10066667,0.0066666)[1:]

if os.path.exists('models/models.json'):
  with open('models/models.json', 'r') as f:
    models = []
    models_def = json.load(f)
    for md in models_def['models']:
      models.append(ort.InferenceSession(os.path.expanduser('models/%s' % md), options))
      models[-1].set_providers([provider], None)
      start_time = time.time()
      history_rows.append(2 if not '-5' in md else 5)
      for i in range(1):
        model_output = models[-1].run(None, {'prod_vehicle1_0:0': np.zeros((1, 50, 13), dtype='float32')[:,-round(min(26, history_rows[len(models)-1]*6.6666667)):,:7], 
                                            'prod_vehicle2_0:0': np.zeros((1, 5, INPUTS-7), dtype='float32')[:,-history_rows[len(models)-1]:,:6],
                                            'prod_camera1_0:0': np.zeros((1, 5, INPUTS-7), dtype='float32')[:,-history_rows[len(models)-1]:,6:-32],
                                            'prod_camera2_0:0': np.zeros((1, 5, INPUTS-7), dtype='float32')[:,-history_rows[len(models)-1]:,-32:],
                                            'fingerprints0:0': [[1,0,0,0]],
                                            'prod_center_0:0': [desired_center],
                                            'l_prob:0': [[0.0]],
                                            'r_prob:0': [[0.0]],
                                            'desired_center:0': [center_correction],
                                            'center_bias:0': [np.ones((OUTPUT_ROWS,1),dtype='float32')]
                                          })[0]
      print(model_output)
      print(np.round(model_output[0,:,12::3]))
      print(time.time()-start_time, md)

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

def send_path_to_controls(model_index, fast_angles, calc_center, angle_plan, path_send, gernPath, cs, projected_rate, angle_bias, lane_width, width_trim, l_prob, r_prob, lr_prob, calibrated):
  angle_plan = np.clip(fast_angles[model_index], angle_plan - accel_limit, angle_plan + accel_limit)

  future_steering = cs.steeringAngle + cs.steeringRate * projected_rate
  angle_plan = np.clip(angle_plan, future_steering - accel_limit, future_steering + accel_limit)

  path_send = log.Event.new_message()
  path_send.init('pathPlan')

  path_send.pathPlan.centerCompensation = 0
  path_send.pathPlan.angleSteers = float(angle_plan[0][5])
  path_send.pathPlan.fastAngles = [[float(x) + angle_bias for x in y] for y in angle_plan]
  path_send.pathPlan.laneWidth = float(lane_width + width_trim)
  path_send.pathPlan.angleOffset = float(calibration[0][0])
  path_send.pathPlan.angleBias = angle_bias
  path_send.pathPlan.modelIndex = model_index
  path_send.pathPlan.paramsValid = calibrated
  path_send.pathPlan.cPoly = [float(x) for x in (calc_center[model_index][:,0])]
  path_send.pathPlan.lPoly = [float(x) for x in (calc_center[model_index][:,1] + 0.5 * lane_width)]
  path_send.pathPlan.rPoly = [float(x) for x in (calc_center[model_index][:,2] - 0.5 * lane_width)]
  path_send.pathPlan.lProb = float(l_prob)
  path_send.pathPlan.rProb = float(r_prob)
  path_send.pathPlan.cProb = float(lr_prob)
  path_send.pathPlan.canTime = cs.canTime
  path_send.pathPlan.sysTime = cs.sysTime
  gernPath.send(path_send.to_bytes())

  desired_center = calc_center[model_index][:,0:1]
  desired_center[:-1] = desired_center[1:]

  return angle_plan, desired_center


gernPath = pub_sock(service_list['pathPlan'].port)
carState = sub_sock(service_list['carState'].port, conflate=False)

width_trim = 0
angle_bias = 0
execution_time_avg = 0.027
time_factor = 1.0
next_params_distance = 133000.0
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
lateral_adjust = 0
frame = 0
dump_sock(carState, True)

calibration_items = [['angle_steers','lateral_accelleration','yaw_rate_can'],['angle_steers2','lateral_accelleration2','yaw_rate_can2'],[],['far_left_1','far_left_7','far_left_9','far_right_1','far_right_7','far_right_9','left_1','left_7','left_9','right_1','right_7','right_9']]
all_items = [['v_ego','angle_steers','lateral_accelleration','angle_rate', 'angle_rate_eps', 'yaw_rate_can','steering_torque'],['v_ego','long_accel', 'lane_width','angle_steers2','lateral_accelleration2','yaw_rate_can2'],
           ['l_blinker','r_blinker',
            'left_missing','l6b_6','l6b_6','l6b_6','l6b_6','l6b_6','l6b_6','l8b_8',
            'far_left_missing','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl6b_6','fl8b_8',
            'right_missing','r6b_6','r6b_6','r6b_6','r6b_6','r6b_6','r6b_6','r8b_8',
            'far_right_missing','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr6b_6','fr8b_8'],
           ['far_left_10', 'far_left_2',  'far_left_1',  'far_left_3',  'far_left_4',  'far_left_5',  'far_left_7',  'far_left_9',  
            'far_right_10','far_right_2', 'far_right_1', 'far_right_3', 'far_right_4', 'far_right_5', 'far_right_7', 'far_right_9', 
            'left_10',     'left_2',      'left_1',      'left_3',      'left_4',      'left_5',      'left_7',      'left_9',      
            'right_10',    'right_2',     'right_1',     'right_3',     'right_4',     'right_5',     'right_7',     'right_9']]
cal_col = [np.zeros((len(all_items[0])),dtype=np.int),np.zeros((len(all_items[1])),dtype=np.int),[], np.zeros((len(all_items[3])),dtype=np.int)]
cal_factor = [np.zeros((len(all_items[0])),dtype='float32'),np.zeros((len(all_items[1])),dtype='float32'),[],np.zeros((len(all_items[3])),dtype='float32')]
for i in range(len(all_items)):
  for col in range(len(all_items[i])):
    print(i,col)
    if len(cal_col[i]) > col:
      cal_col[i][col] = 1 if len(all_items[i]) > col and all_items[i][col] in calibration_items[i] else 0 

adj_items =  ['far_left_2','far_right_2','left_2','right_2']
adj_col = np.zeros((len(adj_items)),dtype=np.int)
for col in range(len(adj_items)):
  adj_col[col] = all_items[3].index(adj_items[col])

kegtime_prev = 0
angle_speed_count = model_output.shape[2] - 7
model_bias = [np.zeros((OUTPUT_ROWS), 'float32'), np.zeros((OUTPUT_ROWS), 'float32')]
center_bias = [np.zeros((OUTPUT_ROWS,1), 'float32'), np.zeros((OUTPUT_ROWS,1), 'float32')]
  
calibrated = True
calibration_data = params.get("CalibrationParams")
if not calibration_data is None:
  calibration_data =  json.loads(calibration_data)
  calibration = np.array(calibration_data['calibration'], dtype='float32')
  if not models_def["reset"] or ("models" in calibration_data and calibration_data['models'] == models_def['models']):
    if 'center_bias' in calibration_data:
      center_bias = [np.expand_dims(np.polyval(np.polyfit(np.arange(15), calibration_data['center_bias'][:OUTPUT_ROWS], 4), np.arange(15)), axis=1), 
                     np.expand_dims(np.polyval(np.polyfit(np.arange(15), calibration_data['center_bias'][-OUTPUT_ROWS:], 4), np.arange(15)), axis=1)]
    if 'model_bias' in calibration_data:
      model_bias = [np.polyval(np.polyfit(np.arange(15), calibration_data['model_bias'][:OUTPUT_ROWS], 4), np.arange(15)), 
                    np.polyval(np.polyfit(np.arange(15), calibration_data['model_bias'][-OUTPUT_ROWS:], 4), np.arange(15))]
    angle_bias = calibration_data['angle_bias']
  else:
    os.system("cp /data/params/d/CalibrationParams /data/params/d/CalibrationParams%d" % int(time.time()))
    print("New models!  Resetting bias")

if calibration_data is None or len(calibration) != (len(calibration_items[0]) + len(calibration_items[1]) + len(calibration_items[3])):
  calibration = [np.zeros(len(calibration_items[0]), dtype='float32'), np.zeros(len(calibration_items[1]), dtype='float32'), [], np.zeros(len(calibration_items[3]), dtype='float32')]
  calibrated = False
  print("resetting calibration")
  params.delete("CalibrationParams")
else:
  calibration = [np.array(calibration[:len(calibration_items[0])], dtype='float32'), 
                np.array(calibration[len(calibration_items[0]):len(calibration_items[0])+len(calibration_items[1])], dtype='float32'), [], 
                np.array(calibration[len(calibration_items[0])+len(calibration_items[1]):], dtype='float32')]
  lane_width = calibration_data['lane_width']

print(calibration)

stock_cam_frame_prev = -1
vehicle_array = [[],[]]
camera_array = [[],[]]
first_model = 0
last_model = len(models)-1
model_factor = 0.5
lateral_factor = 1
yaw_factor = 1
speed_factor = 1
steer_factor = 1
width_factor = 1
angle_plan = 0
model_index = 0
steering_torque = 0.
fast_angles = [np.array([[0],[0]], dtype='float32'),np.array([[0],[0]], dtype='float32')]
rate_adjustment = 1.0

while 1:
  for _cs in carState.recv_multipart():
    start_time = time.time() * 1000
    profiler.checkpoint('inputs_recv', False)

    cs = log.Event.from_bytes(_cs).carState
    vehicle_array[0].append([cs.vEgo, max(-30, min(30, steer_factor * cs.steeringAngle)), lateral_factor * cs.lateralAccel, 
                            max(-40, min(40, steer_factor * cs.steeringRate)), max(-40, min(40, cs.steeringTorqueEps)), 
                            yaw_factor * cs.yawRateCAN, cs.steeringTorque])

    profiler.checkpoint('process_inputs1')

    if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
      stock_cam_frame_prev = cs.camLeft.frame

      left_missing = 1 if cs.camLeft.parm4 == 0 else 0
      far_left_missing = 1 if cs.camFarLeft.parm4 == 0 else 0
      right_missing = 1 if cs.camRight.parm4 == 0 else 0
      far_right_missing = 1 if cs.camFarRight.parm4 == 0 else 0
      
      vehicle_array[1].append([cs.vEgo, cs.longAccel,  width_factor * max(570, lane_width + width_trim), max(-30, min(30, steer_factor * cs.steeringAngle)), lateral_factor * cs.lateralAccel, yaw_factor * cs.yawRateCAN])

      camera_array[0].append(np.clip(np.bitwise_and([0, 0, left_missing,          cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm8, 
                                                           far_left_missing,      cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm8, 
                                                           right_missing,         cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm8, 
                                                           far_right_missing,     cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm8], BIT_MASK), -1, 1))

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
    vehicle_array[1] = vehicle_array[1][-6:]
    vehicle_input = [np.array([[vehicle_array[0]]], dtype='float32'), np.array([[vehicle_array[1]]], dtype='float32')]
    vehicle_input[0][:,:,6:,2:7] = vehicle_input[0][:,:,:-6,2:7]
    vehicle_input[1][:,:,1:,4:6] = vehicle_input[1][:,:,:-1,4:6]

    camera_array[0] = camera_array[0][-6:]
    camera_array[1] = camera_array[1][-6:]
    camera_input = [np.array([[camera_array[0]]], dtype='float32'), np.array([[camera_array[1]]], dtype='float32')]

    profiler.checkpoint('process_inputs1')

    vehicle_input[0][:,:,:,(cal_col[0] == 1)] -= calibration[0]
    vehicle_input[1][:,:,:,(cal_col[1] == 1)] -= calibration[1]
    camera_input[1][:,:,:,(cal_col[3] == 1)] -= calibration[3]

    profiler.checkpoint('calibrate')
    
    if model_index == 0 and lr_prob > 0:
      rate_adjustment = np.interp(cs.vEgo, [0., 40.], [speed_factor, 1.00])
    else:
      rate_adjustment = 1.0
    
    model_output = np.array(models[model_index].run(None, dict({'prod_vehicle1_0:0': vehicle_input[0][0,:, -round(min(26,history_rows[model_index]*6.6666667)):] * np.array([[rate_adjustment, 1.0, rate_adjustment, rate_adjustment, rate_adjustment, rate_adjustment, 1.0]], dtype='float32'),
                                                                'prod_vehicle2_0:0': vehicle_input[1][0,:, -history_rows[model_index]:] * np.array([[rate_adjustment, 1.0 , 1.0, 1.0, rate_adjustment, rate_adjustment]], dtype='float32'),
                                                                'prod_camera1_0:0': camera_input[0][0,:, -history_rows[model_index]:],
                                                                'prod_camera2_0:0': camera_input[1][0,:, -history_rows[model_index]:],
                                                                'fingerprints0:0': fingerprint, 
                                                                'prod_center_0:0': [desired_center],
                                                                'l_prob:0': [[l_prob]],
                                                                'r_prob:0': [[r_prob]],
                                                                'desired_center:0': [center_correction],
                                                                'center_bias:0': [center_bias[model_index]]
                                                                }))[0])
    profiler.checkpoint('predict')

    fast_angles[model_index] = np.transpose(model_output[0,:,:angle_speed_count] + calibration[0][0]) - model_bias[model_index]
    calc_center[model_index] = model_output[0,:,angle_speed_count::3]

    if lr_prob == 0 or len(models) == 1 or model_index == 1 or int(abs(fast_angles[0][10,8]) * model_factor) == 0:
      angle_plan, desired_center = send_path_to_controls(model_index, fast_angles, calc_center, angle_plan, path_send, gernPath, cs, projected_rate, angle_bias, lane_width, width_trim, l_prob, r_prob, lr_prob, calibrated)
      profiler.checkpoint('send')
      time.sleep(0.03)

    if len(models) > 1:
      if (model_index == 0 and lr_prob > 0):

        model_output = np.array(models[1].run(None, dict({'prod_vehicle1_0:0': vehicle_input[0][0,:, -round(min(26,history_rows[1]*6.6666667)):], 
                                                          'prod_vehicle2_0:0': vehicle_input[1][0,:, -history_rows[1]:],
                                                          'prod_camera1_0:0': camera_input[0][0,:, -history_rows[1]:],
                                                          'prod_camera2_0:0': camera_input[1][0,:, -history_rows[1]:],
                                                          'fingerprints0:0': fingerprint, 
                                                          'prod_center_0:0': [desired_center],
                                                          'l_prob:0': [[l_prob]],
                                                          'r_prob:0': [[r_prob]],
                                                          'desired_center:0': [center_correction],
                                                          'center_bias:0': [center_bias[1]]
                                                        }))[0])
        profiler.checkpoint('predict')

        fast_angles[1] = np.transpose(model_output[0,:,:angle_speed_count] + calibration[0][0]) - model_bias[1]
        calc_center[1] = model_output[0,:,angle_speed_count::3]

      if (lr_prob > 0 or model_index == 1) and fast_angles[0].shape == fast_angles[1].shape:
        if (int(abs(fast_angles[1][10,8]) * model_factor) > 0) and model_index == 0 and lr_prob > 0 and calibrated:
          model_index = 1
          angle_plan, desired_center = send_path_to_controls(model_index, fast_angles, calc_center, angle_plan, path_send, gernPath, cs, projected_rate, angle_bias, lane_width, width_trim, l_prob, r_prob, lr_prob, calibrated)
        else:
          model_index = 0
      elif fast_angles[0].shape != fast_angles[1].shape:
        fast_angles[1] = fast_angles[0]

    max_width_step = 0.05 * cs.vEgo * l_prob * r_prob
    lane_width = max(570, lane_width - max_width_step * 2, min(1700, lane_width + max_width_step, cs.camLeft.parm2 - cs.camRight.parm2))

    steer_override_timer -= 1
    if model_index == 0 and steer_override_timer < 0 and abs(cs.steeringRate) < 3 and abs(cs.steeringAngle - calibration[0][0]) < 3 and cs.torqueRequest != 0 and l_prob > 0 and r_prob > 0 and cs.vEgo > 10 and (abs(cs.steeringTorque) < 300 or ((cs.steeringTorque < 0) == (calc_center[0][3,0] < 0))):
      if calc_center[0][4,0] > 0:
        angle_bias += (0.00002 * cs.vEgo)
      elif calc_center[0][4,0] < 0:
        angle_bias -= (0.00002 * cs.vEgo)

      for i in range(len(models)):
        model_bias[i] += (0.00002 * cs.vEgo * lr_prob * (fast_angles[i][10,:] - fast_angles[i][10,4]))
        calc_center[i][:,0] = (l_prob * calc_center[i][:,1] + r_prob * calc_center[i][:,2]) / (l_prob + r_prob)
        center_bias[i][:,0] += (0.00002 * cs.vEgo * lr_prob * (calc_center[i][:,0] - calc_center[i][4,0]))

      if calc_center[0][4,1] > calc_center[0][4,2]:	
        width_trim += 0.5	
      else:	
        width_trim -= 1	
      width_trim = min(100, max(-200, min(width_trim, 0)))

      profiler.checkpoint('bias')
      
    elif model_index == 0 and abs(cs.steeringTorque) > 300 and (cs.steeringTorque < 0) != calc_center[0][4,0] < 0:
      # Prevent angle_bias adjustment for 3 seconds after driver opposes the model
      steer_override_timer = 45

    frame += 1
    distance_driven += cs.vEgo 

    if cs.vEgo > 10 and abs(cs.steeringAngle - calibration[0][0]) <= 3 and abs(cs.steeringRate) < 3 and l_prob > 0 and r_prob > 0:
      cal_factor = update_calibration(calibration, [vehicle_input, camera_input], cal_col, cs)
      profiler.checkpoint('calibrate')

    if frame % 60 == 0:
      print('lane_width: %0.1f angle bias: %0.2f  distance_driven:  %0.2f   center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  l_offset:  %0.2f  r_offset:  %0.2f  model time:  %0.4fs  adj_speed:  %0.1f' % (lane_width, angle_bias, distance_driven, calc_center[0][-1,0], l_prob, r_prob, cs.camLeft.parm2, cs.camRight.parm2, 0.001 * execution_time_avg, max(10, rate_adjustment * cs.vEgo)))

    if ((cs.vEgo < 10 and not cs.cruiseState.enabled) or not calibrated) and distance_driven > next_params_distance:
      next_params_distance = distance_driven + 133000
      if calibrated:
        print(np.round(calibration[0],2))
        put_nonblocking("CalibrationParams", json.dumps({'models': models_def['models'], 'calibration': list(np.concatenate(([float(x) for x in calibration[0]],[float(x) for x in calibration[1]],[float(x) for x in calibration[2]],[float(x) for x in calibration[3]]), axis=0)),'lane_width': float(lane_width),'angle_bias': float(angle_bias), 'center_bias': list([float(x[0]) for x in np.concatenate((center_bias))]), 'model_bias': list([float(x) for x in np.concatenate((model_bias))])}, indent=2))
      else:
        print(list(np.concatenate(([float(x) for x in calibration[0]],[float(x) for x in calibration[1]]), axis=0)))
        params.put("CalibrationParams", json.dumps({'models': models_def['models'], 'calibration': list(np.concatenate(([float(x) for x in calibration[0]],[float(x) for x in calibration[1]],[float(x) for x in calibration[2]],[float(x) for x in calibration[3]]), axis=0)),'lane_width': float(lane_width),'angle_bias': float(angle_bias), 'center_bias': list([float(x[0]) for x in np.concatenate((center_bias))]), 'model_bias': list([float(x) for x in np.concatenate((model_bias))])}, indent=2))
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
        accel_limit = max(0, abs(float(kegman.conf['accelLimit'])) * 6.7) * np.arange(15, dtype='float32')
        lateral_factor = abs(float(kegman.conf['lateralFactor']))
        yaw_factor = abs(float(kegman.conf['yawFactor']))
    
      profiler.checkpoint('kegman')

    execution_time_avg += (max(0.0001, time_factor) * ((time.time()*1000 - start_time) - execution_time_avg))
    time_factor *= 0.96

    if frame % 100 == 0 and profiler.enabled:
      profiler.display()
      profiler.reset(True)
    profiler.checkpoint('profiling')
