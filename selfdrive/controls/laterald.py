#!/usr/bin/env python
import os
import zmq
import time
import json
import platform
import subprocess
import multiprocessing
import numpy as np
import joblib
import requests

from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from enum import Enum
from cereal import log
from cffi import FFI
from setproctitle import setproctitle

from common.params import Params
params = Params()
user_id = str(params.get("DongleId"))

url_string = 'http://127.0.0.1:8086/write?db=carDB&u=liveOP&p=liveOP&precision=ms'

ffi = FFI()
ffi.cdef("long syscall(long number, ...);")
libc = ffi.dlopen(None)

def set_realtime_priority(level):
  if platform.machine() == "x86_64":
    NR_gettid = 186
  elif platform.machine() == "aarch64":
    NR_gettid = 178
  else:
    raise NotImplementedError
  tid = libc.syscall(NR_gettid)
  print("/n/n realtime priority = %d  %s  %s/n" %(level, NR_gettid, str(tid)))
  return subprocess.call(['chrt', '-f', '-p', str(level), str(tid)])

def dump_sock(sock):
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

pathFormatString1 = 'pathPlan,user=' + user_id + ' l0=%0.3f,l1=%0.3f,l2=%0.3f,l3=%0.3f,l4=%0.3f,l5=%0.3f,l6=%0.3f,l7=%0.3f,l8=%0.3f,l9=%0.3f,l10=%0.3f,l11=%0.3f,l12=%0.3f,l13=%0.3f,l14=%0.3f,'
pathFormatString2 = "r0=%0.3f,r1=%0.3f,r2=%0.3f,r3=%0.3f,r4=%0.3f,r5=%0.3f,r6=%0.3f,r7=%0.3f,r8=%0.3f,r9=%0.3f,r10=%0.3f,r11=%0.3f,r12=%0.3f,r13=%0.3f,r14=%0.3f,"
pathFormatString3 = "c0=%0.3f,c1=%0.3f,c2=%0.3f,c3=%0.3f,c4=%0.3f,c5=%0.3f,c6=%0.3f,c7=%0.3f,c8=%0.3f,c9=%0.3f,c10=%0.3f,c11=%0.3f,c12=%0.3f,c13=%0.3f,c14=%0.3f,"
pathFormatString4 = "a3=%0.3f,a4=%0.3f,a5=%0.3f,a6=%0.3f,a10=%0.3f,lprob=%0.3f,rprob=%0.3f,cprob=%0.3f,lane_width=%0.3f,angle=%0.3f,rate=%0.3f,plan_age=%0.3f %d\n"
carStateFormatString2 = "carState,user=" + user_id + " angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d,v_ego=%0.4f,wheel_speed_fl=%0.4f,wheel_speed_fr=%0.4f,wheel_speed_rl=%0.4f,wheel_speed_rr=%0.4f,l_blinker=%d,r_blinker=%d,lk_mode=%d,enabled=%d,left_frame=%d,left_1=%d,left_2=%d,left_3=%d,left_4=%d,left_5=%d,left_6=%d,left_7=%d,left_8=%d,left_9=%d,left_10=%d,left_solid=%d,left_dashed=%d,right_frame=%d,right_1=%d,right_2=%d,right_3=%d,right_4=%d,right_5=%d,right_6=%d,right_7=%d,right_8=%d,right_9=%d,right_10=%d,right_solid=%d,right_dashed=%d,far_left_frame=%d,far_left_1=%d,far_left_2=%d,far_left_3=%d,far_left_4=%d,far_left_5=%d,far_left_6=%d,far_left_7=%d,far_left_8=%d,far_left_9=%d,far_left_10=%d,far_left_solid=%d,far_left_dashed=%d,far_right_frame=%d,far_right_1=%d,far_right_2=%d,far_right_3=%d,far_right_4=%d,far_right_5=%d,far_right_6=%d,far_right_7=%d,far_right_8=%d,far_right_9=%d,far_right_10=%d,far_right_solid=%d,far_right_dashed=%d %d\n"
carStateFormatString1 = "carState,user=" + user_id + " angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d %d\n"
pathDataString = ""
kegmanDataString = ""
carStateDataString1 = ""
carStateDataString2 = ""
insertString = ""
canInsertString = ""

Inputs = 51
Outputs = 5

scaler_type = 'MinMax_tanh'
history_rows = 5

setproctitle('laterald')
set_realtime_priority(1)
poller = zmq.Poller()

carState = sub_sock(service_list['carState'].port, conflate=False, poller=poller)
gernPath = pub_sock(service_list['pathPlan'].port)
gernModelInputs = pub_sock(service_list['model'].port)
gernModelOutputs = sub_sock(8605, poller=poller)

recv_frames = 1
sent_frames = 1
frame_count = 1
dashboard_count = 0

try:
  input_scaler = joblib.load(os.path.expanduser('./models/GRU_%s_%d_inputs_A.scaler' % (scaler_type, Inputs)))
  output_scaler = joblib.load(os.path.expanduser('./models/GRU_%s_%d_outputs_A.scaler' % (scaler_type, Outputs)))
except:
  input_scaler = joblib.load(os.path.expanduser('./models/GRU_%s_%d_inputs_C.scaler' % (scaler_type, Inputs)))
  output_scaler = joblib.load(os.path.expanduser('./models/GRU_%s_%d_outputs_C.scaler' % (scaler_type, Outputs)))

scaler_padding = None 
scaled_camera_array = []
scaled_vehicle_array = []
stock_cam_frame_prev = -1
lane_width = 0
half_width = 0
l_probs = {}
r_probs = {}
l_offset = {}
r_offset = {}
l_prob_smooth = 0.
r_prob_smooth = 0.

path_send = log.Event.new_message()
path_send.init('pathPlan')
advanceSteer = 1
back_log = 0
dump_sock(carState)
              
r = requests.post('http://localhost:8086/query?q=CREATE DATABASE carDB')

row_count = 0
column_count = 0
while 1:
  for socket, event in poller.poll():

    if socket is carState:
      back_log += 1
      _cs = log.Event.from_bytes(socket.recv())
      cs = _cs.carState
      frame_count += 1

      unscaled_input_array = [[cs.vEgo, cs.steeringAngle, cs.lateralAccel, cs.steeringTorqueEps, cs.yawRateCAN, cs.longAccel,              0 ,    0           , cs.steeringRate, cs.steeringTorque, cs.torqueRequest,
                      cs.camLeft.parm1, cs.camLeft.parm2, cs.camLeft.parm3, cs.camLeft.parm4, cs.camLeft.parm5, cs.camLeft.parm6, cs.camLeft.parm7, cs.camLeft.parm8, cs.camLeft.parm9, cs.camLeft.parm10, 
                      cs.camFarLeft.parm1, cs.camFarLeft.parm2, cs.camFarLeft.parm3, cs.camFarLeft.parm4, cs.camFarLeft.parm5, cs.camFarLeft.parm6, cs.camFarLeft.parm7, cs.camFarLeft.parm8, cs.camFarLeft.parm9, cs.camFarLeft.parm10, 
                      cs.camRight.parm1, cs.camRight.parm2, cs.camRight.parm3, cs.camRight.parm4, cs.camRight.parm5, cs.camRight.parm6, cs.camRight.parm7, cs.camRight.parm8, cs.camRight.parm9, cs.camRight.parm10, 
                      cs.camFarRight.parm1, cs.camFarRight.parm2, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm6, cs.camFarRight.parm7, cs.camFarRight.parm8, cs.camFarRight.parm9, cs.camFarRight.parm10]] 

      scaled_data = input_scaler.transform(unscaled_input_array)
      scaled_vehicle_array.append(scaled_data[:,:11])
      if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
        back_log = 0
        scaled_camera_array.append(scaled_data[:,11:])
        if len(scaled_camera_array) > history_rows:
          scaled_array = np.concatenate((scaled_vehicle_array[-history_rows:], scaled_camera_array[-history_rows:]), axis = 2)
          scaled_camera_array.pop(0)
          scaled_vehicle_array.pop(0)

          stock_cam_frame_prev = cs.camLeft.frame

          if recv_frames > 0: sent_frames += 1

          l_prob = cs.camLeft.parm4/127
          r_prob = cs.camRight.parm4/127

          if cs.camLeft.solid and cs.camRight.dashed:
            l_prob *= -1
          elif cs.camRight.solid and cs.camLeft.dashed:
            r_prob *= -1

          l_probs[cs.canTime] = l_prob
          r_probs[cs.canTime] = r_prob
          l_offset[cs.canTime] = cs.camLeft.parm2
          r_offset[cs.canTime] = cs.camRight.parm2
          
          input_array = list(np.asarray(scaled_array).reshape(history_rows * len(scaled_array[0][0])).astype('float'))
          input_array.append(cs.canTime)
          if recv_frames > 5 or sent_frames % 5 == 0:
            gernModelInputs.send_json(list(input_array))
          carStateDataString2 += (carStateFormatString2 % (cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                                                            cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                                                            cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, \
                                                            cs.vEgo, cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr, cs.leftBlinker, cs.rightBlinker, cs.lkMode, cs.cruiseState.enabled, \
                                                            cs.camLeft.frame, cs.camLeft.parm1, cs.camLeft.parm2, cs.camLeft.parm3, cs.camLeft.parm4, cs.camLeft.parm5, cs.camLeft.parm6, cs.camLeft.parm7, cs.camLeft.parm8, cs.camLeft.parm9, cs.camLeft.parm10, cs.camLeft.solid, cs.camLeft.dashed, \
                                                            cs.camRight.frame, cs.camRight.parm1, cs.camRight.parm2, cs.camRight.parm3, cs.camRight.parm4, cs.camRight.parm5, cs.camRight.parm6, cs.camRight.parm7, cs.camRight.parm8, cs.camRight.parm9, cs.camRight.parm10, cs.camRight.solid, cs.camRight.dashed, \
                                                            cs.camFarLeft.frame, cs.camFarLeft.parm1, cs.camFarLeft.parm2, cs.camFarLeft.parm3, cs.camFarLeft.parm4, cs.camFarLeft.parm5, cs.camFarLeft.parm6, cs.camFarLeft.parm7, cs.camFarLeft.parm8, cs.camFarLeft.parm9, cs.camFarLeft.parm10, cs.camFarLeft.solid, cs.camFarLeft.dashed, \
                                                            cs.camFarRight.frame, cs.camFarRight.parm1, cs.camFarRight.parm2, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm6, cs.camFarRight.parm7, cs.camFarRight.parm8, cs.camFarRight.parm9, cs.camFarRight.parm10, cs.camFarRight.solid, cs.camFarRight.dashed, cs.canTime))
      elif cs.vEgo > 0:
        carStateDataString1 += (carStateFormatString1 % (cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                                                          cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                                                          cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, cs.canTime))
    
    if socket is gernModelOutputs:
      recv_frames += 1
      if recv_frames <= 5:  sent_frames = recv_frames

      output_list = list(socket.recv_json())
      model_output = np.asarray(output_list[:-1])
      if scaler_padding is None:
        column_count = Outputs
        row_count = len(model_output)//column_count
        scaler_padding = [np.zeros((row_count,Outputs)), np.zeros((row_count,Outputs))]
        left_center = np.zeros((row_count,1))
        right_center = np.zeros((row_count,1))
        calc_center = np.zeros((row_count,1))
        left_probs = np.zeros((row_count,1))
        right_probs = np.zeros((row_count,1))
        angle = np.zeros((row_count,1))

      model_output = model_output.reshape(row_count,column_count)

      scaler_padding[0] = np.asarray(model_output)
      descaled_output = [output_scaler.inverse_transform(scaler_padding[0]), output_scaler.inverse_transform(scaler_padding[1])]

      l_prob = l_probs.pop(output_list[-1])
      r_prob = r_probs.pop(output_list[-1])

      if l_prob < 0 and r_prob > 0 and descaled_output[0][-1:, 1:2] > -descaled_output[0][-1:, 2:3] * 1.2:
        l_prob *= 0.2
        #print("      Diverging Left", l_prob)
      elif r_prob < 0 and l_prob > 0 and descaled_output[0][-1:, 1:2] * 1.2 < -descaled_output[0][-1:,2:3]:
        r_prob *= 0.2
        #print("      Diverging Right", r_prob)
      elif abs(l_prob) > 0 and abs(r_prob) > 0:
        if lane_width > 0:
          lane_width += 0.01 * (min(700, max(570, l_offset[output_list[-1]] -  r_offset[output_list[-1]]) - lane_width))
        else:
          lane_width = min(700, max(570, l_offset[output_list[-1]] -  r_offset[output_list[-1]]) - lane_width)
          half_width = lane_width / 2
        half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.48))
      else:
        half_width = min(half_width + 1, max(half_width - 1, lane_width * 0.47))
   
      l_prob = abs(l_prob)
      r_prob = abs(r_prob)
      l_prob_smooth = l_prob  #max(0.05, l_prob_smooth - 0.1, min(l_prob_smooth + 0.1, l_prob))
      r_prob_smooth = r_prob  #max(0.05, r_prob_smooth - 0.1, min(r_prob_smooth + 0.1, r_prob))
      lr_prob = (l_prob_smooth + r_prob_smooth) - l_prob_smooth * r_prob_smooth
      a_prob = 1 

      left_probs[:,0] =    l_prob * np.clip(model_output[:,3], 0, 1) + (1 - l_prob) * left_probs[:,0]
      right_probs[:,0] =   r_prob * np.clip(model_output[:,4], 0, 1) + (1 - r_prob) * right_probs[:,0]
      left_center[:,0:]  =  l_prob *   (descaled_output[0][:,1:2] - half_width) + (1 - l_prob) * left_center[:, 0:]  
      right_center[:,0:]  = r_prob *   (descaled_output[0][:,2:3] + half_width) + (1 - r_prob) * right_center[:, 0:] 
      left_center = l_prob_smooth * left_center + (1 - l_prob_smooth) * calc_center
      right_center = r_prob_smooth * right_center + (1 - r_prob_smooth) * calc_center 
      
      angle = np.clip((descaled_output[0][:,0:1] - descaled_output[0][0,0:1]) * (1 + advanceSteer) + descaled_output[0][0,0:1], angle - 1.0, angle + 1.0)
      #angle = np.add(descaled_output[0][1:,0], np.multiply(np.diff(descaled_output[0][:,0]), advanceSteer))
      calc_center = (l_prob_smooth * left_center + r_prob_smooth * right_center) / (l_prob_smooth + r_prob_smooth + 0.05) 

      path_send.pathPlan.mpcAngles = [float(x) for x in angle[:]]
      path_send.pathPlan.angleSteers = float(angle[5])
      path_send.pathPlan.laneWidth = float(lane_width)
      path_send.pathPlan.lPoly = [float(x) for x in (left_center[:,0] + half_width)]
      path_send.pathPlan.rPoly = [float(x) for x in (right_center[:,0] - half_width)]
      path_send.pathPlan.cPoly = [float(x) for x in (calc_center[:,0])]
      path_send.pathPlan.lProb = float(l_prob)
      path_send.pathPlan.rProb = float(r_prob)
      path_send.pathPlan.cProb = float(lr_prob)
      path_send.pathPlan.canTime = output_list[-1]
      gernPath.send(path_send.to_bytes())
      if cs.vEgo >= 0:
        pathDataString += pathFormatString1 % tuple([float(x) for x in (left_center[:,0] + half_width)])
        pathDataString += pathFormatString2 % tuple([float(x) for x in (right_center[:,0] - half_width)])
        pathDataString += pathFormatString3 % tuple([float(x) for x in calc_center[:,0]])
        pathDataString += pathFormatString4 % (path_send.pathPlan.mpcAngles[3], path_send.pathPlan.mpcAngles[4], path_send.pathPlan.mpcAngles[5], path_send.pathPlan.mpcAngles[6], 
                          path_send.pathPlan.mpcAngles[10], path_send.pathPlan.lProb, path_send.pathPlan.rProb, path_send.pathPlan.cProb, path_send.pathPlan.laneWidth, 
                          path_send.pathPlan.angleSteers, path_send.pathPlan.rateSteers, path_send.pathPlan.canTime - path_send.pathPlan.canTime, cs.canTime)
      path_send = log.Event.new_message()
      path_send.init('pathPlan')
      if recv_frames % 30 == 0:
        #try:
        print(' sent: %d dropped: %d backlog: %d half_width: %0.1f center: %0.1f  l_prob:  %0.2f  r_prob:  %0.2f  advance_steer:  %0.2f' % (sent_frames, sent_frames - recv_frames, back_log, half_width, calc_center[-1], l_prob, r_prob, advanceSteer))

    if frame_count >= 100 and back_log == 1:
      try:
        r = requests.post(url_string, data=pathDataString + carStateDataString1 + carStateDataString2)
        #print(influxLineString)
        if dashboard_count % 3 == 0: print('%d %s' % (frame_count, r))
        dashboard_count += 1
      except:
        r = requests.post(url_string, data='create database carDB')
        print(r)
      # Send data to influxdb (after converting to Python3.7)
      carStateDataString2 = ''
      carStateDataString1 = ''
      pathDataString = ''
      frame_count = 0

    if recv_frames % 100 == 0 and back_log == 2:
      try:
        kegman = kegman_conf()  
        advanceSteer = max(0, float(kegman.conf['advanceSteer']))
        #print("advanceSteer = ", advanceSteer)
      except:
        pass

