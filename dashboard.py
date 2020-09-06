#!/usr/bin/env python
import zmq
import time
import gc
import os
import sys
import json
from cereal import log, car
import selfdrive.messaging as messaging
from selfdrive.services import service_list
from common.params import Params
from common.profiler import Profiler
import numpy as np
from setproctitle import setproctitle
from selfdrive.kegman_conf import kegman_conf
import requests

SERVER_ADDRESS = "gernstation.synology.me"
#SERVER_ADDRESS = '192.168.1.3'   

setproctitle('dashboard')

frame_count = 0
params = Params()
profiler = Profiler(False, 'dashboard')
user_id = str(params.get("PandaDongleId", True))
user_id = user_id.replace("'","")

try:
  car_params = car.CarParams.from_bytes(params.get('CarParams', True))
  kegman = kegman_conf(car_params)  
  do_influx = True if kegman.conf['useInfluxDB'] == '1' else False
  kegman_valid = ('tuneRev' in kegman.conf)
except:
  print("kegman error")
  kegman_valid = False
  do_influx = False

do_send_live = False
target_address = '127.0.0.1'
cred = ''  #'u=liveOP&p=liveOP&'
target_URL = 'http://%s:8086/write?db=carDB&%sprecision=ms' % (target_address, cred)
#target_URL = 'http://192.168.137.1:8086/write?db=carDB&precision=ms' 
print(target_URL)

context = zmq.Context()
poller = zmq.Poller()
vEgo = 0.0

carState = messaging.sub_sock(service_list['carState'].port, conflate=False)
pathPlan = messaging.sub_sock(service_list['pathPlan'].port, conflate=True)
heartBeatSub = messaging.sub_sock(8597, addr=SERVER_ADDRESS, conflate=True)

do_send_live = False
serverPush = None   
tuneSub = None
     
if pathPlan != None: poller.register(pathPlan, zmq.POLLIN)
if carState != None: poller.register(carState, zmq.POLLIN)
if heartBeatSub != None: poller.register(heartBeatSub, zmq.POLLIN)

kegmanInsertString = ""
serverCanFormatString="CANData,user=" + user_id + ",src=;pid=; d1=;i,d2=;i; ~"
serverPathFormatString = "pathPlan,user=" + user_id + " l0=;l1=;l2=;l3=;l4=;l5=;l6=;l7=;l8=;l9=;l10=;l11=;l12=;l13=;l14=;r0=;r1=;r2=;r3=;r4=;r5=;r6=;r7=;r8=;r9=;r10=;r11=;r12=;r13=;r14=;c0=;c1=;c2=;c3=;c4=;c5=;c6=;c7=;c8=;c9=;c10=;c11=;c12=;c13=;c14=;a0=;a1=;a3=;a5=;a7=;a14=;fa0=;fa1=;fa3=;fa5=;fa7=;fa14=;lprob=;rprob=;cprob=;center_comp=;lane_width=;angle=;rate=;angle_offset=;lateral_offset=;actual_angle=;plan_age=; ~"
serverPathDataFormatString = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d|"
serverPolyDataFormatString = "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,"
serverPathDataString = []
kegmanDataString = []
serverInsertString = []
serverCanInsertString = []
serverCarStateFormatString2 = "carState,user=" + user_id + " v_ego=;econ_mode=;adjusted_angle=;angle_steers=;angle_rate=;driver_torque=;request=;angle_rate_eps=;yaw_rate_can=;angle_steers_eps=;long_accel=;p2=;p=;i=;f=;damp_angle_steers=;damp_angle_steers_des=;ff_rate=;ff_angle=;calc_rate=;fast_calc_rate=;left_frame=;far_right_frame=;wheel_speed_fl=;wheel_speed_fr=;wheel_speed_rl=;wheel_speed_rr=;l_blinker=;r_blinker=;lk_mode=;enabled=;left_frame=;left_1=;left_2=;left_3=;left_4=;left_5=;left_6=;left_7=;left_8=;left_9=;left_10=;left_11=;left_12=;left_13=;left_full_1=;left_full_2=;right_frame=;right_1=;right_2=;right_3=;right_4=;right_5=;right_6=;right_7=;right_8=;right_9=;right_10=;right_11=;right_12=;right_13=;right_full_1=;right_full_2=;far_left_frame=;far_left_1=;far_left_2=;far_left_3=;far_left_4=;far_left_5=;far_left_6=;far_left_7=;far_left_9=;far_left_8=;far_left_10=;far_left_11=;far_left_12=;far_left_13=;far_left_full_1=;far_left_full_2=;far_right_frame=;far_right_1=;far_right_2=;far_right_3=;far_right_4=;far_right_5=;far_right_6=;far_right_7=;far_right_8=;far_right_9=;far_right_10=;far_right_11=;far_right_12=;far_right_13=;far_right_full_1=;far_right_full_2=;gflags=;glat=;glong=;galt=;gspeed=;gbearing=;gaccuracy=;gNED0=;gNED1=;gNED2=;gvertAccuracy=;gbearingAccuracy=;gspeedAccuracy=;time_delta=; ~"
serverCarStateFormatString1 = "carState,user=" + user_id + " v_ego=;angle_steers=;angle_rate=;driver_torque=;request=;angle_rate_eps=;yaw_rate_can=;angle_steers_eps=;long_accel=;p2=;p=;i=;f=;damp_angle_steers=;damp_angle_steers_des=;ff_rate=;ff_angle=;fast_calc_rate=;left_frame=;far_right_frame=;time_delta=; ~"

serverCarStateDataFormatString2 = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d|"
serverCarStateDataFormatString1 = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d|"
serverCarStateDataString1 = []
serverCarStateDataString2 = []
localPathFormatString1 = 'pathPlan,user=' + user_id + ' l0=%0.3f,l1=%0.3f,l2=%0.3f,l3=%0.3f,l4=%0.3f,l5=%0.3f,l6=%0.3f,l7=%0.3f,l8=%0.3f,l9=%0.3f,l10=%0.3f,l11=%0.3f,l12=%0.3f,l13=%0.3f,l14=%0.3f,'
localPathFormatString2 = "r0=%0.3f,r1=%0.3f,r2=%0.3f,r3=%0.3f,r4=%0.3f,r5=%0.3f,r6=%0.3f,r7=%0.3f,r8=%0.3f,r9=%0.3f,r10=%0.3f,r11=%0.3f,r12=%0.3f,r13=%0.3f,r14=%0.3f,"
localPathFormatString3 = "c0=%0.3f,c1=%0.3f,c2=%0.3f,c3=%0.3f,c4=%0.3f,c5=%0.3f,c6=%0.3f,c7=%0.3f,c8=%0.3f,c9=%0.3f,c10=%0.3f,c11=%0.3f,c12=%0.3f,c13=%0.3f,c14=%0.3f,"
localPathFormatString4 = "a0=%0.3f,a1=%0.3f,a3=%0.3f,a6=%0.3f,a10=%0.3f,a14=%0.3f,fa0=%0.3f,fa1=%0.3f,fa3=%0.3f,fa6=%0.3f,fa10=%0.3f,fa14=%0.3f,lprob=%0.3f,rprob=%0.3f,cprob=%0.3f,lane_width=%0.3f,angle=%0.3f,rate=%0.3f,angle_offset=%0.2f,lateral_offset=%0.2f,actual_angle=%0.1f,plan_age=%0.3f %d\n"
#localCarStateFormatString2 = "carState,user=" + user_id + " v_ego=%0.4f,econ_mode=%d,adjusted_angle=%0.2f,angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d,wheel_speed_fl=%0.4f,wheel_speed_fr=%0.4f,wheel_speed_rl=%0.4f,wheel_speed_rr=%0.4f,l_blinker=%d,r_blinker=%d,lk_mode=%d,enabled=%d,left_frame=%d,left_1=%d,left_2=%d,left_3=%d,left_4=%d,left_5=%d,left_6=%d,left_7=%d,left_8=%d,left_9=%d,left_10=%d,left_11=%d,left_12=%d,left_13=%d,left_full_1=%d,left_full_2=%d,right_frame=%d,right_1=%d,right_2=%d,right_3=%d,right_4=%d,right_5=%d,right_6=%d,right_7=%d,right_8=%d,right_9=%d,right_10=%d,right_11=%d,right_12=%d,right_13=%d,right_full_1=%d,right_full_2=%d,far_left_frame=%d,far_left_1=%d,far_left_2=%d,far_left_3=%d,far_left_4=%d,far_left_5=%d,far_left_6=%d,far_left_7=%d,far_left_9=%d,far_left_8=%d,far_left_10=%d,far_left_11=%d,far_left_12=%d,far_left_13=%d,far_left_full_1=%d,far_left_full_2=%d,far_right_frame=%d,far_right_1=%d,far_right_2=%d,far_right_3=%d,far_right_4=%d,far_right_5=%d,far_right_6=%d,far_right_7=%d,far_right_8=%d,far_right_9=%d,far_right_10=%d,far_right_11=%d,far_right_12=%d,far_right_13=%d,far_right_full_1=%d,far_right_full_2=%d,time_delta=%d %d\n"

localCarStateFormatString2 = "carState,user=" + user_id + " v_ego=%0.4f,econ_mode=%d,adjusted_angle=%0.2f,angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,calc_rate=%0.4f,fast_calc_rate=%0.4f,left_frame=%d,far_right_frame=%d,wheel_speed_fl=%0.4f,wheel_speed_fr=%0.4f,wheel_speed_rl=%0.4f,wheel_speed_rr=%0.4f,l_blinker=%d,r_blinker=%d,lk_mode=%d,enabled=%d,left_frame=%d,left_1=%d,left_2=%d,left_3=%d,left_4=%d,left_5=%d,left_6=%d,left_7=%d,left_8=%d,left_9=%d,left_10=%d,left_11=%d,left_12=%d,left_13=%d,left_full_1=%d,left_full_2=%d,right_frame=%d,right_1=%d,right_2=%d,right_3=%d,right_4=%d,right_5=%d,right_6=%d,right_7=%d,right_8=%d,right_9=%d,right_10=%d,right_11=%d,right_12=%d,right_13=%d,right_full_1=%d,right_full_2=%d,far_left_frame=%d,far_left_1=%d,far_left_2=%d,far_left_3=%d,far_left_4=%d,far_left_5=%d,far_left_6=%d,far_left_7=%d,far_left_9=%d,far_left_8=%d,far_left_10=%d,far_left_11=%d,far_left_12=%d,far_left_13=%d,far_left_full_1=%d,far_left_full_2=%d,far_right_frame=%d,far_right_1=%d,far_right_2=%d,far_right_3=%d,far_right_4=%d,far_right_5=%d,far_right_6=%d,far_right_7=%d,far_right_8=%d,far_right_9=%d,far_right_10=%d,far_right_11=%d,far_right_12=%d,far_right_13=%d,far_right_full_1=%d,far_right_full_2=%d,gflags=%d,glat=%f,glong=%f,galt=%f,gspeed=%f,gbearing=%f,gaccuracy=%f,gNED0=%f,gNED1=%f,gNED2=%f,gvertAccuracy=%f,gbearingAccuracy=%f,gspeedAccuracy=%f,time_delta=%d %d\n"
localCarStateFormatString1 = "carState,user=" + user_id + " v_ego=%0.4f,angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,fast_calc_rate=%0.4f,left_frame=%d,far_right_frame=%d,time_delta=%d %d\n"
localPathDataString = []
kegmanDataString = []
localCarStateDataString1 = []
localCarStateDataString2 = []
insertString = []
fileStrings = []
canInsertString = []
next_beat_check = time.time() + 20
live_stream_lag = 10
angle_offset = 0
angle_bias = 0

frame = 0
active = False
stock_cam_frame_prev = 0
cs = None
lastHeartBeat = 0
gpsCount = 0
kegtime_prev = 0

messaging.drain_sock(carState, True)
messaging.drain_sock(pathPlan, False)
messaging.drain_sock(carState, False)

prev_angle = 0
prev_time = 0
fast_prev_angle = 0
fast_prev_time = 0

previous_minute = 0
logfile = None
cs = None
if not os.path.exists('/data/upload/'):
  os.mkdir('/data/upload')

def is_number(string):
  try:
    float(string)
    return True
  except ValueError:
    return False
        
while 1:

  for socket, event in poller.poll(3000):
    profiler.checkpoint('poller', False)

    if socket is carState:
      # Wait 4 control cycles to let Transcoderd do its good stuff
      time.sleep(0.04)
      for _cs in carState.recv_multipart():
        cs = log.Event.from_bytes(_cs).carState
        vEgo = cs.vEgo
        if vEgo > 0 and cs.canTime//60000 > previous_minute:
          profiler.checkpoint('carstate')
          time.sleep(0.00001)
          if not logfile is None: logfile.close()
          previous_minute = cs.canTime//60000
          logfile = open('/data/upload/%s_%0.0f.dat' % (user_id, time.time()//60), "a")
          profiler.checkpoint('create_file')
          time.sleep(0.00001)

        fast_calculated_rate = 1000 * (cs.steeringAngle - fast_prev_angle) / max(0.00001, cs.sysTime - fast_prev_time)
        fast_prev_angle = cs.steeringAngle
        fast_prev_time = cs.sysTime

        if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
          gps = cs.gpsLocation
          if gps.timestamp == 0 or len(gps.vNED) == 0: 
            vNED = [0,0,0]
          else:
            vNED = gps.vNED
          stock_cam_frame_prev = cs.camLeft.frame
          calculated_rate = 1000 * (cs.steeringAngle - prev_angle) / max(0.00001, cs.sysTime - prev_time)
          prev_angle = cs.steeringAngle
          prev_time = cs.sysTime

          send_data = tuple([cs.vEgo, cs.econMode, cs.adjustedAngle, cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                            cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                            cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, calculated_rate, fast_calculated_rate, cs.camLeft.frame, cs.camFarRight.frame, \
                            cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr, cs.leftBlinker, cs.rightBlinker, cs.lkMode, cs.cruiseState.enabled, \
                            cs.camLeft.frame, cs.camLeft.parm1, cs.camLeft.parm2, cs.camLeft.parm3, cs.camLeft.parm4, cs.camLeft.parm5, cs.camLeft.parm6, cs.camLeft.parm7, cs.camLeft.parm8, cs.camLeft.parm9, cs.camLeft.parm10, cs.camLeft.parm11, cs.camLeft.parm12, cs.camLeft.parm13, cs.camLeft.full1, cs.camLeft.full2, \
                            cs.camRight.frame, cs.camRight.parm1, cs.camRight.parm2, cs.camRight.parm3, cs.camRight.parm4, cs.camRight.parm5, cs.camRight.parm6, cs.camRight.parm7, cs.camRight.parm8, cs.camRight.parm9, cs.camRight.parm10, cs.camRight.parm11, cs.camRight.parm12, cs.camRight.parm13, cs.camRight.full1, cs.camRight.full2, \
                            cs.camFarLeft.frame, cs.camFarLeft.parm1, cs.camFarLeft.parm2, cs.camFarLeft.parm3, cs.camFarLeft.parm4, cs.camFarLeft.parm5, cs.camFarLeft.parm6, cs.camFarLeft.parm7, cs.camFarLeft.parm8, cs.camFarLeft.parm9, cs.camFarLeft.parm10, cs.camFarLeft.parm11, cs.camFarLeft.parm12, cs.camFarLeft.parm13, cs.camFarLeft.full1, cs.camFarLeft.full2, \
                            cs.camFarRight.frame, cs.camFarRight.parm1, cs.camFarRight.parm2, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm6, cs.camFarRight.parm7, cs.camFarRight.parm8, cs.camFarRight.parm9, cs.camFarRight.parm10, cs.camFarRight.parm11, cs.camFarRight.parm12, cs.camFarRight.parm13, cs.camFarRight.full1, \
                            cs.camFarRight.full2, gps.flags, gps.latitude, gps.longitude, gps.altitude, gps.speed, gps.bearing, gps.accuracy, vNED[0], vNED[1], vNED[2], gps.verticalAccuracy, gps.bearingAccuracy, gps.speedAccuracy, cs.canTime - cs.sysTime, cs.canTime])
                            #cs.camFarRight.full2, cs.canTime - cs.sysTime, cs.canTime])
                            
          if do_influx:
            localCarStateDataString2.append(localCarStateFormatString2 % send_data)
          profiler.checkpoint('carstate')
          if vEgo > 0:
            #fileStrings.append(localCarStateFormatString2 % send_data)
            logfile.write(localCarStateFormatString2 % send_data)
            profiler.checkpoint('write_file')
            time.sleep(0.00001)
          if do_send_live and (frame % 2 == 0 or live_stream_lag < 3) and (vEgo > 0 or frame % 45 == 0):
            serverCarStateDataString2.append(serverCarStateDataFormatString2 % send_data)

        elif vEgo > 0:
          send_data = (cs.vEgo, cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                            cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                            cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, fast_calculated_rate, cs.camLeft.frame, cs.camFarRight.frame, cs.canTime - cs.sysTime, cs.canTime)
        
          profiler.checkpoint('carstate')
          #fileStrings.append(localCarStateFormatString1 % send_data)
          logfile.write(localCarStateFormatString1 % send_data)
          profiler.checkpoint('write_file')
          time.sleep(0.00001)
          if do_influx:
            localCarStateDataString1.append(localCarStateFormatString1 % send_data)
          if do_send_live and live_stream_lag < 1.5: 
            serverCarStateDataString1.append(serverCarStateDataFormatString1 % send_data)
      profiler.checkpoint('carstate')
      frame += 1
  
    if socket is pathPlan:
      pp = log.Event.from_bytes(pathPlan.recv()).pathPlan
      if not cs is None and not logfile is None:
        send_data0 = tuple(float(x) for x in tuple(pp.lPoly)[::1])
        send_data1 = tuple(float(x) for x in tuple(pp.rPoly)[::1])
        send_data2 = tuple(float(x) for x in tuple(pp.cPoly)[::1])
        #print(len(pp.fastAngles), len(pp.fastAngles[0]), np.round(pp.fastAngles,1))
        send_data3 = (pp.fastAngles[0][0], pp.fastAngles[0][1], pp.fastAngles[0][3], pp.fastAngles[0][5], pp.fastAngles[0][7], pp.fastAngles[0][14], pp.fastAngles[-1][0], pp.fastAngles[-1][1], pp.fastAngles[-1][3], pp.fastAngles[-1][5], pp.fastAngles[-1][7], pp.fastAngles[-1][14], pp.lProb, pp.rProb, pp.cProb, pp.centerCompensation, pp.laneWidth, pp.angleSteers, pp.rateSteers, pp.angleOffset, pp.lateralOffset, cs.steeringAngle, pp.canTime - pp.canTime, cs.canTime)

        if do_influx:
          localPathDataString.append("".join([localPathFormatString1 % send_data0, localPathFormatString2 % send_data1, localPathFormatString3 % send_data2, localPathFormatString4 % send_data3]))
        #if vEgo > 0: 
        #  logfile.write("".join([localPathFormatString1 % send_data0, localPathFormatString2 % send_data1, localPathFormatString3 % send_data2, localPathFormatString4 % send_data3]))
        if do_send_live and (vEgo > 0 or frame % 45 == 0):
          serverPathDataString.append("".join([serverPolyDataFormatString % send_data0, serverPolyDataFormatString % send_data1, serverPolyDataFormatString % send_data2,serverPathDataFormatString % send_data3]))
        profiler.checkpoint('pathplan')

    if socket is tuneSub:
      next_beat_check = time.time() + 20
      if not heartBeatSub is None: poller.unregister(heartBeatSub)
      heartBeatSub = None
      config = tuneSub.recv_multipart()
      config = json.loads(config[1])
      if 'time' in config:
        live_stream_lag = time.time() - float(config['time'])
        print("live stream lag: %0.1f" % live_stream_lag)
      else:
        print(config)
      do_send_live = True
      do_influx = False
      itemChanged = False
      try:
        for item in kegman.conf:
          if item in config and str(config[item]) != str(kegman.conf[item]) and float(config[item]) != float(kegman.conf[item]) and not item in ['identifier', 'time']:
            print(item, config[item], kegman.conf[item])
            kegman.conf[item] = str(config[item])
            itemChanged = True
          else:
            config[item] = kegman.conf[item]
        if itemChanged:
          kegman.element_updated = True
          kegman.write_config(kegman.conf)
          tunePush.send_json(kegman.conf)
      except:
        pass
      profiler.checkpoint('live_tune')

    if socket is heartBeatSub:
      heartBeatSub.recv()
      #print("  Got heart beat!")
      next_beat_check = time.time() + 20
      if not do_send_live:
        identifier = np.random.randint(0, high=10000)
        tuneSub = context.socket(zmq.SUB)
        tunePush = context.socket(zmq.PUSH)
        serverPush = context.socket(zmq.PUSH)
        tuneSub.connect("tcp://" + SERVER_ADDRESS + ":8596")
        tuneSub.setsockopt_string(zmq.SUBSCRIBE, str(identifier))
        tuneSub.RCVTIMEO = 20000
        poller.register(tuneSub, zmq.POLLIN)
        tunePush.connect("tcp://" + SERVER_ADDRESS + ":8595")
        kegman.conf.update({'identifier': identifier, 'userID': user_id, "time": time.time()})
        tunePush.send_json(kegman.conf)
        serverPush.connect("tcp://" + SERVER_ADDRESS + ":8601")
      time.sleep(0.00001)
      profiler.checkpoint('live_tune')

  if kegman_valid and not cs is None and not logfile is None:
    #try:
    (mode, ino, dev, nlink, uid, gid, size, atime, mtime, kegtime) = os.stat(os.path.expanduser('~/kegman.json'))
    if kegtime != kegtime_prev:
      kegtime_prev = kegtime
      kegman = kegman_conf() 
      kegmanInsertString = ["tuneData,user=" + user_id + " "]
      for key in kegman.conf:
        if is_number(str(kegman.conf[key])) and key not in ['identifier']:
          kegmanInsertString.append(key)
          kegmanInsertString.append("=")
          kegmanInsertString.append(str(kegman.conf[key]))
          kegmanInsertString.append(",")
      kegmanInsertString[-1] = " "
      kegmanInsertString.append(str(int(cs.canTime)))
      kegmanInsertString.append("\n")
      kegmanInsertString = "".join(kegmanInsertString)
      logfile.write(kegmanInsertString)
      print(kegmanInsertString)
    #except:
    #  kegman_valid = False

  if do_influx and len(localCarStateDataString2) >= 45:
    frame += 1
    insertString = "".join(["".join(localCarStateDataString2), "".join(localCarStateDataString1), "".join(localPathDataString), kegmanInsertString, "\n"])
    localCarStateDataString1 = []
    localCarStateDataString2 = []
    localPathDataString = []
    kegmanInsertString = ""
    if do_influx and frame > 5:
      try:
        time.sleep(0.00001)
        r = requests.post(target_URL, data=insertString)
        time.sleep(0.00001)
        #dashPub.send_string(insertString)
        if r.status_code == 404:
          #print(r)
          r = requests.post('http://localhost:8086/query?q=CREATE DATABASE carDB')
          #print(r)
        if frame % 3 == 0: print(len(insertString), r)
      except:
        try:
          r = requests.post('http://localhost:8086/query?q=CREATE DATABASE carDB')
        except:
          r = requests.post(target_URL, data='create database carDB')
    profiler.checkpoint('influx')

  elif do_send_live and ((vEgo > 0 and len(serverCarStateDataString2) >= 45) or (vEgo == 0 and len(serverCarStateDataString2) > 0)):
    insertString = [serverCarStateFormatString2, "".join(serverCarStateDataString2), "!", serverCarStateFormatString1, "".join(serverCarStateDataString1), "!", serverPathFormatString, "".join(serverPathDataString), "!", kegmanInsertString, "~|"]
    insertString = "".join(insertString)
    serverPush.send_json({'identifier': identifier, 'userID': user_id, "time": time.time(), "data": insertString})
    print(len(insertString), "  server sent")
    insertString = None
    serverCarStateDataString1 = []
    serverCarStateDataString2 = []
    serverPathDataString = []
    kegmanInsertString = ""
    kegparams = []
    time.sleep(0.00001)
    profiler.checkpoint('server_push')

  if kegman_valid and time.time() > next_beat_check:
    if not heartBeatSub is None: poller.unregister(heartBeatSub)
    heartBeatSub = messaging.sub_sock(8597, addr=SERVER_ADDRESS, conflate=True)
    poller.register(heartBeatSub, zmq.POLLIN)
    time.sleep(0.00001)
    if do_send_live:
      print('       Lost heart beat!')
      live_stream_lag = 10
      poller.unregister(tuneSub)
      tuneSub.close()
      serverPush.close()
      tunePush.close()
      tuneSub = None
      serverPush = None
      tunePush = None
      do_send_live = False
      gc.collect()
    else:
      print('    Heart beat check!')
    next_beat_check = time.time() + 20
    profiler.checkpoint('live_tune')

  if frame % 100 == 0 and profiler.enabled:
    frame += 1
    profiler.display()
    profiler.reset(True)
  
  #  time.sleep(0.02)
  #time.sleep(3)
  #tunePush.send_json(config)
