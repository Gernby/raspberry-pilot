#!/usr/bin/env python
import zmq
import time
import os
import sys
import json
from cereal import log, car
import selfdrive.messaging as messaging
from selfdrive.services import service_list
from common.params import Params
import numpy as np
from setproctitle import setproctitle
from selfdrive.kegman_conf import kegman_conf
import requests

SERVER_ADDRESS = "gernstation.synology.me"

setproctitle('dashboard')

frame_count = 0
params = Params()
user_id = str(params.get("PandaDongleId", True))
user_id = user_id.replace("'","")

try:
  kegman = kegman_conf()  
  do_influx = True if kegman.conf['useInfluxDB'] == '1' else False
except:
  do_influx = False

do_send_live = False
target_address = '127.0.0.1'
cred = 'u=liveOP&p=liveOP&'
target_URL = 'http://%s:8086/write?db=carDB&%sprecision=ms' % (target_address, cred)
#target_URL = 'http://192.168.137.1:8086/write?db=carDB&precision=ms' 
print(target_URL)

kegman_valid = False
context = zmq.Context()
poller = zmq.Poller()
vEgo = 0.0
carState = messaging.sub_sock(service_list['carState'].port, conflate=False)
pathPlan = messaging.sub_sock(service_list['pathPlan'].port, conflate=True)
tuneSub = None #messaging.sub_sock("tcp://" + server_address + ":8596")
#heartBeatSub = messaging.sub_sock(8602, addr=SERVER_ADDRESS, conflate=True)

#serverPush = context.socket(zmq.PUSH)
#serverPush.connect("tcp://" + SERVER_ADDRESS + ":8593")
if pathPlan != None and do_influx: poller.register(pathPlan, zmq.POLLIN)
#if heartBeatSub != None: poller.register(heartBeatSub, zmq.POLLIN)
if carState != None: poller.register(carState, zmq.POLLIN)
#dashPub = messaging.pub_sock(8597)

#if len(sys.argv) >= 2:

#tunePush.send_json(config)
#tunePush = None
#tuneSub.setsockopt(zmq.SUBSCRIBE, str(user_id))
serverKegmanFormatString = user_id + ",sources=kegman reactRate=%s,dampRate=%s,longOffset=%s,backlash=%s,dampMPC=%s,reactMPC=%s,dampSteer=%s,reactSteer=%s,KpV=%s,KiV=%s,rateFF=%s,angleFF=%s,delaySteer=%s,oscFactor=%s,centerFactor=%s,dampPoly=%s,reactPoly=%s %s\n"
serverCanFormatString="CANData,user=" + user_id + ",src=%s,pid=%s d1=%si,d2=%si "
serverPathFormatString = "pathPlan,user=" + user_id + " l0=%s,l1=%s,l2=%s,l3=%s,l4=%s,l5=%s,l6=%s,l7=%s,l8=%s,l9=%s,l10=%s,l11=%s,l12=%s,l13=%s,l14=%s,r0=%s,r1=%s,r2=%s,r3=%s,r4=%s,r5=%s,r6=%s,r7=%s,r8=%s,r9=%s,r10=%s,r11=%s,r12=%s,r13=%s,r14=%s,c0=%s,c1=%s,c2=%s,c3=%s,c4=%s,c5=%s,c6=%s,c7=%s,c8=%s,c9=%s,c10=%s,c11=%s,c12=%s,c13=%s,c14=%s,a0=%s,a1=%s,a2=%s,a3=%s,a4=%s,a5=%s,a6=%s,lprob=%s,rprob=%s,cprob=%s,lane_width=%s,angle=%s,rate=%s,angle_offset=%s,angle_bias=%s,plan_age=%s %s\n"
serverPathDataFormatString = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d|"
serverPolyDataFormatString = "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,"
serverPathDataString = []
kegmanDataString = []
serverInsertString = []
serverCanInsertString = []
serverCarStateFormatString2 = "carState,user=" + user_id + " adjusted_angle=%s,angle_steers=%s,angle_rate=%s,driver_torque=%s,request=%s,angle_rate_eps=%s,yaw_rate_can=%s,angle_steers_eps=%s,long_accel=%s,p2=%s,p=%s,i=%s,f=%s,damp_angle_steers=%s,damp_angle_steers_des=%s,ff_rate=%s,ff_angle=%s,left_frame=%s,far_right_frame=%s,v_ego=%s,wheel_speed_fl=%s,wheel_speed_fr=%s,wheel_speed_rl=%s,wheel_speed_rr=%s,l_blinker=%s,r_blinker=%s,lk_mode=%s,enabled=%s,left_frame=%s,left_1=%s,left_2=%s,left_3=%s,left_4=%s,left_5=%s,left_6=%s,left_7=%s,left_8=%s,left_9=%s,left_10=%s,left_11=%s,left_13=%s,right_frame=%s,right_1=%s,right_2=%s,right_3=%s,right_4=%s,right_5=%s,right_6=%s,right_7=%s,right_8=%s,right_9=%s,right_10=%s,right_11=%s,right_13=%s,far_left_frame=%s,far_left_1=%s,far_left_2=%s,far_left_3=%s,far_left_4=%s,far_left_5=%s,far_left_6=%s,far_left_7=%s,far_left_8=%s,far_left_9=%s,far_left_10=%s,far_left_11=%s,far_left_13=%s,far_right_frame=%s,far_right_1=%s,far_right_2=%s,far_right_3=%s,far_right_4=%s,far_right_5=%s,far_right_6=%s,far_right_7=%s,far_right_8=%s,far_right_9=%s,far_right_10=%s,far_right_11=%s,far_right_13=%s %s\n"
serverCarStateFormatString1 = "carState,user=" + user_id + " angle_steers=%s,angle_rate=%s,driver_torque=%s,request=%s,angle_rate_eps=%s,yaw_rate_can=%s,angle_steers_eps=%s,long_accel=%s,p2=%s,p=%s,i=%s,f=%s,damp_angle_steers=%s,damp_angle_steers_des=%s,ff_rate=%s,ff_angle=%s,left_frame=%s,far_right_frame=%s %s\n"
serverCarStateDataFormatString2 = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d|"
serverCarStateDataFormatString1 = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d|"
serverCarStateDataString1 = []
serverCarStateDataString2 = []
localPathFormatString1 = 'pathPlan,user=' + user_id + ' l0=%0.3f,l1=%0.3f,l2=%0.3f,l3=%0.3f,l4=%0.3f,l5=%0.3f,l6=%0.3f,l7=%0.3f,l8=%0.3f,l9=%0.3f,l10=%0.3f,l11=%0.3f,l12=%0.3f,l13=%0.3f,l14=%0.3f,'
localPathFormatString2 = "r0=%0.3f,r1=%0.3f,r2=%0.3f,r3=%0.3f,r4=%0.3f,r5=%0.3f,r6=%0.3f,r7=%0.3f,r8=%0.3f,r9=%0.3f,r10=%0.3f,r11=%0.3f,r12=%0.3f,r13=%0.3f,r14=%0.3f,"
localPathFormatString3 = "c0=%0.3f,c1=%0.3f,c2=%0.3f,c3=%0.3f,c4=%0.3f,c5=%0.3f,c6=%0.3f,c7=%0.3f,c8=%0.3f,c9=%0.3f,c10=%0.3f,c11=%0.3f,c12=%0.3f,c13=%0.3f,c14=%0.3f,"
localPathFormatString4 = "a0=%0.3f,a1=%0.3f,a2=%0.3f,a3=%0.3f,a4=%0.3f,a5=%0.3f,a6=%0.3f,lprob=%0.3f,rprob=%0.3f,cprob=%0.3f,lane_width=%0.3f,angle=%0.3f,rate=%0.3f,angle_offset=%0.2f,lateral_offset=%0.2f,actual_angle=%0.1f,plan_age=%0.3f %d\n"
localCarStateFormatString2 = "carState,user=" + user_id + " v_ego=%0.4f,econ_mode=%d,adjusted_angle=%0.2f,angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d,wheel_speed_fl=%0.4f,wheel_speed_fr=%0.4f,wheel_speed_rl=%0.4f,wheel_speed_rr=%0.4f,l_blinker=%d,r_blinker=%d,lk_mode=%d,enabled=%d,left_frame=%d,left_1=%d,left_2=%d,left_3=%d,left_4=%d,left_5=%d,left_6=%d,left_7=%d,left_8=%d,left_9=%d,left_10=%d,left_11=%d,left_12=%d,left_13=%d,left_full_1=%d,left_full_2=%d,right_frame=%d,right_1=%d,right_2=%d,right_3=%d,right_4=%d,right_5=%d,right_6=%d,right_7=%d,right_8=%d,right_9=%d,right_10=%d,right_11=%d,right_12=%d,right_13=%d,right_full_1=%d,right_full_2=%d,far_left_frame=%d,far_left_1=%d,far_left_2=%d,far_left_3=%d,far_left_4=%d,far_left_5=%d,far_left_6=%d,far_left_7=%d,far_left_9=%d,far_left_8=%d,far_left_10=%d,far_left_11=%d,far_left_12=%d,far_left_13=%d,far_left_full_1=%d,far_left_full_2=%d,far_right_frame=%d,far_right_1=%d,far_right_2=%d,far_right_3=%d,far_right_4=%d,far_right_5=%d,far_right_6=%d,far_right_7=%d,far_right_8=%d,far_right_9=%d,far_right_10=%d,far_right_11=%d,far_right_12=%d,far_right_13=%d,far_right_full_1=%d,far_right_full_2=%d %d\n"
localCarStateFormatString1 = "carState,user=" + user_id + " v_ego=%0.4f,angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d %d\n"
localPathDataString = []
kegmanDataString = []
localCarStateDataString1 = []
localCarStateDataString2 = []
insertString = []
fileStrings = []
canInsertString = []

angle_offset = 0
angle_bias = 0

frame = 0
active = False
stock_cam_frame_prev = 0
cs = None
lastHeartBeat = 0

messaging.drain_sock(carState, True)
messaging.drain_sock(pathPlan, False)
messaging.drain_sock(carState, False)
previous_minute = 0
logfile = None
if not os.path.exists('/data/upload/'):
  os.mkdir('/data/upload')

while 1:
  do_send_live = time.time() - lastHeartBeat < 60

  for socket, event in poller.poll(3000):
    if socket is carState:
      for _cs in carState.recv_multipart():
        cs = log.Event.from_bytes(_cs).carState
        vEgo = cs.vEgo
        if vEgo > 0 and time.time()//60 > previous_minute:
          if not logfile is None: logfile.close()
          previous_minute = time.time()//60
          logfile = open('/data/upload/%s_%0.0f.dat' % (user_id, time.time()//60), "a")

        if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
          stock_cam_frame_prev = cs.camLeft.frame
          send_data = tuple([cs.vEgo, cs.econMode, cs.adjustedAngle, cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                            cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                            cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, \
                            cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr, cs.leftBlinker, cs.rightBlinker, cs.lkMode, cs.cruiseState.enabled, \
                            cs.camLeft.frame, cs.camLeft.parm1, cs.camLeft.parm2, cs.camLeft.parm3, cs.camLeft.parm4, cs.camLeft.parm5, cs.camLeft.parm6, cs.camLeft.parm7, cs.camLeft.parm8, cs.camLeft.parm9, cs.camLeft.parm10, cs.camLeft.parm11, cs.camLeft.parm12, cs.camLeft.parm13, cs.camLeft.full1, cs.camLeft.full2, \
                            cs.camRight.frame, cs.camRight.parm1, cs.camRight.parm2, cs.camRight.parm3, cs.camRight.parm4, cs.camRight.parm5, cs.camRight.parm6, cs.camRight.parm7, cs.camRight.parm8, cs.camRight.parm9, cs.camRight.parm10, cs.camRight.parm11, cs.camRight.parm12, cs.camRight.parm13, cs.camRight.full1, cs.camRight.full2, \
                            cs.camFarLeft.frame, cs.camFarLeft.parm1, cs.camFarLeft.parm2, cs.camFarLeft.parm3, cs.camFarLeft.parm4, cs.camFarLeft.parm5, cs.camFarLeft.parm6, cs.camFarLeft.parm7, cs.camFarLeft.parm8, cs.camFarLeft.parm9, cs.camFarLeft.parm10, cs.camFarLeft.parm11, cs.camFarLeft.parm12, cs.camFarLeft.parm13, cs.camFarLeft.full1, cs.camFarLeft.full2, \
                            cs.camFarRight.frame, cs.camFarRight.parm1, cs.camFarRight.parm2, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm6, cs.camFarRight.parm7, cs.camFarRight.parm8, cs.camFarRight.parm9, cs.camFarRight.parm10, cs.camFarRight.parm11, cs.camFarRight.parm12, cs.camFarRight.parm13, cs.camFarRight.full1, cs.camFarRight.full2, cs.canTime])
                    
          if do_influx:
            localCarStateDataString2.append(localCarStateFormatString2 % send_data)
          if vEgo > 0:
            fileStrings.append(localCarStateFormatString2 % send_data)
            #logfile.write(localCarStateFormatString2 % send_data)
          if do_send_live:
            serverCarStateDataString2.append(serverCarStateDataFormatString2 % send_data)
        elif vEgo > 0:
          send_data = (cs.vEgo, cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                            cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                            cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, cs.canTime)
        
          fileStrings.append(localCarStateFormatString1 % send_data)
          #logfile.write(localCarStateFormatString1 % send_data)
          if do_influx:
            localCarStateDataString1.append(localCarStateFormatString1 % send_data)
          if do_send_live: 
            serverCarStateDataString1.append(serverCarStateDataFormatString1 % send_data)
      if vEgo > 0 and len(fileStrings) > 0: 
        logfile.write("".join(fileStrings))
      fileStrings.clear()

    if socket is pathPlan:
      pp = log.Event.from_bytes(pathPlan.recv()).pathPlan
      if not cs is None and not logfile is None:
        send_data0 = tuple(float(x) for x in tuple(pp.lPoly)[::1])
        send_data1 = tuple(float(x) for x in tuple(pp.rPoly)[::1])
        send_data2 = tuple(float(x) for x in tuple(pp.cPoly)[::1])
        send_data3 = (pp.slowAngles[0], pp.slowAngles[1], pp.slowAngles[2], pp.slowAngles[3], pp.slowAngles[4], pp.slowAngles[5], pp.slowAngles[6], pp.lProb, pp.rProb, pp.cProb, pp.laneWidth, pp.angleSteers, pp.rateSteers, pp.angleOffset, pp.lateralOffset, cs.steeringAngle, pp.canTime - pp.canTime, cs.canTime)

        if do_influx:
          localPathDataString.append("".join([localPathFormatString1 % send_data0, localPathFormatString2 % send_data1, localPathFormatString3 % send_data2, localPathFormatString4 % send_data3]))
        #if vEgo > 0: 
        #  logfile.write("".join([localPathFormatString1 % send_data0, localPathFormatString2 % send_data1, localPathFormatString3 % send_data2, localPathFormatString4 % send_data3]))
        if do_send_live:
          serverPathDataString.append("".join([serverPolyDataFormatString % send_data0, serverPolyDataFormatString % send_data1, serverPolyDataFormatString % send_data2,serverPathDataFormatString % send_data3]))

    '''if socket is tuneSub:
      config = json.loads(tuneSub.recv_multipart()[1])
      #print(config)
      with open(os.path.expanduser('~/kegman.json'), 'w') as f:
        json.dump(config, f, indent=2, sort_keys=True)
        os.chmod(os.path.expanduser("~/kegman.json"), 0o764)
    
    if socket is heartBeatSub:
      do_send_live = True
      print("         Heartbeat!")
      messaging.recv_one(heartBeatSub)
      lastHeartBeat = time.time()'''

  if do_influx and len(localCarStateDataString2) >= 15:
    frame += 1
    insertString = "".join(["".join(localCarStateDataString2), "".join(localCarStateDataString1), "".join(localPathDataString)])
    localCarStateDataString1 = []
    localCarStateDataString2 = []
    localPathDataString = []
    if do_influx and frame > 5:
      try:
        r = requests.post(target_URL, data=insertString)
        dashPub.send_string(insertString)
        if r.status_code == 404:
          print(r)
          r = requests.post('http://localhost:8086/query?q=CREATE DATABASE carDB')
          print(r)
        if frame % 3 == 0: print(len(insertString), r)
      except:
        try:
          r = requests.post('http://localhost:8086/query?q=CREATE DATABASE carDB')
        except:
          r = requests.post(target_URL, data='create database carDB')
  elif do_send_live and len(localCarStateDataString2) > 7 and len(serverCarStateDataString2) >= 15:
    insertString = [serverCarStateFormatString2, "~", "".join(serverCarStateDataString2), "!", serverCarStateFormatString1, "~", "".join(serverCarStateDataString1), "!"]
    insertString.extend([serverPathFormatString, "~", "".join(serverPathDataString), "!"])
    if kegman_valid:
      try:
        if False and os.path.isfile(os.path.expanduser('~/kegman.json')):
          with open(os.path.expanduser('~/kegman.json'), 'r') as f:
            config = json.load(f)
            reactMPC = config['reactMPC']
            dampMPC = config['dampMPC']
            reactSteer = config['reactSteer']
            dampSteer = config['dampSteer']
            delaySteer = config['delaySteer']
            steerKpV = config['Kp']
            steerKiV = config['Ki']
            rateFF = config['rateFF']
            oscFactor = config['oscFactor']
            backlash = config['backlash']
            longOffset = config['longOffset']
            dampRate = config['dampRate']
            reactRate = config['reactRate']
            centerFactor = config['centerFactor']
            polyReact = config['reactPoly']
            polyDamp = config['dampPoly']
            polyScale = config['scalePoly']

            insertString.append(serverKegmanFormatString, "~", "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s|" % \
                  (reactRate, dampRate, longOffset, backlash, dampMPC, reactMPC, dampSteer, reactSteer, steerKpV, steerKiV, rateFF, cs.angleFFGain, delaySteer,
                  oscFactor, centerFactor, polyReact, polyDamp, polyScale, cs.canTime), "!")
      except:
        kegman_valid = False
    insertString = "".join(insertString)
    serverPush.send_string(insertString)
    print(len(insertString), "  server sent")
    insertString = None
    serverCarStateDataString1 = []
    serverCarStateDataString2 = []
    serverPathDataString = []
  #if not do_influx:
  #  time.sleep(0.02)
