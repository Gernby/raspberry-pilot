#!/usr/bin/env python
import zmq
import time
import os
import sys
import json
import selfdrive.messaging as messaging
from selfdrive.services import service_list
from common.params import Params
import numpy as np
from setproctitle import setproctitle
import requests

url_string = 'http://127.0.0.1:8086/write?db=carDB&u=liveOP&p=liveOP&precision=ms'

def dashboard_thread(rate=100):

  kegman_valid = True

  setproctitle('dashboard')
  op_address = "127.0.0.1"
  if len(sys.argv) < 2 or sys.argv[1] == 0:
    server_address = "gernstation.synology.me"
  elif int(sys.argv[1]) == 1:
    server_address = "192.168.137.1"
  elif int(sys.argv[1]) == 2:
    server_address = "192.168.1.3"
  elif int(sys.argv[1]) == 3:
    server_address = "192.168.43.221"
  elif int(sys.argv[1]) == 4:
    server_address = "kevo.live"
  elif int(sys.argv[1]) == 5:
    server_address = "192.168.137.1"
    op_address = "192.168.137.7"
  else:
    server_address = None

  print("using %s" % (server_address))

  context = zmq.Context()
  poller = zmq.Poller()
  vEgo = 0.0
  controlsState = messaging.sub_sock(service_list['controlsState'].port, addr=op_address)
  carState = messaging.sub_sock(service_list['carState'].port, conflate=False)
  canData = None #messaging.sub_sock(8602)
  pathPlan = messaging.sub_sock(service_list['pathPlan'].port,addr=op_address)
  tuneSub = None #messaging.sub_sock("tcp://" + server_address + ":8596")
  steerPush = context.socket(zmq.PUSH)
  steerPush.connect("tcp://" + server_address + ":8593")
  #tunePush = context.socket(zmq.PUSH)
  #tunePush.connect("tcp://" + server_address + ":8595")

  #if controlsState != None: poller.register(controlsState, zmq.POLLIN)
  if pathPlan != None: poller.register(pathPlan, zmq.POLLIN)
  if carState != None: poller.register(carState, zmq.POLLIN)

  frame_count = 0
  user_id = 'ddd3e089e7bbe0fc'
  try:
    if os.path.isfile(os.path.expanduser('~/kegman.json')):
      with open(os.path.expanduser('~/kegman.json'), 'r') as f:
        config = json.load(f)
        user_id = config['userID']
        #tunePush.send_json(config)
        #tunePush = None
    else:
        params = Params()
        user_id = params.get("DongleId")
  except:
    params = Params()
    user_id = params.get("DongleId")
    config['userID'] = user_id
    #tunePush.send_json(config)
    #tunePush = None
  print(user_id)
  try:
    if len(user_id) < 2: user_id = 'ddd3e089e7bbe0fc'
  except:
    user_id = 'ddd3e089e7bbe0fc'

  #tunePush.send_json(config)
  #tunePush = None
  #tuneSub.setsockopt(zmq.SUBSCRIBE, str(user_id))
  kegmanFormatString = user_id + ",sources=kegman dampMPC=%0.3f,reactMPC=%0.3f,dampSteer=%0.3f,polyFactor=%0.5f,dampPoly=%0.3f,reactPoly=%0.3f %d\n"
  canFormatString="CANData,user=" + user_id + ",src=%d,pid=%d d1=%di,d2=%di "
  pathFormatString1 = "pathPlan,user=" + user_id + " l0=%0.3f,l1=%0.3f,l2=%0.3f,l3=%0.3f,l4=%0.3f,l5=%0.3f,l6=%0.3f,l7=%0.3f,l8=%0.3f,l9=%0.3f,l10=%0.3f,l11=%0.3f,l12=%0.3f,l13=%0.3f,l14=%0.3f,"
  pathFormatString2 = "r0=%0.3f,r1=%0.3f,r2=%0.3f,r3=%0.3f,r4=%0.3f,r5=%0.3f,r6=%0.3f,r7=%0.3f,r8=%0.3f,r9=%0.3f,r10=%0.3f,r11=%0.3f,r12=%0.3f,r13=%0.3f,r14=%0.3f,"
  pathFormatString3 = "c0=%0.3f,c1=%0.3f,c2=%0.3f,c3=%0.3f,c4=%0.3f,c5=%0.3f,c6=%0.3f,c7=%0.3f,c8=%0.3f,c9=%0.3f,c10=%0.3f,c11=%0.3f,c12=%0.3f,c13=%0.3f,c14=%0.3f,"
  pathFormatString4 = "a3=%0.3f,a4=%0.3f,a5=%0.3f,a6=%0.3f,a10=%0.3f,lprob=%0.3f,rprob=%0.3f,cprob=%0.3f,lane_width=%0.3f,angle=%0.3f,rate=%0.3f,plan_age=%0.3f %d\n"
  carStateFormatString2 = "carState,user=" + user_id + " angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d,v_ego=%0.4f,wheel_speed_fl=%0.4f,wheel_speed_fr=%0.4f,wheel_speed_rl=%0.4f,wheel_speed_rr=%0.4f,l_blinker=%d,r_blinker=%d,lk_mode=%d,enabled=%d,left_frame=%d,left_1=%d,left_2=%d,left_3=%d,left_4=%d,left_5=%d,left_6=%d,left_7=%d,left_8=%d,left_9=%d,left_10=%d,left_solid=%d,left_dashed=%d,right_frame=%d,right_1=%d,right_2=%d,right_3=%d,right_4=%d,right_5=%d,right_6=%d,right_7=%d,right_8=%d,right_9=%d,right_10=%d,right_solid=%d,right_dashed=%d,far_left_frame=%d,far_left_1=%d,far_left_2=%d,far_left_3=%d,far_left_4=%d,far_left_5=%d,far_left_6=%d,far_left_7=%d,far_left_8=%d,far_left_9=%d,far_left_10=%d,far_left_solid=%d,far_left_dashed=%d,far_right_frame=%d,far_right_1=%d,far_right_2=%d,far_right_3=%d,far_right_4=%d,far_right_5=%d,far_right_6=%d,far_right_7=%d,far_right_8=%d,far_right_9=%d,far_right_10=%d,far_right_solid=%d,far_right_dashed=%d %d\n"
  carStateFormatString1 = "carState,user=" + user_id + " angle_steers=%0.4f,angle_rate=%0.4f,driver_torque=%0.4f,request=%0.4f,angle_rate_eps=%0.4f,yaw_rate_can=%0.4f,angle_steers_eps=%0.4f,long_accel=%0.4f,p2=%0.4f,p=%0.4f,i=%0.4f,f=%0.4f,damp_angle_steers=%0.4f,damp_angle_steers_des=%0.4f,ff_rate=%0.4f,ff_angle=%0.4f,left_frame=%d,far_right_frame=%d %d\n"
  pathDataString = ""
  kegmanDataString = ""
  insertString = ""
  canInsertString = ""

  frame = 0
  active = False
  stock_cam_frame_prev = 0
  cs = None

  messaging.drain_sock(carState, True)

  while 1:
    for socket, event in poller.poll(3000):
      if socket is carState:
        _carState = messaging.drain_sock(socket)
        for _cs in _carState:
          receiveTime = int(_cs.carState.canTime)
          cs = _cs.carState
          vEgo = cs.vEgo
          if False cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
            stock_cam_frame_prev = cs.camLeft.frame
            carStateDataString2 += (carStateFormatString2 % (cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                                                              cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                                                              cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, \
                                                              vEgo, cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr, cs.leftBlinker, cs.rightBlinker, cs.lkMode, cs.cruiseState.enabled, \
                                                              cs.camLeft.frame, cs.camLeft.parm1, cs.camLeft.parm2, cs.camLeft.parm3, cs.camLeft.parm4, cs.camLeft.parm5, cs.camLeft.parm6, cs.camLeft.parm7, cs.camLeft.parm8, cs.camLeft.parm9, cs.camLeft.parm10, cs.camLeft.solid, cs.camLeft.dashed, \
                                                              cs.camRight.frame, cs.camRight.parm1, cs.camRight.parm2, cs.camRight.parm3, cs.camRight.parm4, cs.camRight.parm5, cs.camRight.parm6, cs.camRight.parm7, cs.camRight.parm8, cs.camRight.parm9, cs.camRight.parm10, cs.camRight.solid, cs.camRight.dashed, \
                                                              cs.camFarLeft.frame, cs.camFarLeft.parm1, cs.camFarLeft.parm2, cs.camFarLeft.parm3, cs.camFarLeft.parm4, cs.camFarLeft.parm5, cs.camFarLeft.parm6, cs.camFarLeft.parm7, cs.camFarLeft.parm8, cs.camFarLeft.parm9, cs.camFarLeft.parm10, cs.camFarLeft.solid, cs.camFarLeft.dashed, \
                                                              cs.camFarRight.frame, cs.camFarRight.parm1, cs.camFarRight.parm2, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm6, cs.camFarRight.parm7, cs.camFarRight.parm8, cs.camFarRight.parm9, cs.camFarRight.parm10, cs.camFarRight.solid, cs.camFarRight.dashed, receiveTime))
          elif cs.vEgo > 0:
            carStateDataString1 += (carStateFormatString1 % (cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                                                              cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                                                              cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.canTime))

          frame_count += 1

      if socket is pathPlan:
        _pathPlan = messaging.drain_sock(socket)
        for _pp in _pathPlan:
          pp = _pp.pathPlan
          if vEgo >= 0 and not cs is None:
            pathDataString += pathFormatString1 % tuple([float(x) for x in pp.lPoly[:15]])
            pathDataString += pathFormatString2 % tuple([float(x) for x in pp.rPoly[:15]])
            pathDataString += pathFormatString3 % tuple([float(x) for x in pp.cPoly[:15]])
            pathDataString += pathFormatString4 % (pp.mpcAngles[3], pp.mpcAngles[4], pp.mpcAngles[5], pp.mpcAngles[6], pp.mpcAngles[10], pp.lProb, pp.rProb, pp.cProb, pp.laneWidth, pp.angleSteers, pp.rateSteers, cs.canTime - pp.canTime, cs.canTime)
        frame += 1

      if socket is tuneSub:
        config = json.loads(tuneSub.recv_multipart()[1])
        #print(config)
        with open(os.path.expanduser('~/kegman.json'), 'w') as f:
          json.dump(config, f, indent=2, sort_keys=True)
          os.chmod(os.path.expanduser("~/kegman.json"), 0o764)

      if socket is canData:
        canString = canData.recv_string()
        #print(canString)
        if vEgo > 0 and active: canInsertString += canFormatString + str(cs.canTime) + "\n~" + canString + "!"
        frame += 1

    #print(frame_count)
    if frame_count >= 200:
      if kegman_valid:
        try:
          if os.path.isfile(os.path.expanduser('~/kegman.json')):
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

              kegmanDataString += (kegmanFormatString % \
                    (reactRate, dampRate, longOffset, backlash, dampMPC, reactMPC, dampSteer, reactSteer, steerKpV, steerKiV, rateFF, cs.angleFFGain, delaySteer,
                    oscFactor, centerFactor, polyReact, polyDamp, polyScale, cs.canTime))
        except:
          kegman_valid = False

      try:
        r = requests.post(url_string, data=canInsertString + pathDataString + carStateDataString1 + carStateDataString2)
        #print(influxLineString)
        print('%d %s' % (frame_count, r))
      except:
        print(r)

      frame_count = 0
      kegmanDataString = ""
      pathDataString = ""
      canInsertString = ""
      carStateDataString1 = ""
      carStateDataString2 = ""

def main(rate=200):
  dashboard_thread(rate)

if __name__ == "__main__":
  main()
