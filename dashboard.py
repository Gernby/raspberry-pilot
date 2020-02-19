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

def get_format_strings(measurement, user_id, data):
  insert_format_string = "%s,user=%s " % (measurement,user_id)
  data_format_string = ""
  for i in range(len(data)):
    insert_format_string += "%d=" % i + "%s,"
    data_format_string += "%f,"
  data_format_string = data_format_string[:-1]
  insert_format_string = insert_format_string + "gern_time=%s,mpc_time=%s,model_time=%s,path_time=%s %s\n"
  return insert_format_string, data_format_string

def get_format_string_arrays(measurements, user_id, data):
  data_format_strings = []
  for i in range(len(measurements)):
    format_strings = get_format_strings(measurements[i], user_id, data[i])
    data_format_strings.append(format_strings)
  return data_format_strings

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
  kegmanFormatString = user_id + ",sources=kegman reactRate=%s,dampRate=%s,longOffset=%s,backlash=%s,dampMPC=%s,reactMPC=%s,dampSteer=%s,reactSteer=%s,KpV=%s,KiV=%s,rateFF=%s,angleFF=%s,delaySteer=%s,oscFactor=%s,centerFactor=%s,dampPoly=%s,reactPoly=%s %s\n"
  canFormatString="CANData,user=" + user_id + ",src=%s,pid=%s d1=%si,d2=%si "
  pathFormatString = "pathPlan,user=" + user_id + " l0=%s,l1=%s,l2=%s,l3=%s,l4=%s,l5=%s,l6=%s,l7=%s,l8=%s,l9=%s,l10=%s,l11=%s,l12=%s,l13=%s,l14=%s,r0=%s,r1=%s,r2=%s,r3=%s,r4=%s,r5=%s,r6=%s,r7=%s,r8=%s,r9=%s,r10=%s,r11=%s,r12=%s,r13=%s,r14=%s,c0=%s,c1=%s,c2=%s,c3=%s,c4=%s,c5=%s,c6=%s,c7=%s,c8=%s,c9=%s,c10=%s,c11=%s,c12=%s,c13=%s,c14=%s,a3=%s,a4=%s,a5=%s,a6=%s,a10=%s,lprob=%s,rprob=%s,cprob=%s,lane_width=%s,angle=%s,rate=%s,plan_age=%s %s\n"
  pathDataFormatString = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d|"
  polyDataString = "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,"
  pathDataString = ""
  kegmanDataString = ""
  insertString = ""
  canInsertString = ""
  carStateFormatString2 = "carState,user=" + user_id + " angle_steers=%s,angle_rate=%s,driver_torque=%s,request=%s,angle_rate_eps=%s,yaw_rate_can=%s,angle_steers_eps=%s,long_accel=%s,p2=%s,p=%s,i=%s,f=%s,damp_angle_steers=%s,damp_angle_steers_des=%s,ff_rate=%s,ff_angle=%s,left_frame=%s,far_right_frame=%s,v_ego=%s,wheel_speed_fl=%s,wheel_speed_fr=%s,wheel_speed_rl=%s,wheel_speed_rr=%s,l_blinker=%s,r_blinker=%s,lk_mode=%s,enabled=%s,left_frame=%s,left_1=%s,left_2=%s,left_3=%s,left_4=%s,left_5=%s,left_6=%s,left_7=%s,left_8=%s,left_9=%s,left_10=%s,left_solid=%s,left_dashed=%s,right_frame=%s,right_1=%s,right_2=%s,right_3=%s,right_4=%s,right_5=%s,right_6=%s,right_7=%s,right_8=%s,right_9=%s,right_10=%s,right_solid=%s,right_dashed=%s,far_left_frame=%s,far_left_1=%s,far_left_2=%s,far_left_3=%s,far_left_4=%s,far_left_5=%s,far_left_6=%s,far_left_7=%s,far_left_8=%s,far_left_9=%s,far_left_10=%s,far_left_solid=%s,far_left_dashed=%s,far_right_frame=%s,far_right_1=%s,far_right_2=%s,far_right_3=%s,far_right_4=%s,far_right_5=%s,far_right_6=%s,far_right_7=%s,far_right_8=%s,far_right_9=%s,far_right_10=%s,far_right_solid=%s,far_right_dashed=%s %s\n"
  carStateFormatString1 = "carState,user=" + user_id + " angle_steers=%s,angle_rate=%s,driver_torque=%s,request=%s,angle_rate_eps=%s,yaw_rate_can=%s,angle_steers_eps=%s,long_accel=%s,p2=%s,p=%s,i=%s,f=%s,damp_angle_steers=%s,damp_angle_steers_des=%s,ff_rate=%s,ff_angle=%s,left_frame=%s,far_right_frame=%s %s\n"
  carStateDataFormatString2 = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d|"
  carStateDataFormatString1 = "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d|"
  carStateDataString1 = ""
  carStateDataString2 = ""

  frame = 0
  active = False
  stock_cam_frame_prev = 0
  cs = None

  messaging.drain_sock(carState, True)

  if False: 
    DIR = '~/logs/'
    filenumber = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])

    print("start")
    with open(DIR + '/dashboard_file_%d.csv' % filenumber, mode='w') as dash_file:
      print("opened")
      dash_writer = csv.writer(dash_file, delimiter=',', quotechar='', quoting=csv.QUOTE_NONE)
      print("initialized")
      dash_writer.writerow(['angleSteersDes','angleSteers','vEgo','steerOverride','upSteer','uiSteer','ufSteer','time'])
      print("first row")

      while 1:
        receiveTime = int(time.time() * 1000)
        for socket, event in poller.poll(0):
          if socket is live100:
            _live100 = messaging.recv_one(socket)
            vEgo = _live100.live100.vEgo
            if vEgo >= 0:
              frame_count += 1
              dash_writer.writerow([str(round(_live100.live100.angleSteersDes, 2)),
                                    str(round(_live100.live100.angleSteers, 2)),
                                    str(round(_live100.live100.vEgo, 1)),
                                    1 if _live100.live100.steerOverride else 0,
                                    str(round(_live100.live100.upSteer, 4)),
                                    str(round(_live100.live100.uiSteer, 4)),
                                    str(round(_live100.live100.ufSteer, 4)),
                                    str(receiveTime)])
            else:
              skipped_count += 1
          else:
            skipped_count += 1
        if frame_count % 200 == 0:
          print("captured = %d" % frame_count)
          frame_count += 1
        if skipped_count % 200 == 0:
          print("skipped = %d" % skipped_count)
          skipped_count += 1


  while 1:
    for socket, event in poller.poll(3000):
      if socket is carState:
        _carState = messaging.drain_sock(socket)
        for _cs in _carState:
          #_controlsState = messaging.recv_one(controlsState)
          #cs = _controlsState.controlsState
          receiveTime = int(_cs.carState.canTime)
          cs = _cs.carState
          vEgo = cs.vEgo
          if cs.camLeft.frame != stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
            stock_cam_frame_prev = cs.camLeft.frame
            carStateDataString2 += (carStateDataFormatString2 % (cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                                                              cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                                                              cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, \
                                                              vEgo, cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr, cs.leftBlinker, cs.rightBlinker, cs.lkMode, cs.cruiseState.enabled, \
                                                              cs.camLeft.frame, cs.camLeft.parm1, cs.camLeft.parm2, cs.camLeft.parm3, cs.camLeft.parm4, cs.camLeft.parm5, cs.camLeft.parm6, cs.camLeft.parm7, cs.camLeft.parm8, cs.camLeft.parm9, cs.camLeft.parm10, cs.camLeft.solid, cs.camLeft.dashed, \
                                                              cs.camRight.frame, cs.camRight.parm1, cs.camRight.parm2, cs.camRight.parm3, cs.camRight.parm4, cs.camRight.parm5, cs.camRight.parm6, cs.camRight.parm7, cs.camRight.parm8, cs.camRight.parm9, cs.camRight.parm10, cs.camRight.solid, cs.camRight.dashed, \
                                                              cs.camFarLeft.frame, cs.camFarLeft.parm1, cs.camFarLeft.parm2, cs.camFarLeft.parm3, cs.camFarLeft.parm4, cs.camFarLeft.parm5, cs.camFarLeft.parm6, cs.camFarLeft.parm7, cs.camFarLeft.parm8, cs.camFarLeft.parm9, cs.camFarLeft.parm10, cs.camFarLeft.solid, cs.camFarLeft.dashed, \
                                                              cs.camFarRight.frame, cs.camFarRight.parm1, cs.camFarRight.parm2, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm6, cs.camFarRight.parm7, cs.camFarRight.parm8, cs.camFarRight.parm9, cs.camFarRight.parm10, cs.camFarRight.solid, cs.camFarRight.dashed, receiveTime))
          elif cs.vEgo > 0:
            carStateDataString1 += (carStateDataFormatString1 % (cs.steeringAngle, cs.steeringRate, cs.steeringTorque, cs.torqueRequest, cs.steeringTorqueEps, cs.yawRateCAN, cs.lateralAccel, cs.longAccel, \
                                                              cs.lateralControlState.pidState.p2, cs.lateralControlState.pidState.p, cs.lateralControlState.pidState.i, cs.lateralControlState.pidState.f, \
                                                              cs.lateralControlState.pidState.steerAngle, cs.lateralControlState.pidState.steerAngleDes, 1.0 - cs.lateralControlState.pidState.angleFFRatio, cs.lateralControlState.pidState.angleFFRatio, cs.camLeft.frame, cs.camFarRight.frame, cs.canTime))
          #else:
          #  print(cs.canTime)

          frame_count += 1

      if socket is pathPlan:
        _pathPlan = messaging.drain_sock(socket)
        for _pp in _pathPlan:
          pp = _pp.pathPlan
          if vEgo >= 0 and not cs is None:
            format_count = 6 * len(pp.lPoly)
            pathDataString += polyDataString[:format_count] % tuple(map(float, pp.lPoly)[:15])
            pathDataString += polyDataString[:format_count] % tuple(map(float, pp.rPoly)[:15])
            pathDataString += polyDataString[:format_count] % tuple(map(float, pp.cPoly)[:15])
            pathDataString +=  (pathDataFormatString % (pp.mpcAngles[3], pp.mpcAngles[4], pp.mpcAngles[5], pp.mpcAngles[6], pp.mpcAngles[10], pp.lProb, pp.rProb, pp.cProb, pp.laneWidth, pp.angleSteers, pp.rateSteers, cs.canTime - pp.canTime, cs.canTime))
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

              kegmanDataString += ("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s|" % \
                    (reactRate, dampRate, longOffset, backlash, dampMPC, reactMPC, dampSteer, reactSteer, steerKpV, steerKiV, rateFF, cs.angleFFGain, delaySteer,
                    oscFactor, centerFactor, polyReact, polyDamp, polyScale, cs.canTime))
              insertString += kegmanFormatString + "~" + kegmanDataString + "!"
        except:
          kegman_valid = False
      insertString += pathFormatString + "~" + pathDataString + "!"
      insertString += carStateFormatString1 + "~" + carStateDataString1 + "!"
      insertString += carStateFormatString2 + "~" + carStateDataString2 + "!"

      steerPush.send_string(insertString)
      print(len(insertString))
      frame_count = 0
      kegmanDataString = ""
      pathDataString = ""
      insertString = ""
      canInsertString = ""
      carStateDataString1 = ""
      carStateDataString2 = ""

def main(rate=200):
  dashboard_thread(rate)

if __name__ == "__main__":
  main()
