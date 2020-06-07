#!/usr/bin/env python
import os
import zmq
import time
import json
import numpy as np
import joblib
import requests

from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from cereal import log, car
from common.params import Params

BIT_MASK = [1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8, 
            1, 128, 64, 32, 8, 4, 2, 8]

INPUTS = 73
OUTPUTS = 9
MODEL_VERSION = '042'
HISTORY_ROWS = 5

def pub_sock(port, addr="*"):
  context = zmq.Context.instance()
  sock = context.socket(zmq.PUB)
  sock.bind("tcp://%s:%d" % (addr, port))
  return sock

class Lateral(object):
  def __init__(self):
    self.params = Params()
    self.use_bias = 1
    self.use_lateral_offset = 1
    self.use_angle_offset = 1
    self.gernModelInputs = pub_sock(service_list['model'].port)
    self.camera_array = []
    self.vehicle_array = []
    self.stock_cam_frame_prev = -1
    self.advanceSteer = 1
    self.frame_count = 0
    self.centerOffset = 0
    '''self.inner_angles = [0., 0.]

    lateral_params = self.params.get("LateralParams")
    try:
      lateral_params = json.loads(lateral_params)
    except:
      lateral_params = {'angle_bias': 0, 'angle_offset': 0, 'lateral_offset': 0.}

    if "angle_bias" in lateral_params:
      self.angle_bias = float(lateral_params['angle_bias'])
    else:
      self.angle_bias = 0.
    if "angle_offset" in lateral_params:
      self.angle_offset = float(lateral_params['angle_offset'])
    else:
      self.angle_offset = 0.
    if "lateral_offset" in lateral_params:
      self.lateral_offset = float(lateral_params['lateral_offset'])
    else:
      self.lateral_offset = 0.
    self.angle_factor = 1.0'''

  def update(self, cs, path_plan):
    self.frame_count += 1

    '''try:
      if cs.vEgo > 10 and abs(cs.steeringRate) < 5 :
        if cs.yawRateCAN > 0 != cs.lateralAccel < 0:
          if cs.steeringAngle + self.angle_offset < 0 and path_plan.cPoly[5] > 0:
            self.angle_offset += (0.0001 * cs.vEgo)
          elif cs.steeringAngle + self.angle_offset > 0 and path_plan.cPoly[5] < 0:
            self.angle_offset -= (0.0001 * cs.vEgo)
    except:
      pass'''

    left_missing = 1 if cs.camLeft.parm4 == 0 else 0
    far_left_missing = 1 if cs.camFarLeft.parm4 == 0 else 0
    right_missing = 1 if cs.camRight.parm4 == 0 else 0
    far_right_missing = 1 if cs.camFarRight.parm4 == 0 else 0
    
    camera_flags = np.bitwise_and([left_missing, cs.camLeft.parm6, cs.camLeft.parm6, cs.camLeft.parm6, cs.camLeft.parm6, cs.camLeft.parm6, cs.camLeft.parm6, cs.camLeft.parm8, 
                                   far_left_missing, cs.camFarLeft.parm6, cs.camFarLeft.parm6, cs.camFarLeft.parm6, cs.camFarLeft.parm6, cs.camFarLeft.parm6, cs.camFarLeft.parm6, cs.camFarLeft.parm8, 
                                   right_missing, cs.camRight.parm6, cs.camRight.parm6, cs.camRight.parm6, cs.camRight.parm6, cs.camRight.parm6, cs.camRight.parm6, cs.camRight.parm8, 
                                   far_right_missing, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm8], BIT_MASK)

    camera_flags = np.minimum(1, camera_flags)

    left_10 = cs.camLeft.parm10 if cs.camLeft.parm10 >= 0 else cs.camLeft.parm10 + 128
    far_left_10 = cs.camFarLeft.parm10 if cs.camFarLeft.parm10 >= 0 else cs.camFarLeft.parm10 + 128
    right_10 = cs.camRight.parm10 if cs.camRight.parm10 <= 0 else cs.camRight.parm10 - 128
    far_right_10 = cs.camFarRight.parm10 if cs.camFarRight.parm10 <= 0 else cs.camFarRight.parm10 - 128

    self.vehicle_array.append([cs.vEgo, cs.steeringAngle, cs.lateralAccel, cs.steeringTorqueEps, cs.yawRateCAN, cs.longAccel,  max(600, path_plan.laneWidth), 0 , 0])

    if cs.camLeft.frame != self.stock_cam_frame_prev and cs.camLeft.frame == cs.camFarLeft.frame:

      self.camera_array.append(np.concatenate((camera_flags, 
                                                [far_left_10,  max(0, cs.camFarLeft.parm2),  cs.camFarLeft.parm1,  cs.camFarLeft.parm3,  cs.camFarLeft.parm4,  cs.camFarLeft.parm5,  cs.camFarLeft.parm7,  cs.camFarLeft.parm9], 
                                                [far_right_10, min(0, cs.camFarRight.parm2), cs.camFarRight.parm1, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm7, cs.camFarRight.parm9],
                                                [left_10,       max(0, cs.camLeft.parm2),     cs.camLeft.parm1,     cs.camLeft.parm3,     cs.camLeft.parm4,     cs.camLeft.parm5,     cs.camLeft.parm7,     cs.camLeft.parm9],    
                                                [right_10,     min(0, cs.camRight.parm2),    cs.camRight.parm1,    cs.camRight.parm3,    cs.camRight.parm4,    cs.camRight.parm5,    cs.camRight.parm7,    cs.camRight.parm9]),axis=0))
      if len(self.camera_array) > HISTORY_ROWS:
        self.stock_cam_frame_prev = cs.camLeft.frame
        cs.modelData = [float(x) for x in list(np.asarray(np.concatenate((self.vehicle_array[-HISTORY_ROWS:], self.camera_array[-HISTORY_ROWS:]), axis = 1)).reshape(HISTORY_ROWS * INPUTS))]
        self.gernModelInputs.send(cs.to_bytes())
        self.camera_array.pop(0)
        self.vehicle_array.pop(0)

    '''# TODO: replace kegman_conf with params!
    if self.frame_count % 100 == 0:
    #try:
      kegman = kegman_conf()  
      self.advanceSteer = max(0, float(kegman.conf['advanceSteer']))
      self.angle_factor = float(kegman.conf['angleFactor'])

    if self.frame_count % 1000 == 2:
      self.params.put("LateralParams", json.dumps({'angle_offset': self.angle_offset, 'angle_bias': self.angle_bias, 'lateral_offset': self.lateral_offset}))'''
