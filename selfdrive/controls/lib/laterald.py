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

INPUTS = 77
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

  def update(self, cs, path_plan):
    self.frame_count += 1

    self.vehicle_array.append([cs.vEgo, cs.steeringAngle, cs.lateralAccel, cs.steeringTorqueEps, cs.yawRateCAN])

    if cs.camLeft.frame != self.stock_cam_frame_prev and cs.camLeft.frame == cs.camFarLeft.frame:

      left_missing = 1 if cs.camLeft.parm4 == 0 else 0
      far_left_missing = 1 if cs.camFarLeft.parm4 == 0 else 0
      right_missing = 1 if cs.camRight.parm4 == 0 else 0
      far_right_missing = 1 if cs.camFarRight.parm4 == 0 else 0
      
      camera_flags = np.bitwise_and([left_missing,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm6,     cs.camLeft.parm8, 
                                    far_left_missing,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm6,  cs.camFarLeft.parm8, 
                                    right_missing,     cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm6,    cs.camRight.parm8, 
                                    far_right_missing, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm6, cs.camFarRight.parm8], BIT_MASK)

      self.camera_array.append(np.concatenate(([cs.vEgo, cs.longAccel,  max(570, path_plan.laneWidth), cs.steeringAngle, cs.lateralAccel, cs.yawRateCAN, 0, 0], np.minimum(1, camera_flags), 
                                                [cs.camFarLeft.parm10,  cs.camFarLeft.parm2,  cs.camFarLeft.parm1,  cs.camFarLeft.parm3,  cs.camFarLeft.parm4,  cs.camFarLeft.parm5,  cs.camFarLeft.parm7,  cs.camFarLeft.parm9], 
                                                [cs.camFarRight.parm10, cs.camFarRight.parm2, cs.camFarRight.parm1, cs.camFarRight.parm3, cs.camFarRight.parm4, cs.camFarRight.parm5, cs.camFarRight.parm7, cs.camFarRight.parm9],
                                                [cs.camLeft.parm10,     cs.camLeft.parm2,     cs.camLeft.parm1,     cs.camLeft.parm3,     cs.camLeft.parm4,     cs.camLeft.parm5,     cs.camLeft.parm7,     cs.camLeft.parm9],    
                                                [cs.camRight.parm10,    cs.camRight.parm2,    cs.camRight.parm1,    cs.camRight.parm3,    cs.camRight.parm4,    cs.camRight.parm5,    cs.camRight.parm7,    cs.camRight.parm9]),axis=0))

      if len(self.camera_array) > HISTORY_ROWS:
        self.stock_cam_frame_prev = cs.camLeft.frame
        cs.modelData = [float(x) for x in list(np.asarray(np.concatenate((self.vehicle_array[-HISTORY_ROWS:], self.camera_array[-HISTORY_ROWS:]), axis = 1)).reshape(HISTORY_ROWS * INPUTS))]
        self.gernModelInputs.send(cs.to_bytes())
        self.camera_array.pop(0)
        self.vehicle_array.pop(0)
