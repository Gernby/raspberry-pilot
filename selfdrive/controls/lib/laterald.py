#!/usr/bin/env python
import os
import zmq
import time
import json
import numpy as np
import requests
import selfdrive.messaging as messaging
from selfdrive.kegman_conf import kegman_conf
from selfdrive.services import service_list
from cereal import log, car
from common.params import Params

INPUTS = 77
HISTORY_ROWS = 5

class Lateral(object):
  def __init__(self, CP):
    self.params = Params()
    self.use_bias = 1
    self.use_lateral_offset = 1
    self.use_angle_offset = 1
    self.carstate = messaging.pub_sock(service_list['carState'].port)
    self.cs_prev = []
    self.camera_array = []
    self.vehicle_array = []
    self.stock_cam_frame_prev = -1
    self.advanceSteer = 1
    self.frame_count = 0
    self.centerOffset = 0
    self.lastGPS = 0

  def update(self, cs, sm, can_index, can_count):
    self.frame_count += 1

    cs_send = messaging.new_message()
    cs_send.init('carState')
    cs_send.valid = cs.canValid

    if cs.camLeft.frame != self.stock_cam_frame_prev and cs.camLeft.frame == cs.camFarRight.frame:
      self.stock_cam_frame_prev = cs.camLeft.frame
      gps = sm['gpsLocationExternal']
      #sm.update(0)
      cs.gpsLocation.longitude = gps.longitude
      cs.gpsLocation.latitude = gps.latitude
      cs.gpsLocation.altitude = gps.altitude
      cs.gpsLocation.flags = gps.flags
      vNED = [float(x) for x in gps.vNED]
      if len(vNED) == 0:
        vNED = [0,0,0]
      cs.gpsLocation.vNED = vNED
      cs.gpsLocation.bearing = gps.bearing
      cs.gpsLocation.speed = gps.speed
      cs.gpsLocation.accuracy = gps.accuracy
      cs.gpsLocation.verticalAccuracy = gps.verticalAccuracy
      cs.gpsLocation.bearingAccuracy = gps.bearingAccuracy
      cs.gpsLocation.speedAccuracy = gps.speedAccuracy
      cs.gpsLocation.timestamp = gps.timestamp

      cs_send.carState = cs
      self.cs_prev.append(cs_send.to_bytes())
      self.carstate.send_multipart(self.cs_prev)
      self.cs_prev.clear()
              
    else:
      cs_send.carState = cs
      self.cs_prev.append(cs_send.to_bytes())
