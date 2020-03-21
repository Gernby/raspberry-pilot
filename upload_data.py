#!/usr/bin/env python
from os import path
import zmq
import time
import os
import json
import requests
import sys 
import gc
from common.params import Params
if len(sys.argv) < 2 or sys.argv[1] == 0:
  destination = "gernstation.synology.me"
  min_time = (time.time() - 72 * 60 * 60) * 1000
#elif sys.argv[2] == '1':
#  destination = "192.168.1.180"

params = Params()
user_id = params.get("DongleId")
context = zmq.Context()
steerPush = context.socket(zmq.PUSH)
steerPush.connect("tcp://" + destination + ":8594")

for cols in ["angle_offset,lateral_offset,wheel_speed_fl,wheel_speed_fr,wheel_speed_rl,wheel_speed_rr,l_blinker,r_blinker,lk_mode,enabled,left_frame,left_1,left_2,left_3,left_4,left_5,left_6,left_7,left_8,left_9,left_10,left_11,left_12,left_13,left_full_1,left_full_2,right_frame,right_1,right_2,right_3,right_4,right_5,right_6,right_7,right_8,right_9,right_10,right_11,right_12,right_13,right_full_1,right_full_2,far_left_frame,far_left_1,far_left_2,far_left_3,far_left_4,far_left_5,far_left_6,far_left_7,far_left_8,far_left_9,far_left_10,far_left_11,far_left_12,far_left_13,far_left_full_1,far_left_full_2,far_right_frame,far_right_1,far_right_2,far_right_3,far_right_4,far_right_5,far_right_6,far_right_7,far_right_8,far_right_9,far_right_10,far_right_11,far_right_12,far_right_13,far_right_full_1,far_right_full_2 from carState where (left_1 >= 0 or left_1 <= 0) and " , \
             "v_ego,angle_steers,angle_rate,driver_torque,request,angle_rate_eps,yaw_rate_can,angle_steers_eps,long_accel,p2,p,i,f,damp_angle_steers,damp_angle_steers_des,ff_rate,ff_angle,left_frame,far_right_frame from carState where"]:
  startTime = time.time()
  limit = 9999999999
  recordcount = 0
  max_limit = 5000
  min_time = 0

  while min(max_limit, limit) > 0 and recordcount % max_limit == 0:
    exec_query = '&q=select %s time > %dms fill(previous) limit %s ' % (cols, min_time, min(max_limit, limit))
    r = requests.get('http://localhost:8086/query?db=carDB&u=liveOP&p=liveOP&epoch=ms&precision=ms%s' % exec_query)

    try:
      result = json.loads(r.content)
      if len(result['results']) > 0:
        steerPush.send_multipart((user_id, r.content))
        print(len(r.content))
        limit -= max_limit
        recordcount += len(result['results'][0]['series'][0]['values'])
        min_time = result['results'][0]['series'][0]['values'][-1][0]
      else:
        break
    except:
      limit = 0
    if recordcount > 0 and recordcount % (max_limit * 5) == 0: print("records: %d  time per record: %f" % (recordcount, (time.time() - startTime) / recordcount))
