#!/usr/bin/env python
from os import path
from datetime import datetime
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
  min_time = 0  #(time.time() - 72 * 60 * 60) * 1000
elif sys.argv[1] == '1':
  min_time = 0
  destination = "192.168.1.3"

params = Params()
user_id =  str(params.get("PandaDongleId"))
user_id = user_id.replace("'","")
context = zmq.Context()
steerPush = context.socket(zmq.PUSH)
steerPush.connect("tcp://" + destination + ":8594")
start_time = time.time() * 1000 #- 24 * 60 * 60 * 1000
min_time = start_time 

for days in range(30):
  for hours in range(24):
    print("querying from {0:%m-%d-%Y %H:%M}".format(datetime.fromtimestamp(min_time/1000)))
    for cols in ["v_ego,angle_steers,angle_rate,driver_torque,request,angle_rate_eps,yaw_rate_can,angle_steers_eps,long_accel,p2,p,i,f,damp_angle_steers,damp_angle_steers_des,ff_rate,ff_angle,left_frame,far_right_frame from carState where ", 
                "wheel_speed_fl,wheel_speed_fr,wheel_speed_rl,wheel_speed_rr,l_blinker,r_blinker,lk_mode,enabled,left_1,left_2,left_3,left_4,left_5,left_6,left_7,left_8,left_9,left_10,left_11,left_12,left_13,left_full_1,left_full_2,right_frame,right_1,right_2,right_3,right_4,right_5,right_6,right_7,right_8,right_9,right_10,right_11,right_12,right_13,right_full_1,right_full_2,far_left_frame,far_left_1,far_left_2,far_left_3,far_left_4,far_left_5,far_left_6,far_left_7,far_left_8,far_left_9,far_left_10,far_left_11,far_left_12,far_left_13,far_left_full_1,far_left_full_2,far_right_1,far_right_2,far_right_3,far_right_4,far_right_5,far_right_6,far_right_7,far_right_8,far_right_9,far_right_10,far_right_11,far_right_12,far_right_13,far_right_full_1,far_right_full_2 from carState where "]:
      next_time = time.time()
      limit = 7000000
      recordcount = 0
      max_limit = 5000
      min_time = start_time - (days * 24 * 60 * 60 * 1000) - (hours * 60 * 60 * 1000)
      max_time = min_time + 60 * 60 * 1000

      while min(max_limit, limit) > 0 and recordcount % max_limit == 0:
        qry_start = time.time()

        exec_query = '&q=select %s time > %dms and time < %dms limit %s ' % (cols, min_time, max_time, min(max_limit, limit))
        r = requests.get('http://localhost:8086/query?db=carDB&u=liveOP&p=liveOP&epoch=ms&precision=ms%s' % exec_query)

        send_start = time.time()

        try:
          result = json.loads(r.content)
          result.update({"user_id": user_id})
          steerPush.send_string(json.dumps(result))

          send_end = time.time()

          sleep_time = max(send_end - send_start, send_start - qry_start) - min(send_end - send_start, send_start - qry_start)
          time.sleep(sleep_time)
          print( send_start - qry_start, send_end - send_start, sleep_time)
          if len(result['results']) > 0:
            print(len(r.content))
            limit -= max_limit
            recordcount += len(result['results'][0]['series'][0]['values'])
            min_time = result['results'][0]['series'][0]['values'][-1][0]
          else:
            print('Done uploading %d records' % recordcount)
            break
        except:
          print('Done uploading %d records' % recordcount)
          limit = 0
        if recordcount > 0 and recordcount % (max_limit * 5) == 0: 
          print("records: %d  " % recordcount)