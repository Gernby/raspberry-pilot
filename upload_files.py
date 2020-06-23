#!/usr/bin/env python
from os import path
from datetime import datetime
import zmq
import time
import os
import json
import numpy as np
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
identifier = np.random.randint(0, high=10000)
context = zmq.Context()
dataPush = context.socket(zmq.PUSH)
dataPush.connect("tcp://" + destination + ":8593")
start_time = time.time() * 1000 #- 24 * 60 * 60 * 1000
dataSub = context.socket(zmq.SUB)
dataSub.connect("tcp://" + destination + ":8602")
dataSub.setsockopt_string(zmq.SUBSCRIBE, str(identifier))   #, user_id)

next_time = time.time()
limit = 7000000
recordcount = 0
max_limit = 5000

directory = os.fsencode('/data/upload')
file_data = {"user_id": str(params.get("PandaDongleId")), "file_name": "", "file_content": "", "identifier": identifier}

for file in os.listdir('/data/upload/'):
  filename = os.fsdecode(file)
  if filename.endswith(".dat"): 
    #print(directory, filename)
    with open(os.path.join('/data/upload/', filename)) as myfile:
      inString = myfile.read()
      print(len(inString))
      if len(inString) > 0:
        file_data.update({"file_name": filename, "file_content": inString})
        dataPush.send_string(json.dumps(file_data))
        reply = dataSub.recv_string()
        print(reply)
        #time.sleep(10)
