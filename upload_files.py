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
from selfdrive.kegman_conf import kegman_conf
from common.params import Params
if len(sys.argv) < 2 or sys.argv[1] == 0:
  destination = "gernstation.synology.me"
  min_time = 0  #(time.time() - 72 * 60 * 60) * 1000
elif sys.argv[1] == '1':
  min_time = 0
  destination = "192.168.1.2"
print("using %s" % destination)

kegman = kegman_conf()
do_import_local = True if kegman.conf['useLocalImport'] == "1" else False
print("useLocalImport = ", do_import_local)
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
file_list = []

upload_list = os.listdir('/data/upload/')
file_count = 0
for file in upload_list:
  filename = os.fsdecode(file) 
  if filename.endswith(".dat"): file_count += 1
print("Total files to upload: %d" % file_count)
for file in upload_list: 
  filename = os.fsdecode(file)
  if filename.endswith(".dat"): 
    #print(directory, filename)
    with open(os.path.join('/data/upload/', filename)) as myfile:
      inString = myfile.read()
      if len(inString) > 0:
        file_data.update({"file_name": filename, "file_content": inString.replace('carState', user_id)})
        file_list.append(filename)
        dataPush.send_string(json.dumps(file_data))
        print("characters sent: %d" % len(inString))
        if do_import_local:
          try:
            r = requests.post('http://localhost:8086/write?db=carDB&%sprecision=ms', data=inString)
            print(r)
          except:
            r = requests.post('http://localhost:8086/query?q=CREATE DATABASE carDB')
            #time.sleep(1)
            r = requests.post('http://localhost:8086/write?db=carDB&%sprecision=ms', data=inString)
            print(r)
        #time.sleep(1)
        if len(file_list) > 5:
          #print(dataSub.recv_string())
          reply = dataSub.recv_multipart()
          time.sleep(1)
          return_data = json.loads(reply[1])
          file_to_delete = file_list.pop(file_list.index(return_data['filename']))
          if return_data['statuscode'] == 204:
            print("successfully processed: %s  files remaining: %d" % (file_to_delete, file_count))
            #os.rename('/data/upload/%s' % file_to_delete, '/data/upload/%s' % file_to_delete.replace('.dat','.bak'))
            os.remove('/data/upload/%s' % file_to_delete)
          else:
            print(" Oops!  status_code: %s    NOT successful with file: %s" % (str(return_data['statuscode']), file_to_delete))
      else:
        os.remove('/data/upload/%s' % filename)
        print("empty file deleted:  %s" % filename)
      file_count -= 1

for i in range(len(file_list)):
  reply = dataSub.recv_multipart()
  return_data = json.loads(reply[1])
  file_to_delete = file_list.pop(file_list.index(return_data['filename']))
  if return_data['statuscode'] == 204:
    print("successfully processed: %s" % (file_to_delete))
    #os.rename('/data/upload/%s' % file_to_delete, '/data/upload/%s' % file_to_delete.replace('.dat','.bak'))
    os.remove('/data/upload/%s' % file_to_delete)
  else:
    print(" Oops!  status_code: %s    NOT successful with file: %s" % (str(return_data['statuscode']), file_to_delete))
        #time.sleep(10)
