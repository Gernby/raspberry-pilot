#!/usr/bin/env python
import os
if False:
  model_name = 'GRU_Split_Lane_Steer_Angle_0_Lag_1_Smooth_64_Batch_65_5_15_5_Hist_100_Future_0_0_0_Drop_Final'
else:
  os.environ["CUDA_VISIBLE_DEVICES"]="-1"
  model_name = 'GRU_Split_Lane_Steer_Angle_0_Lag_1_Smooth_64_Batch_65_5_15_5_Hist_200_Future_0_0_0_Drop_Final_CPU'
history_rows = 5
inputs = 59

import zmq
import time
import json
import platform
import subprocess
import multiprocessing
from tensorflow.python.keras.models import load_model #, Model  #, Sequential
import numpy as np
import joblib
from selfdrive.services import service_list
from enum import Enum
from cffi import FFI
from setproctitle import setproctitle

ffi = FFI()
ffi.cdef("long syscall(long number, ...);")
libc = ffi.dlopen(None)

def set_realtime_priority(level):
  if platform.machine() == "x86_64":
    NR_gettid = 186
  elif platform.machine() == "aarch64":
    NR_gettid = 178
  else:
    raise NotImplementedError
  tid = libc.syscall(NR_gettid)
  print("/n/n realtime priority = %d  %s  %s/n" %(level, NR_gettid, str(tid)))
  return subprocess.call(['chrt', '-f', '-p', str(level), str(tid)])

def drain_sock(sock, wait_for_one=False):
  ret = []
  if wait_for_one:
    ret.append(sock.recv())
  while 1:
    try:
      ret.append(sock.recv(zmq.NOBLOCK))
    except zmq.error.Again:
      break
  return ret

def dashboard_thread(rate=100):

  setproctitle('transcoderd')
  set_realtime_priority(1)
  ipaddress = "tcp://127.0.0.1"

  context = zmq.Context.instance()
  gernModelInputs = context.socket(zmq.SUB)
  gernModelInputs.connect("tcp://127.0.0.1:%d" % service_list['model'].port)
  gernModelInputs.setsockopt(zmq.SUBSCRIBE, b"")

  gernModelOutputs = context.socket(zmq.PUB)
  gernModelOutputs.bind("tcp://*:8605")

  model = load_model(os.path.expanduser('./models/' + model_name + '.hdf5'))
  model_input = np.zeros((history_rows, inputs))
  model.predict_on_batch([[model_input[:,:8]], [model_input[:,8:11]], [model_input[:,-48:-24]], [model_input[:,-24:]]])
  frame = 0

  drain_sock(gernModelInputs, True)

  while 1:
    model_input_array = [gernModelInputs.recv()]

    if len(model_input_array) > 1:
      print("                         lagging lateral by %d" % len(model_input_array))

    for dat in model_input_array[-1:]:
      input_list = json.loads(dat)
      model_input = np.array(input_list[:-1]).reshape(history_rows, inputs)
      #print(model_input.shape)

      all_inputs = [[model_input[:,:-48-3]], [model_input[:,-48-3:-48]], [model_input[:,-48:-24]], [model_input[:,-24:]]]
      #print(model_output[0])

      model_output = list(model.predict_on_batch(all_inputs)[0].astype('float'))
      model_output.append(input_list[-1])
      gernModelOutputs.send_json(model_output)
      if frame % 30 == 0:
        print(frame, time.time())
      frame += 1

def main(rate=200):
  dashboard_thread(rate)

if __name__ == "__main__":
  main()
