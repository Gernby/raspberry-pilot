#!/usr/bin/env python
import zmq
import requests
import sys
import time

url_string = 'http://127.0.0.1:8086/write?db=carDB&u=liveOP&p=liveOP&precision=ms'
context = zmq.Context()
dash_socket = context.socket(zmq.PULL)
#tune_socket = context.socket(zmq.PULL)
dash_socket.bind("tcp://*:8593")
#tune_socket.connect("tcp://*:8595")

poller = zmq.Poller()
poller.register(dash_socket, zmq.POLLIN)
frame_count = 0
influxLineString = ""

while 1:
  try:
    socks = dict(poller.poll(500))

  except KeyboardInterrupt:
    dash_socket.close()
    context.term()
    break

  for sock in socks:
    while 1:
      thisData = None
      try:
        thisData = sock.recv_string(zmq.NOBLOCK)
      except zmq.error.Again:
        thisData = None
        break
      except KeyboardInterrupt:
        dash_socket.close()
        context.term()
        break


      if thisData is not None and sock is dash_socket:
        #print(thisData)
        try:
          strMeasurements = (thisData).split('!')
          for measurement in strMeasurements:
            #print(measurement)
            strData = (measurement).split('~')
            if len(strData) > 1:
              latControlstring = strData[0]
              #print(strData[1])
              strData = (strData[1]).split('|')
              for strFrame in strData:
                strValues = (strFrame).split(',')
                #print("insert format elements: %d, insert values: %d" % (len(latControlstring.split(',')), len(strValues)))
                if len(strValues) > 2:
                  influxLineString = influxLineString + str(latControlstring % tuple(strValues))
                  #print(str(latControlstring % tuple(strValues)))

        except:
          print("insert format elements: %d, insert values: %d" % (len((latControlstring).split(',')), len(strValues)))
          print(latControlstring, strValues)

  if len(influxLineString) > 0:
    try:
      r = requests.post(url_string, data=influxLineString)
      #print(influxLineString)
      print('%d %d %s' % (frame_count, len(influxLineString), r))
      influxLineString = ""
      frame_count = 0
    except:
      print(r)
