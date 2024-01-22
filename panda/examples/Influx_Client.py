import threading
from influxdb import InfluxDBClient

influx = InfluxDBClient('localhost', 8086, '', '', 'carDB')
influxLineString = ""
canFormatString = "CANData,user=%s,src=%d,pid=%s d1=%di,d2=%di %d\n"

class Influx_Client():
    def __init__(self, panda_serial):
        self.user_id = panda_serial
        self.t = []

    def InsertData(self, receiveTime, canData):
        try:
            def SendRequest(data=canData, influxLineString = ""):
                for d in data:
                    influxLineString += (canFormatString % (self.user_id,d[0],d[1],d[2]>>32,d[2] & 0xFFFFFFFF, receiveTime))
                headers = { 'Content-type': 'application/octet-stream', 'Accept': 'text/plain' }
                influx.request("write",'POST', {'db':'carDB'}, influxLineString.encode('utf-8'), 204, headers)
            if len(self.t) > 0: print(self.t.pop(0))
            self.t.append(threading.Thread(target=SendRequest,).run())
        except:
            pass
