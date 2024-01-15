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
        influxLineString = ""
        for cData in canData:
            influxLineString += (canFormatString % (self.user_id,cData[0],cData[1],abs(cData[2])>>32,abs(cData[2]) & 0xFFFFFFFF, receiveTime))
        if influxLineString != "":
            try:
                def SendRequest(dataString=influxLineString):
                    headers = { 'Content-type': 'application/octet-stream', 'Accept': 'text/plain' }
                    influx.request("write",'POST', {'db':'carDB'}, dataString.encode('utf-8'), 204, headers)
            
                if len(self.t) > 0: print(self.t.pop(0))
                self.t.append(threading.Thread(target=SendRequest,).run())
                self.t[0].start()
            except:
                pass
