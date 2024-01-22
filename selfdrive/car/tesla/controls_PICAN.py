# This script adds the following features for Tesla Model 3 (confirmed for '23+ RWD and Performance), but it might work for all Model 3/Y and possibly some S/X:
# * Auto-reengagement of autosteer after lane change or steering override as long as speed control is still active (i.e. no brake pedal press)
#   NOTE: This gives a "single-click" autosteer engagement, but the option for single/double click engagement MUST be set to double!
# * Temporary throttle override from Chill to Standard/Sport when throttle is pressed more than 78% (NOTE: The "CHILL" message on the screen will not change)
# * Persistent throttle override from Chill to Standard/Sport when the right steering wheel scroll is swiped up and cruise control is not active (swipe down to end throttle override)
# * Satisfies the "Apply steering torque" requirement while autosteer is enabled.  The driver camera still monitors driver attentiveness, which is important!
#
# This could run on any device that has a PICAN interface with the vehicle, but has only been tested using the Raspberry-Pilot tech stack (RPi + PICAN2)
# The CAN connect used for development and testing was through this harness: https://www.gpstrackingamerica.com/shop/hrn-ct20t11/
# 
# TO-DO:
# * Enhance script for a Pi with PiCAN2 Hat, since Comma Panda's are expensive, if available at all
# * Find these CAN packets:
#   * Whether AutoSteer is "ready" for engagement.  Previously documented CAN frames are no longer valid
#   * Whether the current road has extra speed restictions (i.e. surface street vs highway)
#   * What is the current cruise control max speed
import time
import setproctitle
import os
import can
from Influx_Client import Influx_Client
from selfdrive.car.tesla.carstate import CarState

os.system("sudo /sbin/ip link set can0 down")
time.sleep(0.1)
os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
time.sleep(0.1)
bus = can.interface.Bus(channel='can0', bustype='socketcan')

setproctitle.setproctitle('GernbyMode')
CS = CarState()

logging = True
logData = []
ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,921,928,935,965,979,997]

if logging:
    ic = Influx_Client(p.get_serial()[0])

while True:
    msg = bus.recv()
    if msg.arbitration_id in [599, 280, 659, 820, 585, 1013, 297, 962, 553, 1001, 1021]:
        sendData = CS.update([(msg.timestamp, msg.arbitration_id, 0, msg.data)])

        for pid, _, cData in sendData:
            bus.send(can.Message(arbitration_id=pid, data=[d for d in cData], is_extended_id=False, dlc=len(cData)))

    if logging and pid not in ignorePIDs: 
        logData.append([msg.timestamp, 0, msg.arbitration_id, int.from_bytes(msg.data, byteorder='little', signed=False)])

        if len(logData) > 250:
            print(len(logData))
            logData.append([msg.timestamp, 0, "steer", CS.steerAngle//10])
            logData.append([msg.timestamp, 0, "APStatus", CS.lastAPStatus])
            logData.append([msg.timestamp, 0, "speed", CS.speed])
            ic.InsertData(logData)
            logData = []
