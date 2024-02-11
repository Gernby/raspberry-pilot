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
from carstate import CarState

setproctitle.setproctitle('GernbyMode')
CS = CarState()

os.system("sudo /sbin/ip link set can0 down")
time.sleep(0.1) 
os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
time.sleep(0.1)
bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)
time.sleep(0.1)

filterCAN = []
for pid in CS.Update:
    filterCAN.append({"can_id": pid, "can_mask": pid, "extended": False})
bus.set_filters(filterCAN)

loopStart = 0
logging = False
logData = []
sendCAN = None

if logging:
    from Influx_Client import Influx_Client
    ic = Influx_Client('PICAN')

while True:
    msg = bus.recv()

    if msg.arbitration_id in CS.Update:
        sendCAN = CS.Update[msg.arbitration_id](msg.timestamp, msg.arbitration_id, msg.channel, bytearray(msg.data))

    if not sendCAN is None and len(sendCAN) > 0:
        for pid, _, cData in sendCAN:
            bus.send(can.Message(arbitration_id=pid, data=[d for d in cData], is_extended_id=False, dlc=len(cData)))
        sendCAN = None

    if logging and msg.arbitration_id not in CS.ignorePIDs:
        logData.append([msg.timestamp, 0, msg.arbitration_id, int.from_bytes(msg.data, byteorder='little', signed=False)])

        if len(logData) > 250:
            ic.InsertData(CS, logData)
            logData = []
