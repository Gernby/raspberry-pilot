# This script adds the following features for Tesla Model 3 (confirmed for '23+ RWD and Performance), but it might work for all Model 3/Y and possibly some S/X:
# * Auto-reengagement of autosteer after lane change or steering override as long as speed control is still active (i.e. no brake pedal press)
#   NOTE: This gives a "single-click" autosteer engagement, but the option for single/double click engagement MUST be set to double!
# * Temporary throttle override from Chill to Standard/Sport when throttle is pressed more than 78% (NOTE: The "CHILL" message on the screen will not change)
# * Persistent throttle override from Chill to Standard/Sport when the right steering wheel scroll is swiped up and cruise control is not active (swipe down to end throttle override)
# * Satisfies the "Apply steering torque" requirement while autosteer is enabled.  The driver camera still monitors driver attentiveness, which is important!
# * For Dual-CAN mode: After first AP engagement (until AP is cancelled via right stalk up-click), AP will auto-engage anytime AP is "Ready"
#
# This could run on any device that has a PICAN interface with the vehicle, but has only been tested using the Raspberry-Pilot tech stack (RPi + PICAN2)
# The CAN connect used for development and testing was through these harnesses: 
#       Dual-CAN operation (full features):      https://evoffer.com/product/model-3-y-can-diagnostic-cable/
#       Single-CAN operation (limited features): https://www.gpstrackingamerica.com/shop/hrn-ct20t11/
# 
# TO-DO:
# * Find these CAN packets:
#   * Whether the current road has extra speed restictions (i.e. surface street vs highway)
#   * What is the current cruise control max speed
#
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
for channel in CS.Update[:-1]:
    for pid in channel:
        filterCAN.append({"can_id": pid, "can_mask": pid, "extended": False})
bus.set_filters(filterCAN)

loopStart = 0
logging = False
logData = []
sendCAN = None
research = False

if logging:
    from Influx_Client import Influx_Client
    IC = Influx_Client('PICAN')

while True:
    msg = bus.recv()

    if msg.arbitration_id in CS.Update[int(msg.channel[-1:])]:
        sendCAN = CS.Update[int(msg.channel[-1:])][msg.arbitration_id](msg.timestamp, msg.arbitration_id, int(msg.channel[-1]), bytearray(msg.data))

        if sendCAN:
            for pid, _, cData in sendCAN:
                bus.send(can.Message(arbitration_id=pid, data=[d for d in cData], is_extended_id=False, dlc=len(cData)))
            sendCAN = None
    elif research:
        CS.Update[-1](msg.timestamp, msg.arbitration_id, int(msg.channel[-1]), bytearray(msg.data))

    if logging and msg.arbitration_id not in CS.ignorePIDs:
        logData.append([msg.timestamp, 0, msg.arbitration_id, int.from_bytes(msg.data, byteorder='little', signed=False)])

        if len(logData) > 250:
            IC.InsertData(CS, logData)
            logData = []
