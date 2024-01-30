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

setproctitle.setproctitle('GernbyMode')
CS = CarState()

os.system("sudo /sbin/ip link set can0 down")
time.sleep(0.1) 
os.system("sudo /sbin/ip link set can0 up type can bitrate 500000") # triple-sampling on restart-ms 10")
time.sleep(0.1)
bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False, rx_fifo_size=64, tx_fifo_size=16)
time.sleep(0.1)
can_filter = CS.can_filters[:-3]
bus.set_filters(can_filter)

loopStart = 0
logging = False
logData = []

if logging:
    ic = Influx_Client('PICAN')

while True:
    if CS.parked:  # reduce poll rate while parked
        period = 0.1
    else:
        period = 0.009
    
    sleepTime = loopStart + period - time.time()

    if sleepTime > 0:  
        time.sleep(sleepTime)  # limit polling frequency
        loopStart = loopStart + period
    else:
        loopStart = time.time()

    processData = []

    while True:
        msg = bus.recv(0)
        if msg is not None:

            if msg.arbitration_id in CS.pids:
                processData.append((msg.timestamp, msg.arbitration_id, 0, bytearray(msg.data)))

            if logging and msg.arbitration_id not in CS.ignorePIDs:
                logData.append([msg.timestamp, 0, msg.arbitration_id, int.from_bytes(msg.data, byteorder='little', signed=False)])
        else:
            break

    if len(processData) > 0:
        sendData = CS.update(processData)
        processData = []
        
        for pid, _, cData in sendData:
            bus.send(can.Message(arbitration_id=pid, data=[d for d in cData], is_extended_id=False, dlc=len(cData)))

    if logging and len(logData) > 250:
        print(len(logData))
        logData.append([processData[-1][0], 0, "steer", CS.steerAngle//10])
        logData.append([processData[-1][0], 0, "APStatus", CS.lastAPStatus])
        logData.append([processData[-1][0], 0, "speed", CS.speed])
        ic.InsertData(logData)
        logData = []
