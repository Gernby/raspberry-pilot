# This script adds the following features for Tesla Model 3 (confirmed for '23+ RWD and Performance), but it might work for all Model 3/Y and possibly some S/X:
# * Auto-reengagement of autosteer after lane change or steering override as long as speed control is still active (i.e. no brake pedal press)
#   NOTE: This gives a "single-click" autosteer engagement, but the option for single/double click engagement MUST be set to double!
# * Temporary throttle override from Chill to Standard/Sport when throttle is pressed more than 78% (NOTE: The "CHILL" message on the screen will not change)
# * Persistent throttle override from Chill to Standard/Sport when the right steering wheel scroll is swiped up and cruise control is not active (swipe down to end throttle override)
# * Satisfies the "Apply steering torque" requirement while autosteer is enabled.  The driver camera still monitors driver attentiveness, which is important!
#
# This could run on any device that has a PandaCAN interface with the vehicle, but has only been tested using the Raspberry-Pilot tech stack (RPi4 + White Comma Panda with Raspberry-Pilot firmware)
# The CAN interface used for development and testing was a White Comma Panda running the Raspberry-Pilot firmware and this harness: https://www.gpstrackingamerica.com/shop/hrn-ct20t11/
# If running on an RPi with Comma Panda, the RPi needs to be configured for USB OTG, which requires raspberry-pilot/phonelibs/usercfg.txt to be copied into /boot/firmware
# The CPU utilization on an RPi4 is so low that the clock speed should be reduced for power savings, especially while stopped / idle.  I have mine set to 600 MHz.
# 
# TO-DO:
# * Enhance script for a Pi with PiCAN2 Hat, since Comma Panda's are expensive, if available at all
# * Find these CAN packets:
#   * Whether AutoSteer is "ready" for engagement.  Previously documented CAN frames are no longer valid
#   * Whether the current road has extra speed restictions (i.e. surface street vs highway)
#   * What is the current cruise control max speed
import time
import panda
import setproctitle
from Influx_Client import Influx_Client
from selfdrive.car.tesla.carstate import CarState

setproctitle.setproctitle('GernbyMode')
CS = CarState()

p = panda.Panda()
p.set_can_speed_kbps(0,500)
p.set_can_speed_kbps(1,500)
p.set_can_speed_kbps(2,500)
p.set_safety_mode(panda.Panda.SAFETY_ALLOUTPUT)

loopStart = 0
logging = False
logData = []

if logging:
    ic = Influx_Client(p.get_serial()[0])

while True:
    if CS.parked:  # reduce poll rate while parked
        period = 0.1
    else:
        period = 0.009 if not logging else 0.001
    
    sleepTime = loopStart + period - time.time()

    if sleepTime > 0:  
        time.sleep(sleepTime)  # limit polling frequency
        loopStart = loopStart + period
    else:
        loopStart = time.time()

    processData = []

    for pid, _, cData, bus in p.can_recv():

        if pid in CS.pids:
            processData.append((loopStart, pid, bus, cData))

        if logging and pid not in CS.ignorePIDs: 
            logData.append([loopStart, bus, pid, int.from_bytes(cData, byteorder='little', signed=False)])

    if len(processData) > 0:
        sendData = CS.update(processData)

        for pid, bus, cData in sendData:
            p.can_send(pid, cData, bus)

    if logging and len(logData) > 250:
        logData.append([loopStart, 0, "steer", abs(CS.steerAngle//10)])
        logData.append([loopStart, 0, "APStatus", CS.lastAPStatus])
        logData.append([loopStart, 0, "speed", CS.speed])
        ic.InsertData(logData)
        logData = []
