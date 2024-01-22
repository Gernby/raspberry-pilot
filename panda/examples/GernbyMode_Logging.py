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
while True:
    try:
        sleepTime = time.time() + (0.003 if speed > 0 else 0.003) - loopStart
        if sleepTime > 0:  time.sleep(sleepTime)  # Process CAN data more than 100 times per second when stopped, and more than 200 times when moving
        loopStart, pidCount, loopCount, canData = time.time(), 0, loopCount + 1, []
        for pid, _, cData, bus in p.can_recv():
            pidCount += 1
            if pid in [599, 280, 659, 820, 585, 1013, 297, 962, 553, 1001, 1021]:
                canData.append([bus, pid, int.from_bytes(cData, byteorder='little', signed=False)])
                if pid == 599:  speed = cData[3]  # get vehicle speed
                elif pid == 280:  tempBalls = cData[4] > 200  # override to standard / sport if throttle is above 78%
                elif pid == 659:  p659, autoSteer = cData, cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
                elif pid == 820:  p820 = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
                elif pid == 585 and cData[2] & 15 > leftStalkStatus:  leftStalkStatus = cData[2] & 15  # Get left stalk status, and bump the status up for full click vs half click
                elif pid == 1013 and cData[5] > 0:  turnSignal, prevTurnSignal, nextClick = True, enabled, max(nextClick, loopStart + (0.5 if leftStalkStatus in (4,8) else 4)) # Delay spoof if turn signal is on
                elif pid == 297:  # get steering angle
                    cData[3] &= 63  # mask upper bits
                    steerAngle = struct.unpack("<1h",cData[2:4])[0] - 8192  # decode angle using multiple / partial bytes
                elif pid == 962 and cData[3] not in [0,85]: # Get right steering wheel up / down swipe
                    if cData[3] <= 64 and cData[3] >= 44 and enabled:  nextClick = max(nextClick, loopStart + 5)  # ensure enough time for the cruise speed decrease to be applied
                    elif cData[3] < 20 and cData[3] > 1 and not enabled:  moreBalls = True  # up swipe will lock standard / sport throttle override mode
                    elif cData[3] < 64 and cData[3] > 44 and not enabled:  moreBalls = False  # down swipe will end throttle override mode
                elif pid == 553 and enabled and lanesVisible and cData[1] <= 15 and loopStart > nextClick and (lastAPStatus == 33 or abs(steerAngle) < 50):
                    cData[0], cData[1] = p553[cData[1]], (cData[1] + 1) % 16 + 48
                    _, nextClick, prevTurnSignal, leftStalkStatus = p.can_send(553, cData, bus), max(nextClick, loopStart + 0.5), 0, 0  # It's time to spoof or reengage autosteer
                elif pid == 1001:  # get AutoPilot state and check for driver override
                    if lastAPStatus == 33 and cData[3] & 33 == 32 and not (turnSignal or prevTurnSignal):  driverOverride = True  # prevent auto reengage of autosteer
                    elif cData[3] & 33 != 32:  driverOverride, prevTurnSignal = False, turnSignal  # if speed control is disabled or autosteer is enabled, reset the override and signal history
                    lastAPStatus = cData[3] & 33
                    if cData[3] & 33 > 0 and speed > 0 and autoSteer and lanesVisible and not enabled and not driverOverride:
                        enabled = cData[2] & 1  # AP is active
                    elif (cData[3] & 33 == 0 or cData[2] & 1 == 0 or speed == 0 or driverOverride) and enabled == 1:
                        enabled = 0  # AP is not active
                        if cData[3] & 33 == 0 or cData[2] & 1 == 0 or driverOverride:  nextClick = max(nextClick, loopStart + 0.5)  # if the car isn't moving or AP isn't engaged, then delay the click
                elif pid == 1021 and cData[0] == 0 and cData[6] & 1 == 0:  print("Double-Pull AP engagement is required for Auto-Reengage to work")
            elif not pid in ignorePIDs: canData.append([bus, pid, int.from_bytes(cData, byteorder='little', signed=False)])
        if (moreBalls or tempBalls) and p820[0] & 32 == 0 and p659[5] & 16 == 0:  # initialize throttle override mode to Standard / Sport
                p820[0], p820[6], p820[7], p659[5], p659[6], p659[7] = (p820[0] + 32) % 256, (p820[6] + 16) % 256, (p820[7] + 48) % 256, (p659[5] + 16) % 256, (p659[6] + 16) % 256, (p659[7] + 32) % 256
                _, _, p820[0], p659[0] = p.can_send(820, p820, bus), p.can_send(659, p659, bus), 32, 16  # send packets and prevent another throttle override before next update from controller
        if pidCount > 0:
            canData.append([0,"steer",steerAngle//10])
            canData.append([0,"APStatus",lastAPStatus])
            canData.append([0,"speed",speed])
            ic.InsertData(int(loopStart * 1000000000), canData)
    except Exception as e:  # initialize everything when an exception happens
        import time, panda, setproctitle, struct
        from Influx_Client import Influx_Client
        _, p, _ = time.sleep(1), panda.Panda(), setproctitle.setproctitle('GernbyMode')
        _, _, _, _, ic = p.set_can_speed_kbps(0,500), p.set_can_speed_kbps(1,500), p.set_can_speed_kbps(2,500), p.set_safety_mode(panda.Panda.SAFETY_ALLOUTPUT), Influx_Client(p.get_serial()[0])
        lanesVisible, enabled, driverOverride, autoSteer, nextClick, speed, turnSignal, prevTurnSignal, leftStalkStatus, moreBalls, tempBalls, loopCount, steerAngle = 1,0,0,0,0,0,0,0,0,0,0,0,0
        canData , loopCount, lastAPStatus, lastCANSend, signalSteerAngle, loopStart, p820, p659, p553 = {},0,0,0,0,time.time(),[],[],[75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,921,928,935,965,979,997]

