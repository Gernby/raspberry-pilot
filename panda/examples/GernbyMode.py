import time, panda, setproctitle
setproctitle.setproctitle('GernbyMode')
enabled, available, nextClick, speed, turnSignal, prevTurnSignal, leftStalkStatus, moreBalls, tempBalls, loopCount, prevLoopStart = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time.time()
next553 = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
while True:
    try:
        time.sleep(time.time() + (0.0033 if speed > 0 else 0.0067) - prevLoopStart)  # Process CAN data more than 100 times per second when stopped, and more than 200 times when moving
        #print(loopCount)
        prevLoopStart, loopCount = time.time(), 0
        for pid, _, cData, _ in p.can_recv():
            #loopCount += 1  # count CAN messages per loop for performance tuning
            if pid == 553:  # consider sending single right stalk click
                if enabled and cData[1] <= 15 and prevLoopStart > nextClick:
                    cData[0], cData[1] = next553[cData[1]], (cData[1] + 1) % 16 + 48
                    p.can_send(553, cData, 0)
                    if prevTurnSignal:  nextClick, prevTurnSignal, leftStalkStatus = prevLoopStart + 0.2, 0, 0  # turn signal just ended, so reengage autosteer by sending another stalk click sooner
                    else:  nextClick = prevLoopStart + 1.2  # send right stalk click again later to satisfy steering nag
                elif cData[1] > 15:  nextClick = max(nextClick, prevLoopStart + 0.25)  # delay next click if right stalk is currently being clicked
            elif pid == 659 and cData[5] & 16 == 0:  # get / set throttle mode
                if moreBalls or tempBalls:
                    cData[5], cData[6], cData[7] = (cData[5] + 16) % 256, (cData[6] + 16) % 256, (cData[7] + 32) % 256
                    p.can_send(659, cData, 0)
                else:  last659 = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            elif pid == 820 and cData[0] & 32 == 0:  # get / set throttle mode
                if moreBalls or tempBalls:
                    cData[0], cData[6], cData[7] = (cData[0] + 32) % 256, (cData[6] + 16) % 256, (cData[7] + 48) % 256
                    p.can_send(820, cData, 0)
                else:  last820 = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            elif pid == 1001:  # get AutoPilot state
                available = cData[2] & 1   # not sure if this matters, but it might reduce premature engagement warning chimes
                if cData[3] > 0 and enabled == 0 and speed > 0:
                    enabled = 1 & available  # AP is active
                elif (cData[3] == 0 or speed == 0) and enabled == 1:
                    enabled = 0  # AP is not active
                    if cData[3] == 0:  nextClick = prevLoopStart + 0.2  # if the car isn't moving or AP isn't engaged, then delay the click
            elif pid == 1013:  # get turn signal status
                turnSignal = cData[5] > 0
                if turnSignal:  # delay stalk click until turn signal ends
                    nextClick = prevLoopStart + (0.2 if leftStalkStatus in (4,8) else 4)  # half signal click requires a longer delay before reengagement
                    prevTurnSignal = True
            elif pid == 599:  speed = cData[3]  # get vehicle speed
            elif pid == 280:  tempBalls = cData[4] > 200  # override to standard / sport if throttle is above 78%
            elif pid == 585 and cData[2] > leftStalkStatus:  leftStalkStatus = cData[2]  # Get left stalk status, and bump the status up for full click vs half click
            elif pid == 962 and cData[3] not in [0,85] and not enabled: # Get right steering wheel up / down swipe
                if cData[3] < 20 and cData[3] > 1:  moreBalls = True  # up swipe will lock standard / sport throttle override mode
                elif cData[3] < 64 and cData[3] > 44:  moreBalls = False  # down swipe will end throttle override mode
        if (moreBalls or tempBalls) and last820[0] & 32 == 0 and last659[5] & 16 == 0:  # initialize throttle override mode to Standard / Sport
                last820[0], last820[6], last820[7] = (last820[0] + 32) % 256, (last820[6] + 16) % 256, (last820[7] + 48) % 256
                p.can_send(820, last820, 0)
                last659[5], last659[6], last659[7] = (last659[5] + 16) % 256, (last659[6] + 16) % 256, (last659[7] + 32) % 256
                p.can_send(659, last659, 0)
                last820[0], last659 = 32, 16  # prevent another override packet before next update from controller
    except Exception as e:
        time.sleep(1)
        p = panda.Panda()
        p.set_can_speed_kbps(0,500)
        p.set_safety_mode(panda.Panda.SAFETY_ALLOUTPUT)
