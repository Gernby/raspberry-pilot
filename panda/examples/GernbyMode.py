import time, panda, setproctitle
setproctitle.setproctitle('GernbyMode')
enabled, nextClick, speed, turnSignal, prevTurnSignal, leftStalkStatus, moreBalls, tempBalls = 0, 0, 0, 0, 0, 0, 0, 0
next553 = {'70,0':  [75,49], '68,1':  [93,50], '82,2':  [98,51], '109,3': [76,52], '67,4':  [78,53], '65,5':  [210,54], '221,6': [246,55], '249,7': [67,56], '76,8':  [170,57], '165,9': [249,58], '246,10':[131,59], '140,11':[70,60], '73,12': [32,61], '47,13': [62,62], '49,14': [52,63], '59,15': [73,48]}
while True:
    try:
        time.sleep(0.0025)
        for pid, _, cData, _ in p.can_recv():
            if pid == 553:  # send single right stalk click
                if enabled and time.time() > nextClick and "%d,%d" % (cData[0],cData[1]) in next553:
                    if prevTurnSignal:  # turn signal just ended, so reengage AP
                        nextClick = time.time() + 0.2
                        prevTurnSignal, leftStalkStatus = 0, 0
                    else:
                        nextClick = time.time() + 2
                    b = next553["%d,%d" % (cData[0],cData[1])]
                    cData[0], cData[1] = b[0], b[1]
                    p.can_send(553, cData, 0)
                elif not enabled and cData[1] > 15:  # delay next click if right stalk is currently being clicked
                    nextClick = time.time() + 0.25
            elif pid == 659 and cData[5] & 16 == 0:  # get / set throttle mode
                if moreBalls or tempBalls:
                    cData[5], cData[6], cData[7] = (cData[5] + 16) % 256, (cData[6] + 16) % 256, (cData[7] + 32) % 256
                    p.can_send(659, cData, 0)
                else:
                    last659 = cData
            elif pid == 820 and cData[0] & 32 == 0:  # get / set throttle mode
                if moreBalls or tempBalls:
                    cData[0], cData[6], cData[7] = (cData[0] + 32) % 256, (cData[6] + 16) % 256, (cData[7] + 48) % 256
                    p.can_send(820, cData, 0)
                else:
                    last820 = cData                
            elif pid == 1001:  # get AutoPilot state
                if cData[3] > 0 and enabled == 0 and speed > 0:
                    enabled = 1  # AP is active
                elif (cData[3] == 0 or speed == 0) and enabled == 1:
                    enabled = 0  # AP is not active
                    if cData[3] == 0: 
                        nextClick = time.time() + 0.2
            elif pid == 585:  # get left stalk status
                if cData[2] > leftStalkStatus:
                    leftStalkStatus = cData[2]
            elif pid == 1013:  # get turn signal status
                turnSignal = cData[5] > 0
                if turnSignal:  # delay stalk click until turn signal ends
                    nextClick = time.time() + (0.2 if leftStalkStatus in (4,8) else 4)
                    prevTurnSignal = True
            elif pid == 599:  # get vehicle speed
                speed = cData[3]
            elif pid == 280: # get throttle position
                tempBalls = cData[4] > 200
            elif pid == 962 and cData[3] not in [0,85] and not enabled: # Get right steering wheel up / down swipe
                if cData[3] < 20 and cData[3] > 1:
                    moreBalls = True
                elif cData[3] < 64 and cData[3] > 44:
                    moreBalls = False
        if moreBalls or tempBalls:
            if last820[0] & 32 == 0:  # set throttle mode to Standard / Sport
                last820[0], last820[6], last820[7] = (last820[0] + 32) % 256, (last820[6] + 16) % 256, (last820[7] + 48) % 256
                p.can_send(820, last820, 0)
                last820[0] = 32
            if last659[0] & 16 == 0:  # set throttle mode to Standard / Sport
                last659[5], last659[6], last659[7] = (last659[5] + 16) % 256, (last659[6] + 16) % 256, (last659[7] + 32) % 256
                p.can_send(659, last659, 0)
                last659[0] = 16
    except Exception as e:
        time.sleep(1)
        p = panda.Panda()
        p.set_can_speed_kbps(0,500)
        p.set_safety_mode(panda.Panda.SAFETY_ALLOUTPUT)