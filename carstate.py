MIN_REGEN_PERCENT = 95   # Must be increments of 5%
MAX_REGEN_PERCENT = 100  # Must be increments of 5%
MIN_REGEN_AFTER_SPEED = 60
MAX_REGEN_BELOW_SPEED = 50

REGEN_SLOPE = (MIN_REGEN_AFTER_SPEED - MAX_REGEN_BELOW_SPEED) / (0.2 * (MAX_REGEN_PERCENT - MIN_REGEN_PERCENT))
MAX_ADJUST = (100 - MIN_REGEN_PERCENT) * 0.2
MIN_ADJUST = (100 - MAX_REGEN_PERCENT) * 0.2
print(REGEN_SLOPE, MAX_ADJUST, MIN_ADJUST)

class CarState():
    def __init__(self):
        self.moreBalls = 0
        self.tempBalls = 0
        self.enabled = 0
        self.autoSteer = 0
        self.nextClickTime = 0.
        self.speed = 0
        self.steerAngle = 0
        self.lastAPStatus = 0
        self.lastAutoSteerTime = 0
        self.accelPedal = 0
        self.parked = True
        self.motorPID = 0
        self.throttlePID = 0
        self.lastStalk = 0
        self.autopilotReady = 1
        self.handsOnState = 0
        self.handsOnLevel = 0
        self.driverSteerTorque = 0
        self.steerTorqueAdjust = 0
        self.brakePressed = 0
        self.autoEngage = 0
        self.avgLaneCenter = 100.
        self.fastAvgSpeed = 0.
        self.avgSpeed = 0.
        self.chassisBusAvailable = 0
        self.FSD = False
        self.blinkersOn = False
        self.closeToCenter = False
        self.logFile = None
        self.notDecelerating = False
        self.accelerating = False
        self.conditionBattery = False
        self.sendCAN = []
        self.stalkCondition = {}
        self.allPIDs = [[],[]]
        self.motor = [32,0,20]
        self.throttleMode = [0,0,0,0,0,16]
        self.histClick = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.rightStalkCRC = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        self.leftStalkCRC = {2: [208,18,235,235,187,116,102,7,102,218,16,2,43,151,246,163],
                             6: [152,90,163,163,243,60,46,79,46,146,88,74,99,223,190,235]}
        self.ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,
                           2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,928,935,965,979,997]
        self.recordPIDs = [264, 697, 744, 880, 905, 1160]

        def SendCAN(tstmp):
            data = []
            for i in range(len(self.sendCAN) - 1, 0, -1):
                if self.sendCAN[i][0] <= tstmp:
                    data.append(self.sendCAN.pop(i)[1:])
            if len(data) > 0:
                return data
            else:
                return None

        def BiggerBalls(tstmp, bus):
            if (self.moreBalls or self.tempBalls) and self.motor[0] & 32 == 0 and self.throttleMode[5] & 16 == 0:  # override throttle mode to Standard / Sport
                self.motor[0] = (self.motor[0] + 32) & 255
                self.motor[6] = (self.motor[6] + 16) & 255
                self.motor[7] = (self.motor[7] + 48) & 255
                self.sendCAN.append((tstmp, self.motorPID, bus, bytearray(self.motor)))
                self.throttleMode[5] = (self.throttleMode[5] + 16) & 255
                self.throttleMode[6] = (self.throttleMode[6] + 16) & 255
                self.throttleMode[7] = (self.throttleMode[7] + 32) & 255
                self.sendCAN.append((tstmp, self.throttlePID, bus, bytearray(self.throttleMode)))
                self.motor[0] = 32
                self.throttleMode[0] = 16
            return SendCAN(tstmp)

        def AdjustRegenBraking(tstmp, bus):
            if self.lastAPStatus != 32 and (self.nextClickTime < tstmp or self.lastAPStatus == 33):  # override Regen percent to be 100% at speed up to 30 MPH then steadily decrease to 0% as speed increases to 60+ MPH
                regenAdjustment = int(min(MAX_ADJUST, max(MIN_ADJUST, ((self.speed - MAX_REGEN_BELOW_SPEED) / REGEN_SLOPE))))
                self.motor[2] = (self.motor[2] - regenAdjustment) & 255
                self.motor[6] = (self.motor[6] + 16) & 255
                self.motor[7] = (self.motor[7] + 16 - regenAdjustment) & 255
                self.sendCAN.append((tstmp, self.motorPID, bus, bytearray(self.motor)))
            elif self.nextClickTime >= tstmp and self.motor[2] < 20:
                self.motor[7] = (self.motor[7] + 16 + 20 - self.motor[2]) & 255
                self.motor[6] = (self.motor[6] + 16) & 255
                self.motor[2] = 20
                self.sendCAN.append((tstmp, self.motorPID, bus, bytearray(self.motor)))
            return SendCAN(tstmp)

        def Throttle(tstmp, pid, bus, cData):
            self.throttlePID = pid
            self.throttleMode = cData  # get throttle mode
            self.autoSteer = cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return BiggerBalls(tstmp, bus)

        def Motor(tstmp, pid, bus, cData):
            self.motorPID = pid
            self.motor = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            if self.moreBalls or self.tempBalls:
                return BiggerBalls(tstmp, bus)
            else:
                return AdjustRegenBraking(tstmp, bus)
            
        def DriveState(tstmp, pid, bus, cData):
            self.parked = cData[2] & 2 > 0  # check gear state
            self.accelPedal = cData[4]  # get accelerator pedal position
            self.tempBalls = self.accelPedal > 200  # override to standard / sport if throttle is above 78%
            if self.parked:  self.autoEngage = 0
            return BiggerBalls(tstmp, bus)

        def RightScroll(tstmp, pid, bus, cData):
            if cData[3] not in [0,85]:
                if cData[3] <= 64 and cData[3] >= 44 and self.enabled:
                    self.nextClickTime = max(self.nextClickTime, tstmp + 4)  # ensure enough time for the cruise speed decrease to be applied
                elif cData[3] < 20 and cData[3] > 1 and not self.enabled and not self.moreBalls:
                    self.moreBalls = True  # up swipe will lock standard / sport throttle override mode
                elif cData[3] < 20 and cData[3] > 1 and not self.enabled and not self.conditionBattery:
                    self.conditionBattery = True  # up swipe will lock standard / sport throttle override mode
                elif cData[3] < 64 and cData[3] > 44 and not self.enabled and self.conditionBattery:
                    self.conditionBattery = False  # down swipe will end throttle override mode
                elif cData[3] < 64 and cData[3] > 44 and not self.enabled and self.moreBalls:
                    self.moreBalls = False  # down swipe will end throttle override mode
            return BiggerBalls(tstmp, bus)

        def VehicleSpeed(tstmp, pid, bus, cData):
            self.avgSpeed += 0.1 * (cData[3] - self.avgSpeed)
            if not self.brakePressed and (self.avgSpeed - cData[3]) < 0.01:
                self.notDecelerating = True
            else:
                self.notDecelerating = False
            if cData[3] > self.speed and self.speed >= 10:
                self.accelerating = True
            elif cData[3] < self.speed:
                self.accelerating = False
            self.speed = cData[3]
            return SendCAN(tstmp)

        def LeftStalk(tstmp, pid, bus, cData):
            if cData[2] & 15 > 0:
                self.closeToCenter = False
                self.nextClickTime = max(self.nextClickTime, tstmp + 1.) # Delay spoof if turn signal is on
            if self.lastAPStatus == 33 and not self.blinkersOn and cData[2] & 15 in (2,6):
                cData[0] = self.leftStalkCRC[cData[2]][cData[1]]
                cData[1] = (cData[1] + 1) & 15
                cData[2] = cData[2] + 2
                cData[3] = 0
                self.sendCAN.append((tstmp + 0.025, pid, bus, bytearray(cData)))
                cData[0] = self.leftStalkCRC[cData[2] - 2][cData[1]]
                cData[1] = (cData[1] + 1) & 15
                self.sendCAN.append((tstmp + 0.075, pid, bus, bytearray(cData)))
            return SendCAN(tstmp)

        def TurnSignal(tstmp, pid, bus, cData):
            #PrintBitsAndBytes(tstmp, pid, bus, cData)
            if cData[0] % 16 > 0:
                self.closeToCenter = False
                self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)
            self.blinkersOn = cData[0] % 16 > 0
            return SendCAN(tstmp)

        def SteerAngle(tstmp, pid, bus, cData):
            cData[3] &= 63  # mask upper bits
            self.steerAngle = ((int.from_bytes(cData, byteorder='little', signed=False) >> 16) & 16383) - 8192
            return SendCAN(tstmp)

        def BrakePedal(tstmp, pid, bus, cData):
            self.brakePressed = cData[2] & 2
            if self.brakePressed:
                if self.FSD:  self.autoEngage = 0
                self.accelerating = False
                self.notDecelerating = False
            return SendCAN(tstmp)

        def VirtualLane(tstmp, pid, bus, cData):
            if self.lastAPStatus == 33:
                self.avgLaneCenter += 0.05 * (cData[2] - self.avgLaneCenter)
            laneOffset = cData[2] - self.avgLaneCenter
            self.closeToCenter = abs(laneOffset) < 20 and not self.blinkersOn and ((tstmp - self.lastAutoSteerTime) > 2. or self.lastAPStatus == 33)
            return SendCAN(tstmp)

        def DriverAssistState(tstmp, pid, bus, cData):
            self.lastAPStatus = cData[3] & 33
            if self.lastAPStatus == 33:
                self.lastAutoSteerTime = tstmp
            if cData[3] & 33 > 0 and self.speed > 0 and self.autoSteer and not self.enabled:
                self.enabled = 1  # AP is active
            elif (cData[3] & 33 == 0 or self.speed == 0) and self.enabled:
                self.enabled = 0  # AP is not active
                if cData[3] & 33 == 0:
                    self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)  # if the car isn't moving or AP isn't engaged, then delay the click
            return SendCAN(tstmp)

        def EnoughClicksAlready():  # Prevent unintended triggering of the "Rainbow Road" Easter Egg
            return sum(self.histClick[-10:]) > 1 or sum(self.histClick[-15:-10]) > 1 or sum(self.histClick[-20:-15]) > 1 or sum(self.histClick[-25:-20]) > 1

        def dualCANFSD(tstmp, cData):
            return any((all((self.autoEngage, not self.brakePressed, self.autopilotReady, self.closeToCenter, self.notDecelerating, (tstmp - self.lastAutoSteerTime) > 2.)), self.lastAPStatus == 33))

        def dualCANAP(tstmp, cData):
            self.closeToCenter = True
            return any((all((self.autoEngage, self.notDecelerating or self.accelerating, self.autopilotReady, self.closeToCenter, (tstmp - self.lastAutoSteerTime) > 2., abs(self.steerAngle) < 50)),
                        all((self.enabled, self.accelPedal < 100, not self.brakePressed, self.autopilotReady, (self.lastAPStatus == 33 or abs(self.steerAngle) < 50)))))

        def singleCANAP(tstmp, cData):
            return all((self.enabled, self.accelPedal < 100, not self.brakePressed, (self.lastAPStatus == 33 or abs(self.steerAngle) < 50)))

        def APMode(tstmp, pid, bus, cData):
            if cData[0] == 0 and self.chassisBusAvailable:
                if cData[6] & 1 == 0 and self.stalkCondition is not dualCANFSD:
                    self.stalkCondition = dualCANFSD
                    self.FSD = True
                    print("FSD mode")
                elif cData[6] & 1 == 1 and self.stalkCondition is not dualCANAP:
                    self.stalkCondition = dualCANAP
                    self.FSD = False
                    print("AP mode")

        def RightStalk(tstmp, pid, bus, cData):
            #PrintBitsAndBytes(tstmp, pid, bus, cData)
            #print(self.stalkCondition(tstmp, cData), not self.blinkersOn, cData[1] <= 15, tstmp > self.nextClickTime, not EnoughClicksAlready())
            if all((self.stalkCondition(tstmp, cData), not self.blinkersOn, cData[1] <= 15, tstmp > self.nextClickTime, not EnoughClicksAlready())):
                cData[0] = self.rightStalkCRC[cData[1]]
                cData[1] = (cData[1] + 1) % 16 + 48
                self.sendCAN.append((tstmp + 0.05, pid, bus, bytearray(cData)))  # It's time to spoof or reengage autosteer
                self.histClick.append(1)
                self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)
                self.autoEngage = self.chassisBusAvailable
            else:  # keep track of the number of new stalk clicks (rising edge) to prevent rainbow road and multiple autosteer unavailable alerts
                self.histClick.append(1 if cData[1] >> 4 == 3 and not self.lastStalk >> 4 == 3 else 0)
                if (cData[1] >> 4) & 7 in [3,4] and self.motor[2] < 20:
                    self.nextClickTime = tstmp + 0.25
                    AdjustRegenBraking(tstmp, bus)
                elif (cData[1] >> 4) & 7 in [1,2]:
                    self.autoEngage = 0
            self.lastStalk = cData[1]
            self.histClick.pop(0)
            return SendCAN(tstmp)

        def AutoPilotState(tstmp, pid, bus, cData):
            # Hands On State: 0 "NOT_REQD" 1 "REQD_DETECTED" 2 "REQD_NOT_DETECTED" 3 "REQD_VISUAL" 4 "REQD_CHIME_1" 5 "REQD_CHIME_2" 6 "REQD_SLOWING" 
            #                 7 "REQD_STRUCK_OUT" 8 "SUSPENDED" ;9 "REQD_ESCALATED_CHIME_1" 10 "REQD_ESCALATED_CHIME_2" 15 "SNA" 
            self.handsOnState = (cData[5] >> 2) & 15
            self.autopilotReady = cData[0] & 15 in [2, 3, 4, 5]
            self.chassisBusAvailable = 1
            if self.lastAPStatus == 33 and self.handsOnState in [0, 1, 7, 8, 15]:
                self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)
            return SendCAN(tstmp)
            
        def PrintBits(tstmp, pid, bus, cData):
            print(pid, bus, "{0:64b}".format(int.from_bytes(cData, byteorder='little', signed=False)))
            return None

        def PrintBytes(tstmp, pid, bus, cData):
            print(pid, bus, ["%3.0f" % d for d in cData])
            return None

        def PrintBitsAndString(tstmp, pid, bus, cData):
            print("%0.4f" % tstmp, pid, bus, "{0:64b}".format(int.from_bytes(cData, byteorder='little', signed=False)), cData)
            return None

        def PrintBitsAndBytes(tstmp, pid, bus, cData):
            print("%0.4f" % tstmp, pid, bus, "{0:64b}".format(int.from_bytes(cData, byteorder='little', signed=False)), ["%3.0f" % d for d in cData])
            return None

        def Research(tstmp, pid, bus, cData):
            if self.logFile is None:
                self.monitorPIDs = [[],[],[]]
                self.allPIDs = [[],[],[]]
                self.baselineBits = [{},{},{}]
                self.lastBytes = [{},{},{}]
                self.allBitChanges = [{},{},{}]
                self.logFile = open('/home/raspilot/raspilot/bitChanges-%0.0f.dat' % (tstmp//60), "a")
                print(self.logFile)

            if (len(self.monitorPIDs[bus]) == 0 or pid in self.monitorPIDs[bus]) and not pid in self.allPIDs[bus]:
                self.allPIDs[bus].append(pid)
                for b in range(len(cData)):
                    self.allBitChanges[bus]["%s|%d" % (pid, b)] = 0
                    self.lastBytes[bus]["%s|%d" % (pid, b)] = cData[b]

            if pid in self.allPIDs[bus]:
                for b in range(len(cData)):
                    if "%s|%d" % (pid, b) not in self.lastBytes[bus]:
                        self.lastBytes[bus]["%s|%d" % (pid, b)] = cData[b]
                        self.allBitChanges[bus]["%s|%d" % (pid, b)] = 0
                    newBitChanges = self.lastBytes[bus]["%s|%d" % (pid, b)] ^ cData[b]
                    if self.allBitChanges[bus]["%s|%d" % (pid, b)] != self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges:
                        print(int(tstmp), bus, pid, b, self.allBitChanges[bus]["%s|%d" % (pid, b)] ^ newBitChanges, self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges)
                        self.logFile.write("%0.0f %d %d %d %d\n" % (tstmp, pid, b, self.allBitChanges[bus]["%s|%d" % (pid, b)] ^ newBitChanges, self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges))
                        self.allBitChanges[bus]["%s|%d" % (pid, b)] = self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges
                    self.lastBytes[bus]["%s|%d" % (pid, b)] = cData[b]

        self.dualCANFSD = dualCANFSD
        self.dualCANAP = dualCANAP
        self.singleCANAP = singleCANAP
        
        self.Update = [{  # Vehicle Bus
                            280:  DriveState,
                            297:  SteerAngle,
                            553:  RightStalk,
                            585:  LeftStalk,
                            599:  VehicleSpeed,
                            659:  Throttle,
                            820:  Motor,
                            925:  BrakePedal,
                            962:  RightScroll,
                            1001: DriverAssistState,
                            1013: TurnSignal,
                            1021: APMode
                       },
                       {  # Chassis Bus
                            569:  VirtualLane,
                            921:  AutoPilotState,
                       },
                       Research]

    def InitStalkCondition(self):
        if self.FSD:
            print("FSD Mode enabled")
            self.stalkCondition = self.dualCANFSD
        elif self.chassisBusAvailable:
            print("Dual CAN AP Mode enabled")
            self.stalkCondition = self.dualCANAP
        else:
            print("Single CAN AP Mode enabled")
            self.stalkCondition = self.singleCANAP
