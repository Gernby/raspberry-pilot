import time
import threading
#from sshkeyboard import listen_keyboard

class CarState():
    def __init__(self):
        self.moreBalls = 0
        self.tempBalls = 0
        self.enabled = 0
        self.autoSteer = 0
        self.nextClickTime = 0.
        self.speed = 0
        self.leftStalkStatus = 0
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
        self.brakePressed = 0
        self.chassisBusAvailable = 0
        self.autoEngage = 0
        self.avgLaneCenter = 100.
        self.blinkersOn = False
        self.closeToCenter = False
        self.logFile = None
        self.sendCAN = []
        self.motor = [32]
        self.throttleMode = [0,0,0,0,0,16]
        self.histClick = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.rightStalkCRC = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        self.ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,
                           2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,928,935,965,979,997]

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

        def Throttle(tstmp, pid, bus, cData):
            self.throttlePID = pid
            self.throttleMode = cData  # get throttle mode
            self.autoSteer = cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return BiggerBalls(tstmp, bus)

        def Motor(tstmp, pid, bus, cData):
            self.motorPID = pid
            self.motor = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return BiggerBalls(tstmp, bus)

        def DriveState(tstmp, pid, bus, cData):
            self.parked = cData[2] & 2 > 0  # check gear state
            self.accelPedal = cData[4]  # get accelerator pedal position
            self.tempBalls = self.accelPedal > 200  # override to standard / sport if throttle is above 78%
            return BiggerBalls(tstmp, bus)

        def RightScroll(tstmp, pid, bus, cData):
            if cData[3] not in [0,85]:
                if cData[3] <= 64 and cData[3] >= 44 and self.enabled:
                    self.nextClickTime = max(self.nextClickTime, tstmp + 4)  # ensure enough time for the cruise speed decrease to be applied
                elif cData[3] < 20 and cData[3] > 1 and not self.enabled:
                    self.moreBalls = True  # up swipe will lock standard / sport throttle override mode
                elif cData[3] < 64 and cData[3] > 44 and not self.enabled:
                    self.moreBalls = False  # down swipe will end throttle override mode
            return BiggerBalls(tstmp, bus)

        def VehicleSpeed(tstmp, pid, bus, cData):
            self.speed = cData[3]
            return SendCAN(tstmp)

        def LeftStalk(tstmp, pid, bus, cData):
            if cData[2] & 15 > self.leftStalkStatus:
                self.blinkersOn = True
                self.closeToCenter = False
                self.leftStalkStatus = cData[2] & 15  # Get left stalk status, and bump the status up for full click vs half click turn signal
                self.nextClickTime = max(self.nextClickTime, tstmp + 1.) # Delay spoof if turn signal is on
            return SendCAN(tstmp)

        def TurnSignal(tstmp, pid, bus, cData):
            if cData[5] > 0:
                self.closeToCenter = False
                self.nextClickTime = max(self.nextClickTime, tstmp + (0.5 if self.leftStalkStatus in (4,8) else 4.)) # Delay spoof if turn signal is on
            self.blinkersOn = cData[5] > 0
            return SendCAN(tstmp)

        def SteerAngle(tstmp, pid, bus, cData):
            cData[3] &= 63  # mask upper bits
            self.steerAngle = ((int.from_bytes(cData, byteorder='little', signed=False) >> 16) & 16383) - 8192
            return SendCAN(tstmp)

        def BrakePedal(tstmp, pid, bus, cData):
            self.brakePressed = cData[2] & 2
            return SendCAN(tstmp)

        def VirtualLane(tstmp, pid, bus, cData):
            if self.lastAPStatus == 33:
                self.avgLaneCenter += 0.01 * (cData[2] - self.avgLaneCenter)
            laneOffset = cData[2] - self.avgLaneCenter
            self.closeToCenter = abs(laneOffset) < 10 and not self.blinkersOn and self.lastAPStatus == 32 and (tstmp - self.lastAutoSteerTime) > 2.
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

        def RightStalk(tstmp, pid, bus, cData):
            if all(((self.enabled or self.autoEngage), self.autopilotReady, not self.brakePressed, not self.blinkersOn, self.accelPedal < 100, cData[1] <= 15, 
                    (tstmp > self.nextClickTime or self.closeToCenter), (self.lastAPStatus == 33 or abs(self.steerAngle) < 50), not EnoughClicksAlready())):
                cData[0] = self.rightStalkCRC[cData[1]]
                cData[1] = (cData[1] + 1) % 16 + 48
                self.sendCAN.append((tstmp + 0.05, pid, bus, bytearray(cData)))  # It's time to spoof or reengage autosteer
                self.histClick.append(1)
                self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)
                self.leftStalkStatus = 0
                self.autoEngage = self.chassisBusAvailable
            else:  # keep track of the number of new stalk clicks (rising edge) to prevent rainbow road and multiple autosteer unavailable alerts
                self.histClick.append(1 if cData[1] >> 4 == 3 and not self.lastStalk >> 4 == 3 else 0)
                if cData[1] >> 4 == 1:
                    self.autoEngage = 0
            self.lastStalk = cData[1]
            self.histClick.pop(0)
            return SendCAN(tstmp)

        def AutoPilotState(tstmp, pid, bus, cData):
            # Hands On State: 0 "NOT_REQD" 1 "REQD_DETECTED" 2 "REQD_NOT_DETECTED" 3 "REQD_VISUAL" 4 "REQD_CHIME_1" 5 "REQD_CHIME_2" 6 "REQD_SLOWING" 
            #                 7 "REQD_STRUCK_OUT" 8 "SUSPENDED" ;9 "REQD_ESCALATED_CHIME_1" 10 "REQD_ESCALATED_CHIME_2" 15 "SNA" 
            self.handsOnState = (cData[5] >> 2) & 15
            self.autopilotReady = cData[0] & 15 in [2, 3, 5]
            self.chassisBusAvailable = 1
            if self.lastAPStatus == 33 and self.handsOnState in [0, 1, 7, 8, 15]:
                self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)
            return SendCAN(tstmp)

        def DASSpeed(tstmp, pid, bus, cData):
            self.cruiseSpeed = 0.0621 * (int.from_bytes(cData, byteorder='little', signed=False) & 4095)

        def PrintBits(tstmp, pid, bus, cData):
            print(pid, bus, "{0:64b}".format(int.from_bytes(cData, byteorder='little', signed=False)))
            return None

        def PrintBytes(tstmp, pid, bus, cData):
            print(pid, bus, ["%3.0f" % d for d in cData])
            return None

        def PrintBitsAndString(tstmp, pid, bus, cData):
            print("%0.4f" % tstmp, pid, bus, "{0:64b}".format(int.from_bytes(cData, byteorder='little', signed=False)), cData)
            return None

        def Research(tstmp, pid, bus, cData):
            if self.logFile is None:
                self.monitorPIDs = [[],[]]
                self.allPIDs = [[],[]]
                self.baselineBits = [{},{}]
                self.lastBytes = [{},{}]
                self.allBitChanges = [{},{}]
                self.logFile = open('/home/raspilot/raspilot/bitChanges-%0.0f.dat' % (tstmp//60), "a")

                def monitor_keys():
                    def press(key):
                        print(f"'{key}' pressed")
                        self.logFile.write("%0.0f  %s pressed" % (time.time(), str(key)))
                    
                    while True:
                        listen_keyboard(on_press=press, sleep=0.1)
                
                self.t = threading.Thread(target=monitor_keys, )

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
                        print(int(tstmp), pid, b, self.allBitChanges[bus]["%s|%d" % (pid, b)] ^ newBitChanges, self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges)
                        self.logFile.write("%0.0f %d %d %d %d\n" % (tstmp, pid, b, self.allBitChanges[bus]["%s|%d" % (pid, b)] ^ newBitChanges, self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges))
                        self.allBitChanges[bus]["%s|%d" % (pid, b)] = self.allBitChanges[bus]["%s|%d" % (pid, b)] | newBitChanges
                    self.lastBytes[bus]["%s|%d" % (pid, b)] = cData[b]

        self.Update = [{  # Vehicle Bus
                            280:  DriveState,
                            297:  SteerAngle,
                            553:  RightStalk,
                            585:  LeftStalk,
                            599:  VehicleSpeed,
                            659:  Throttle,
                            820:  Motor,
                            #851: PrintBitsAndString,
                            925:  BrakePedal,
                            962:  RightScroll,
                            1001: DriverAssistState,
                            1013: TurnSignal
                       },
                       {  # Chassis Bus
                            569:  VirtualLane,
                            #697:  DASSpeed,
                            921:  AutoPilotState,
                       },
                       Research]
