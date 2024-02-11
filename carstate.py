import struct

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
        self.accelPedal = 0
        self.parked = True
        self.motorPID = 0
        self.throttlePID = 0
        self.lastStalk = 0
        self.motor = [32]
        self.throttleMode = [0,0,0,0,0,16]
        self.histClick = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.rightStalkCRC = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        self.ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,
                           2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,921,928,935,965,979,997]

        def VehicleSpeed(tstmp, pid, bus, cData):
            self.speed = cData[3]
            return None

        def BiggerBalls(bus):
            if (self.moreBalls or self.tempBalls) and self.motor[0] & 32 == 0 and self.throttleMode[5] & 16 == 0:  # override throttle mode to Standard / Sport
                sendCAN = []
                self.motor[0] = (self.motor[0] + 32) % 256
                self.motor[6] = (self.motor[6] + 16) % 256
                self.motor[7] = (self.motor[7] + 48) % 256
                sendCAN.append((self.motorPID, bus, bytearray(self.motor)))
                self.throttleMode[5] = (self.throttleMode[5] + 16) % 256
                self.throttleMode[6] = (self.throttleMode[6] + 16) % 256
                self.throttleMode[7] = (self.throttleMode[7] + 32) % 256
                sendCAN.append((self.throttlePID, bus, bytearray(self.throttleMode)))
                self.motor[0] = 32
                self.throttleMode[0] = 16
                return sendCAN
            else:
                return None

        def Throttle(tstmp, pid, bus, cData):
            self.throttlePID = pid
            self.throttleMode = cData  # get throttle mode
            self.autoSteer = cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return BiggerBalls(bus)

        def Motor(tstmp, pid, bus, cData):
            self.motorPID = pid
            self.motor = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return BiggerBalls(bus)

        def DriveState(tstmp, pid, bus, cData):
            self.parked = cData[2] & 2 > 0  # check gear state
            self.accelPedal = cData[4]  # get accelerator pedal position
            self.tempBalls = self.accelPedal > 200  # override to standard / sport if throttle is above 78%
            return BiggerBalls(bus)

        def RightScroll(tstmp, pid, bus, cData):
            if cData[3] not in [0,85]:
                if cData[3] <= 64 and cData[3] >= 44 and self.enabled:
                    self.nextClickTime = max(self.nextClickTime, tstmp + 4)  # ensure enough time for the cruise speed decrease to be applied
                    return None
                elif cData[3] < 20 and cData[3] > 1 and not self.enabled:
                    self.moreBalls = True  # up swipe will lock standard / sport throttle override mode
                    return BiggerBalls(bus)
                elif cData[3] < 64 and cData[3] > 44 and not self.enabled:
                    self.moreBalls = False  # down swipe will end throttle override mode
                    return None
            else:
                return None

        def LeftStalk(tstmp, pid, bus, cData):
            if cData[2] & 15 > self.leftStalkStatus:
                self.leftStalkStatus = cData[2] & 15  # Get left stalk status, and bump the status up for full click vs half click turn signal
            return None

        def TurnSignal(tstmp, pid, bus, cData):
            if cData[5] > 0:
                self.nextClickTime = max(self.nextClickTime, tstmp + (0.5 if self.leftStalkStatus in (4,8) else 4)) # Delay spoof if turn signal is on
            return None

        def SteerAngle(tstmp, pid, bus, cData):
            cData[3] &= 63  # mask upper bits
            self.steerAngle = struct.unpack("<1h", cData[2:4])[0] - 8192  # decode angle using multiple / partial bytes
            return None

        def EnoughClicksAlready():
            return sum(self.histClick[-10:]) > 1 or sum(self.histClick[-15:-10]) > 1 or sum(self.histClick[-20:-15]) > 1 or sum(self.histClick[-25:-20]) > 1

        def RightStalk(tstmp, pid, bus, cData):
            if all((self.enabled, self.accelPedal < 100, cData[1] <= 15, tstmp > self.nextClickTime, (self.lastAPStatus == 33 or abs(self.steerAngle) < 50), not EnoughClicksAlready())):
                sendCAN = []
                cData[0] = self.rightStalkCRC[cData[1]]
                cData[1] = (cData[1] + 1) % 16 + 48
                sendCAN.append((pid, bus, bytearray(cData)))  # It's time to spoof or reengage autosteer
                self.histClick.append(1)
                self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)
                self.leftStalkStatus = 0
            else:  # keep track of the number of new stalk clicks (rising edge) to prevent rainbow road and multiple autosteer unavailable alerts
                self.histClick.append(1 if cData[1] >> 4 == 3 and not self.lastStalk >> 4 == 3 else 0)
                self.lastStalk = cData[1]
                sendCAN = None
            self.histClick.pop(0)
            return sendCAN

        def AutoPilotState(tstmp, pid, bus, cData):
            self.lastAPStatus = cData[3] & 33
            if cData[3] & 33 > 0 and self.speed > 0 and self.autoSteer and not self.enabled:
                self.enabled = 1  # AP is active
            elif (cData[3] & 33 == 0 or self.speed == 0) and self.enabled:
                self.enabled = 0  # AP is not active
                if cData[3] & 33 == 0:
                    self.nextClickTime = max(self.nextClickTime, tstmp + 0.5)  # if the car isn't moving or AP isn't engaged, then delay the click
            return None

        self.Update = {}
        self.Update[280]  = DriveState
        self.Update[297]  = SteerAngle
        self.Update[553]  = RightStalk
        self.Update[585]  = LeftStalk
        self.Update[599]  = VehicleSpeed
        self.Update[659]  = Throttle
        self.Update[820]  = Motor
        self.Update[962]  = RightScroll
        self.Update[1001] = AutoPilotState
        self.Update[1013] = TurnSignal
