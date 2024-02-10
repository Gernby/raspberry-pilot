import struct

class CarState():
    def __init__(self):
        self.moreBalls = 0
        self.tempBalls = 0
        self.enabled = 0
        self.autoSteer = 0
        self.nextClick = 0
        self.speed = 0
        self.leftStalkStatus = 0
        self.steerAngle = 0
        self.lastAPStatus = 0
        self.accelPedal = 0
        self.parked = True
        self.motor = [32]
        self.throttleMode = [0,0,0,0,0,16]
        self.p553 = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        self.ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,
                           2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,921,928,935,965,979,997]
        self.pids = [280, 297, 553, 585, 599, 659, 820, 962, 1001, 1013, 1021]
        self.can_filters = []
        for pid in self.pids:
            self.can_filters.append({"can_id": pid, "can_mask": pid, "extended": False})
        self.last553 = 0
        self.hist553 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.update = {}

        def vehicleSpeed(tstmp, bus, cData):
            self.speed = cData[3]

        def driveState(tstmp, bus, cData):
            self.parked = cData[2] & 2 > 0  # check gear state
            self.accelPedal = cData[4]  # get accelerator pedal position
            self.tempBalls = cData[4] > 200  # override to standard / sport if throttle is above 78%

        def biggerBalls(bus):
            canSend = []
            if (self.moreBalls or self.tempBalls) and self.motor[0] & 32 == 0 and self.throttleMode[5] & 16 == 0:  # override throttle mode to Standard / Sport
                self.motor[0] = (self.motor[0] + 32) % 256
                self.motor[6] = (self.motor[6] + 16) % 256
                self.motor[7] = (self.motor[7] + 48) % 256
                canSend.append((820, bus, bytearray(self.motor)))
                self.throttleMode[5] = (self.throttleMode[5] + 16) % 256
                self.throttleMode[6] = (self.throttleMode[6] + 16) % 256
                self.throttleMode[7] = (self.throttleMode[7] + 32) % 256
                canSend.append((659, bus, bytearray(self.throttleMode)))
                self.motor[0] = 32
                self.throttleMode[0] = 16
            return canSend

        def throttle(tstmp, bus, cData):
            self.throttleMode = cData  # get throttle mode
            self.autoSteer = cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return biggerBalls(bus)

        def motor(tstmp, bus, cData):
            self.motor = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon
            return biggerBalls(bus)

        def leftStalk(tstmp, bus, cData):
            if cData[2] & 15 > self.leftStalkStatus:
                self.leftStalkStatus = cData[2] & 15  # Get left stalk status, and bump the status up for full click vs half click
        
        def turnSignal(tstmp, bus, cData):
            if cData[5] > 0:
                self.nextClick = max(self.nextClick, tstmp + (0.5 if self.leftStalkStatus in (4,8) else 4)) # Delay spoof if turn signal is on

        def steerAngle(tstmp, bus, cData):
            cData[3] &= 63  # mask upper bits
            self.steerAngle = struct.unpack("<1h", cData[2:4])[0] - 8192  # decode angle using multiple / partial bytes

        def rightScroll(tstmp, bus, cData):
            if cData[3] not in [0,85]:
                if cData[3] <= 64 and cData[3] >= 44 and self.enabled:
                    self.nextClick = max(self.nextClick, tstmp + 4)  # ensure enough time for the cruise speed decrease to be applied

                elif cData[3] < 20 and cData[3] > 1 and not self.enabled:
                  self.moreBalls = True  # up swipe will lock standard / sport throttle override mode

                elif cData[3] < 64 and cData[3] > 44 and not self.enabled:
                  self.moreBalls = False  # down swipe will end throttle override mode

        def enoughAlready():
            return sum(self.hist553[-10:]) > 1 or sum(self.hist553[-15:-10]) > 1 or sum(self.hist553[-20:-15]) > 1 or sum(self.hist553[-25:-20]) > 1

        def rightStalk(tstmp, bus, cData):
            canSend = []

            if all((self.enabled, self.accelPedal < 100, cData[1] <= 15, tstmp > self.nextClick, (self.lastAPStatus == 33 or abs(self.steerAngle) < 50), not enoughAlready())):
                cData[0] = self.p553[cData[1]]
                cData[1] = (cData[1] + 1) % 16 + 48
                canSend.append((553, bus, bytearray(cData)))  # It's time to spoof or reengage autosteer
                self.hist553.append(1)
                self.nextClick = max(self.nextClick, tstmp + 0.5)
                self.leftStalkStatus = 0

            else:  # keep track of the number of new stalk clicks (rising edge) to prevent rainbow road and multiple autosteer unavailable alerts
                self.hist553.append(1 if cData[1] >> 4 == 3 and not self.last553 >> 4 == 3 else 0)
                self.last553 = cData[1]
            self.hist553.pop(0)
            return canSend

        def autopilotState(tstmp, bus, cData):
            self.lastAPStatus = cData[3] & 33

            if cData[3] & 33 > 0 and self.speed > 0 and self.autoSteer and not self.enabled:
                self.enabled = 1  # AP is active

            elif (cData[3] & 33 == 0 or self.speed == 0) and self.enabled:
                self.enabled = 0  # AP is not active

                if cData[3] & 33 == 0:
                    self.nextClick = max(self.nextClick, tstmp + 0.5)  # if the car isn't moving or AP isn't engaged, then delay the click

        self.update[599] = vehicleSpeed
        self.update[280] = driveState
        self.update[659] = throttle
        self.update[820] = motor
        self.update[585] = leftStalk
        self.update[1013] = turnSignal
        self.update[297] = steerAngle
        self.update[962] = rightScroll
        self.update[553] = rightStalk        
        self.update[1001] = autopilotState
