import struct

class CarState():
    def __init__(self):
        self.moreBalls = 0
        self.tempBalls = 0
        self.enabled = 0
        self.driverOverride = 0
        self.autoSteer = 0
        self.nextClick = 0
        self.turnSignal = 0
        self.prevTurnSignal = 0
        self.speed = 0
        self.leftStalkStatus = 0
        self.steerAngle = 0
        self.lastAPStatus = 0
        self.accelPedal = 0
        self.p820 = []
        self.p659 = []
        self.p553 = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        self.counter553 = 0
        self.last553 = 0
        self.hist553 = [0,0,0,0,0,0,0,0,0,0]

    def update(self, data):
        canSend = []
        
        for tstmp, pid, bus, cData in data:

            if pid == 599:  
                self.speed = cData[3]  # get vehicle speed

            elif pid == 280:  
                self.accelPedal = cData[4]  # get accelerator pedal position
                self.tempBalls =cData[4] > 200  # override to standard / sport if throttle is above 78%

            elif pid == 659:  
                self.p659 = cData  # get throttle mode
                self.autoSteer = cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon

            elif pid == 820:  
                self.p820 = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon

            elif pid == 585 and cData[2] & 15 > self.leftStalkStatus:  
                self.leftStalkStatus = cData[2] & 15  # Get left stalk status, and bump the status up for full click vs half click

            elif pid == 1013 and cData[5] > 0:  # get turn signal status
                self.turnSignal = True
                self.prevTurnSignal = self.enabled
                self.nextClick = max(self.nextClick, tstmp + (0.5 if self.leftStalkStatus in (4,8) else 4)) # Delay spoof if turn signal is on

            elif pid == 297:  # get steering angle
                cData[3] &= 63  # mask upper bits
                self.steerAngle = struct.unpack("<1h", cData[2:4])[0] - 8192  # decode angle using multiple / partial bytes

            elif pid == 962 and cData[3] not in [0,85]: # Get right steering wheel up / down swipe

                if cData[3] <= 64 and cData[3] >= 44 and self.enabled:  
                  self.nextClick = max(self.nextClick, tstmp + 4)  # ensure enough time for the cruise speed decrease to be applied

                elif cData[3] < 20 and cData[3] > 1 and not self.enabled:  
                  self.moreBalls = True  # up swipe will lock standard / sport throttle override mode

                elif cData[3] < 64 and cData[3] > 44 and not self.enabled:  
                  self.moreBalls = False  # down swipe will end throttle override mode

            elif pid == 553:
                self.counter553 += 1  # used to ensure that right-stalk click doesn't occur too often in some situations (intersections, etc.)

                if all((self.enabled, self.accelPedal < 100, cData[1] <= 15, tstmp > self.nextClick, (self.lastAPStatus == 33 or abs(self.steerAngle) < 50), sum(self.hist553) <= 1)):
                    cData[0] = self.p553[cData[1]]
                    cData[1] = (cData[1] + 1) % 16 + 48
                    canSend.append((553, bus, bytearray(cData)))  # It's time to spoof or reengage autosteer
                    self.hist553[self.counter553 % 10] = 1
                    self.nextClick = max(self.nextClick, tstmp + 0.5)
                    self.prevTurnSignal = 0
                    self.leftStalkStatus = 0

                else:  # keep track of the number of new stalk clicks (rising edge) to prevent rainbow road and multiple autosteer unavailable alerts
                    self.hist553[self.counter553 % 10] = 1 if cData[1] >> 4 == 3 and not self.last553 >> 4 == 3 else 0
                    self.last553 = cData[1]

            elif pid == 1001:  # get AutoPilot state and check for driver override

                if self.lastAPStatus == 33 and cData[3] & 33 == 32 and not (self.turnSignal or self.prevTurnSignal):  
                    self.driverOverride = True  # prevent auto reengage of autosteer

                elif cData[3] & 33 != 32:  # if speed control is disabled or autosteer is enabled, reset the override and signal history
                    self.driverOverride = False
                    self.prevTurnSignal = self.turnSignal

                self.lastAPStatus = cData[3] & 33

                if cData[3] & 33 > 0 and self.speed > 0 and self.autoSteer and not self.enabled and not self.driverOverride:
                    self.enabled = cData[2] & 1  # AP is active

                elif (cData[3] & 33 == 0 or cData[2] & 1 == 0 or self.speed == 0 or self.driverOverride) and self.enabled == 1:
                    self.enabled = 0  # AP is not active

                    if cData[3] & 33 == 0 or cData[2] & 1 == 0 or self.driverOverride:  
                        self.nextClick = max(self.nextClick, tstmp + 0.5)  # if the car isn't moving or AP isn't engaged, then delay the click

            elif pid == 1021 and cData[0] == 0 and cData[6] & 1 == 0:
                print("Double-Pull AP engagement is required for Auto-Reengage to work")

        if (self.moreBalls or self.tempBalls) and self.p820[0] & 32 == 0 and self.p659[5] & 16 == 0:  # override throttle mode to Standard / Sport
            self.p820[0] = (self.p820[0] + 32) % 256
            self.p820[6] = (self.p820[6] + 16) % 256
            self.p820[7] = (self.p820[7] + 48) % 256
            canSend.append((820, bus, bytearray(self.p820)))
            self.p659[5] = (self.p659[5] + 16) % 256
            self.p659[6] = (self.p659[6] + 16) % 256
            self.p659[7] = (self.p659[7] + 32) % 256
            canSend.append((659, bus, bytearray(self.p659)))
            self.p820[0] = 32
            self.p659[0] = 16
        return canSend
