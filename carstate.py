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
        self.p820 = [32]
        self.p659 = [0,0,0,0,0,16]
        self.p553 = [75,93,98,76,78,210,246,67,170,249,131,70,32,62,52,73]
        self.ignorePIDs = [1000,1005,1060,1107,1132,1284,1316,1321,1359,1364,1448,1508,1524,1541,1542,1547,1550,1588,1651,1697,1698,1723,
                           2036,313,504,532,555,637,643,669,701,772,777,829,854,855,858,859,866,871,872,896,900,921,928,935,965,979,997]
        self.pids = [280, 297, 553, 585, 599, 659, 820, 962, 1001, 1013, 1021]
        self.can_filters = []
        for pid in self.pids:
            self.can_filters.append({"can_id": pid, "can_mask": pid}) #, "extended": False})
        self.last553 = 0
        self.hist553 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
    def enoughAlready(self):
        return sum(self.hist553[-10:]) > 1 or sum(self.hist553[-15:-10]) > 1 or sum(self.hist553[-20:-15]) > 1 or sum(self.hist553[-25:-20]) > 1

    def update(self, data):
        canSend = []

        for tstmp, pid, bus, cData in data:

            if pid == 599:
                self.speed = cData[3]  # get vehicle speed

            elif pid == 280:
                self.parked = cData[2] & 2 > 0  # check gear state
                self.accelPedal = cData[4]  # get accelerator pedal position
                self.tempBalls = cData[4] > 200  # override to standard / sport if throttle is above 78%

            elif pid == 659:
                self.p659 = cData  # get throttle mode
                self.autoSteer = cData[4] & 64  # capture this throttle data in case throttle or up-swipe meets conditions for override soon

            elif pid == 820:
                self.p820 = cData  # capture this throttle data in case throttle or up-swipe meets conditions for override soon

            elif pid == 585 and cData[2] & 15 > self.leftStalkStatus:
                self.leftStalkStatus = cData[2] & 15  # Get left stalk status, and bump the status up for full click vs half click

            elif pid == 1013 and cData[5] > 0:  # get turn signal status
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
                if all((self.enabled, self.accelPedal < 100, cData[1] <= 15, tstmp > self.nextClick, (self.lastAPStatus == 33 or abs(self.steerAngle) < 50), not self.enoughAlready())):
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

            elif pid == 1001:  # get AutoPilot state
                self.lastAPStatus = cData[3] & 33

                if cData[3] & 33 > 0 and self.speed > 0 and self.autoSteer and not self.enabled:
                    self.enabled = cData[2] & 1  # AP is active

                elif (cData[3] & 33 == 0 or cData[2] & 1 == 0 or self.speed == 0) and self.enabled == 1:
                    self.enabled = 0  # AP is not active

                    if cData[3] & 33 == 0 or cData[2] & 1 == 0:
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
