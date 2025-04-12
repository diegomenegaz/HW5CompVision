from maestro import Controller

HeadVertPORT = 4
HeadHorPORT = 3
MPORT = 0
MPORT2 = 1

class MovementControl:
    _instance = None

    @staticmethod
    def getInst():
        if MovementControl._instance is None:
            MovementControl._instance = MovementControl()
        return MovementControl._instance

    def __init__(self):
        self.m = Controller("/dev/ttyUSB0")  # Replace with your correct device

    def setTarget(self, port, value):
        self.m.setTarget(port, value)

    def arc_L(self):
        self.setTarget(MPORT, 5500)
        self.setTarget(MPORT2, 7000)

    def arc_R(self):
        self.setTarget(MPORT, 7000)
        self.setTarget(MPORT2, 5500)

    def stop(self):
        self.setTarget(MPORT, 6000)
        self.setTarget(MPORT2, 6000)

    def pan(self, val):
        self.setTarget(HeadHorPORT, val)

    def tilt(self, val):
        self.setTarget(HeadVertPORT, val)
