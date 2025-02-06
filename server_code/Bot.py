
class Bot:
    def __init__(self, nameSpace:str):
        self.nameSpace = nameSpace
        self.currentPosition = "None"
        self.currentOrientation ="None"
        self.currentTrajectory = [[-1000,-1000,-1000]]
        self.currentBatteryPercentage = -1.0
        

