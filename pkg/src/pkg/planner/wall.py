
class Wall:

    def __init__(self, speed_param=None):
        if speed_param == None:
            self.speed = 1.0
        else:
            self.speed = speed_param
        self.steer = 0

    def process_lidar(self, scan_msg):
        go = scan_msg[540]
        right = scan_msg[320]
        left = scan_msg[760]
        rightside = scan_msg[270]
        leftside = scan_msg[810]

        # print(go)


        if go<4 :
            self.speed = 2
            self.steer = 0
            if right<left:
                self.speed=3
                self.steer=1.5
            elif left<right:
                self.speed=3
                self.steer=-1.5

        elif rightside < 0.2:
            self.speed=1
            self.steer=2
        elif leftside < 0.2:
            self.speed=1
            self.steer=-2
        else :
            self.speed=6
            self.steer=0

        return self.speed, self.steer