import numpy as np
import math

class SpeedController:
    def __init__(self, params):

        self.MU = params['mu']
        self.GRAVITY_ACC = params['g']
        self.PI = params['pi']
        self.WHEEL_BASE = params['racecar_length']
        self.SPEED_MAX = params['max_speed'] # 15.0
        self.SPEED_MIN = params['min_speed']

        self.sus_a = params['sus_a']
        self.sus_b = params['sus_b']
        self.braking_a = params['braking_a']
        self.braking_b = params['braking_b']

        self.scan = []
        self.current_speed = 5.0
        self.steering_angle = 0.0
        self.current_idx = 0
        self.lap_count = 0

    def braking_distance(self):
        current_distance = np.fabs(np.average(self.scan[499:580]))

        if np.isnan(current_distance):
            print("SCAN ERROR")
            current_distance = 1.0
        # braking_a: -1
        braking_speed = np.sqrt(2 * self.MU * self.GRAVITY_ACC * np.fabs(current_distance)) - self.braking_a
        # braking_b: 1.1
        braking_speed *= self.braking_b

        if braking_speed >= self.SPEED_MAX:
            braking_speed = self.SPEED_MAX

        return braking_speed

    def angle_based(self, max_speed=8.0, min_speed=4.0):
        if np.fabs(self.steering_angle) > self.PI / 8:
            angular_speed = min_speed
        else:
            angular_speed = float(-(3 / self.PI) * (max_speed - min_speed) * np.fabs(self.steering_angle) + max_speed)

        return angular_speed

    def combined_speed(self):
        current_distance = np.fabs(np.average(self.scan[499:580]))
        combined_speed = 0

        braking_speed = self.braking_distance()

        if current_distance < 5:
            if self.current_speed < 9:
                combined_speed = self.angle_based()
            else:
                combined_speed = self.angle_based(self.SPEED_MAX, self.current_speed)
        else:
            combined_speed = self.speed_suspension(braking_speed)

        return combined_speed

    def speed_suspension(self, set_speed):
        final_speed = 0
        if self.current_speed <= set_speed:
            final_speed = set_speed
        else:
            # sus_b
            final_speed = self.current_speed - np.fabs((set_speed - self.current_speed) * self.sus_b)

        return final_speed

    def routine(self, scan, speed, steer, idx, mode):
        calculated_speed = 0

        self.scan = scan
        self.current_speed = speed
        self.steering_angle = steer
        self.current_idx = idx
        # print(self.current_idx)
        
        #%
        if self.current_idx >= 4000 and self.lap_count == 0:
            # print(self.lap_count)
            self.lap_count = 1
        # lap_time: 78.55
        if self.current_idx < 200 and self.lap_count == 0 :#or (self.current_idx > 500 and self.current_idx < 550) :#or (self.current_idx > 700 and self.current_idx < 750): #350 , 525
            # print(self.current_idx)
            self.SPEED_MAX = 20 #20
            self.braking_a = -10.0 #-5.0
            self.sus_b = 3.25 #3.25

        # elif self.current_idx < 200 and self.lap_count == 1:
        #     self.SPEED_MAX = 20# - 0.12 * (self.current_idx - 100) #20
        #     self.braking_a = -10.0 + 0.07 * (self.current_idx - 100) #-5.0
        #     self.sus_b = 3.25 - 0.01 * (self.current_idx - 100)
        #     if self.current_idx < 100:
        #         # print('in')
        #         self.SPEED_MAX = 20 #20
        #         self.braking_a = -10.0 #-5.0
        #         self.sus_b = 3.25 #3.25

        # elif self.current_idx >= 200 and self.current_idx <= 850:  # 350
        #     self.SPEED_MAX = 20 #20
        #     self.braking_a = -3.0 #-5.0
        #     self.sus_b = 2.25 #3.25

        # elif self.current_idx >= 850 and self.current_idx <= 950:  # 350
        #     self.SPEED_MAX = 20 - 0.12 * (self.current_idx - 850) #20
        #     self.braking_a = -3.0 + 0.018 * (self.current_idx - 850) #-5.0
        #     self.sus_b = 2.25 - 0.01 * (self.current_idx - 850)
            
        else:
            self.SPEED_MAX = 12 #15
            self.braking_a = -1.1 #-1.8
            self.sus_b = 1.25 #1.25

        if mode == 0:
            # Braking_Distance_based Speed
            braking_speed = self.braking_distance()
            calculated_speed = self.speed_suspension(braking_speed)
        elif mode == 1:
            # Angle_Based Speed
            # calculated_speed = self.angle_based()
            if self.current_speed < 7:
                calculated_speed = self.angle_based()
                print(f"Angular Speed: {calculated_speed}")
            else:
                calculated_speed = self.angle_based(self.current_speed,8)

        return calculated_speed
