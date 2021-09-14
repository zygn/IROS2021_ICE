import numpy as np

class SpeedController:
    def __init__ (self, params):
        self.params = params
        self.scan = []
        self.current_speed = 0.0
        self.steering = 0.0
        self.current_idx = 0

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.WHEEL_BASE = params.robot_length
        self.SPEED_MAX = params.max_speed

        self.wps, self.wp_num = self.load_wps()
    
    def load_wps(self):
        wpt_path = self.params.wpt_path
        wpt_delimiter = self.params.wpt_delimeter

        file_wps = np.genfromtxt(wpt_path, delimiter=wpt_delimiter, dtype='float')

        temp_waypoint = []
        wp_num = 0
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            wp_num += 1

        return temp_waypoint, wp_num
    
    def get_distance(self, origin, target):
        dx = origin[0] - target[0]
        dy = origin[1] - target[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def find_desired_wp(self):
        look_const = 2.0

        wp_index = self.current_idx
        wp_target = self.current_idx + 1

        temp_distance = 0

        while True:
            if wp_target >= self.wp_num - 1:
                wp_target = 0
            temp_distance = self.get_distance(self.wps[wp_target], self.wps[wp_index])
            if temp_distance > look_const: break
            wp_target += 1

        return wp_target

    def braking_based(self):
        current_distance = np.fabs(np.average(self.scan[499:580]))

        if np.isnan(current_distance):
            print("SCAN ERROR")
            current_distance = 1.0
        
        if self.current_speed> 10:
            current_distance -= self.current_speed * 0.7
        
        maximum_speed = np.sqrt(2 * self.MU * self.GRAVITY_ACC * np.fabs(current_distance)) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX
        
        return maximum_speed
    
    def road_direction_based(self):
        current_target = self.wps[self.current_idx]
        next_idx = self.find_desired_wp()
        next_target = self.wps[next_idx]

        dx = current_target[0] - next_target[0]
        dy = current_target[1] - next_target[1]
        
        road_direction = np.fabs(np.arctan2(dy, dx))
        return road_direction
        
    def speed_supensor(self, set_speed):
        final_speed = 0
        if self.current_speed <= set_speed:
            if self.current_speed >= 10:
                final_speed = set_speed
            else:
                final_speed = self.current_speed + np.fabs((set_speed - self.current_speed) * self.WHEEL_BASE)
        else:
            final_speed = self.current_speed - np.fabs((set_speed - self.current_speed) * 0.2)
        
        return final_speed

    def routine(self, scan, speed, steer, idx):
        self.scan = scan
        self.current_speed = speed
        self.steering = steer
        self.current_idx = idx
        
        b_speed = self.braking_based()
        road_direction = self.road_direction_based()

        if np.fabs(road_direction) > self.PI / 8 or np.fabs(self.steering) > self.PI:
            set_speed = b_speed
        else:
            set_speed = (float)(-3/self.PI)*(b_speed - self.current_speed)*np.fabs(road_direction)+b_speed
            set_speed = np.fabs(set_speed)
        
        calculated_speed = self.speed_supensor(set_speed)
        
        return calculated_speed

