import numpy as np

class SpeedController:
    def __init__(self):

        self.mode = 0
        self.MU = 0.523
        self.GRAVITY_ACC = 9.81
        self.PI = 3.141592
        self.WHEEL_BASE = 0.3302
        self.SPEED_MAX = 15.0 # 15.0
        self.SPEED_MIN = 5.0

        self.scan = []
        self.current_speed = 5.0
        self.steering_angle = 0.0
        self.current_idx = 0
        #%
        # lap_time: 78.76 braking_a: -2.0 sus_b: 1.25
        self.braking_a = -2.0#-1.591111111111111111#-0.611111111111111 #-0.611111111111111 
        self.braking_b = 1.05555555555556 #1.0
        self.sus_a = 0.522222222222222 #0.688889 #0.3 # 0.3
        self.sus_b = 1.25#0.961111111111111 #0.511111 #0.511111

        self.wpt_path = 'pkg/SOCHI_for_pp.csv'
        # self.wpt_path = '/catkin_ws/src/pkg/src/pkg/SOCHI_for_pp.csv'
        self.wpt_delimeter = ','

        self.wps, self.wp_num = self.load_wps()

    def const_speed(self):
        speed_straight = 14
        speed_corner = 6
        straight_steer = np.pi / 18

        if np.abs(self.steering_angle) > straight_steer:
            const_speed = speed_corner
        else:
            const_speed = speed_straight

        return const_speed

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

    # Road Direction Based Speed Control
    def load_wps(self):
        wpt_path = self.wpt_path
        wpt_delimiter = self.wpt_delimeter

        file_wps = np.genfromtxt(wpt_path, delimiter=wpt_delimiter, dtype='float')

        temp_waypoint = []
        wp_num = 0
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            wp_num += 1

        return temp_waypoint, wp_num

    def get_distance(self, origin, target):
        _dx = origin[0] - target[0]
        _dy = origin[1] - target[1]

        _res = np.sqrt(_dx ** 2 + _dy ** 2)

        return _res

    def find_next_target_wp(self):
        look_const = 2.0

        current_idx = self.current_idx
        wp_target = self.current_idx + 1

        temp_distance = 0
        while True:
            if wp_target >= self.wp_num - 1:
                wp_target = 0

            temp_distance = self.get_distance(self.wps[wp_target], self.wps[current_idx])

            if temp_distance > look_const: break
            wp_target += 1

        return wp_target

    def find_road_direction(self):
        current_point = self.wps[self.current_idx]
        next_idx = self.find_next_target_wp()
        target_point = self.wps[next_idx]

        dx = current_point[0] - target_point[0]
        dy = current_point[1] - target_point[1]

        road_direction = np.arctan2(dy, dx)

        return road_direction

    def direction_speed(self):
        current_distance = np.fabs(np.average(self.scan[499:580]))
        direction_speed = 0
        road_direction = np.fabs(self.find_road_direction())

        braking_speed = self.braking_distance()

        if current_distance < 5:
            if self.current_speed < 9:
                direction_speed = self.angle_based()
            else:
                direction_speed = self.angle_based(self.SPEED_MAX, self.current_speed)
            # direction_speed = self.speed_suspension(angle_speed)
        elif current_distance < 10:
            direction_speed = self.speed_suspension(braking_speed)
        else:
            # direction_speed = float(-(3 / self.PI) * (braking_speed - self.current_speed) * np.fabs(road_direction) + braking_speed)
            direction_speed = self.speed_suspension(braking_speed)

        return direction_speed

    def speed_suspension(self, set_speed):
        final_speed = 0
        if self.current_speed <= set_speed:
            final_speed = set_speed
            # if self.current_speed >= 10:
            #     final_speed = set_speed
            # else:
            #     # sus_a
            #     final_speed = self.current_speed + np.fabs((set_speed - self.current_speed) * self.sus_a)
        else:
            # sus_b
            final_speed = self.current_speed - np.fabs((set_speed - self.current_speed) * self.sus_b)

        return final_speed

    def routine(self, scan, speed, steer, idx):
        calculated_speed = 0

        self.scan = scan
        self.current_speed = speed
        self.steering_angle = steer
        self.current_idx = idx
        #%
        # lap_time: 78.55
        if self.current_idx < 200 or (self.current_idx > 350 and self.current_idx < 600):
            print(self.current_idx)
            self.SPEED_MAX = 20 #20
            self.braking_a = -5.0 #-5.0
            self.sus_b = 3.25 #3.25
        elif self.current_idx >= 200 and self.current_idx <= 350:
            self.SPEED_MAX = 20 #20
            self.braking_a = -3.0 #-5.0
            self.sus_b = 2.25 #3.25
        else:
            self.SPEED_MAX = 20 #15
            self.braking_a = -1.8 #-1.8
            self.sus_b = 1.25 #1.25

        if self.mode == 0:
            # Const Speed
            calculated_speed = self.const_speed()
        elif self.mode == 1:
            # Braking_Distance_based Speed
            braking_speed = self.braking_distance()
            calculated_speed = self.speed_suspension(braking_speed)
        elif self.mode == 2:
            # Angle_Based Speed
            calculated_speed = self.angle_based()
        elif self.mode == 3:
            # Braking_distance + Road Direction based Speed
            calculated_speed = self.direction_speed()

        return calculated_speed

class FGM_CONV:
    BUBBLE_RADIUS = 200
    PREPROCESS_CONV_SIZE = 100  # PREPROCESS_consecutive_SIZE
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self):
        self.radians_per_elem = None
        
        self.current_speed = 2.0
        self.GRAVITY_ACC = 9.81
        self.MU = 0.523
        self.ROBOT_LENGTH = 0.3302

        self.speed_controller = SpeedController()
        self.speed_past = 0


    def preprocess_lidar(self, ranges):

        self.radians_per_elem = (2 * np.pi) / len(ranges)
        proc_ranges = np.array(ranges[180:-180])  # scan range 180 deg
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)  # for checking error.
        return proc_ranges

    def find_max_gap(self, free_space_ranges):

        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):

        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):

        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2

        return steering_angle

    def driving(self, scan_data, odom_data):
        ranges = scan_data
        proc_ranges = self.preprocess_lidar(ranges)
        closest = proc_ranges.argmin()
        
        self.current_speed = odom_data['linear_vel_x']
        self.scan_filtered = scan_data

        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)

        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        steering_angle = self.get_angle(best, len(proc_ranges))
        speed = self.speed_controller.routine(scan_data, self.current_speed, steering_angle, 0)
        self.speed_past = speed

        return speed, steering_angle

    def process_observation(self, ranges, ego_odom):
        if ego_odom:
            return self.driving(ranges, ego_odom)