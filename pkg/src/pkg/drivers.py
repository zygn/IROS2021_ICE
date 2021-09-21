import numpy as np


class SpeedController:
    def __init__(self):

        self.mode = 1
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

        self.braking_a = 0.0 # 0.0 
        self.braking_b = 1.0
        self.sus_a = 0.3 # 0.3
        self.sus_b = 0.511111

        # self.wpt_path = '/pkg/SOCHI_for_pp.csv'
        self.wpt_path = '/catkin_ws/src/pkg/src/pkg/SOCHI_for_pp.csv'
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
            if self.current_speed <= 10:
                final_speed = set_speed
            else:
                # sus_a
                final_speed = self.current_speed + np.fabs((set_speed - self.current_speed) * self.sus_a)
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

class FGM_GNU_CONV:
    def __init__(self):

        self.RACECAR_LENGTH = 0.3302
        self.ROBOT_LENGTH = 0.3302
        self.SPEED_MAX = 15.0
        self.SPEED_MIN = 5.0

        self.MU = 0.523
        self.GRAVITY_ACC = 9.81
        self.PI = 3.141592
        self.ROBOT_SCALE = 0.2032

        self.LOOK = 2.0
        self.THRESHOLD = 6.0
        self.GAP_SIZE = 1
        self.FILTER_SCALE = 1.1
        self.GAP_THETA_GAIN = 20.0
        self.REF_THETA_GAIN = 1.5

        self.BEST_POINT_CONV_SIZE = 80

        # self.waypoint_real_path = '/pkg/SOCHI_for_pp.csv'
        self.waypoint_real_path = '/catkin_ws/src/pkg/src/pkg/SOCHI_for_pp.csv'
        self.waypoint_delimeter = ','

        self.scan_range = 0
        self.desired_gap = 0
        self.speed_gain = 0
        self.steering_gain = 0
        self.gain_cont = 0
        self.speed_cont = 0
        self.desired_wp_rt = [0, 0]

        self.speed_up = 0

        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0

        self.max_angle = 0
        self.wp_angle = 0
        self.detect_range_s = 299
        self.detect_range_e = 779
        self.gaps = []
        self.for_gap = [0, 0, 0]
        self.for_point = 0

        self.interval = 0.00435  # 1도 = 0.0175라디안

        self.front_idx = 0
        self.theta_for = self.PI / 3
        self.gap_cont = 0

        self.current_speed = 5.0
        self.dmin_past = 0
        self.lap = 0

        self.closest_wp_dist = 0
        self.closest_obs_dist = 0

        self.scan_filtered_data = None 

        self.speed_control = SpeedController()

        # init finished
        print("Initialization Success (Suck-C'ex)")



    def find_nearest_obs(self, obs):
        min_di = 0
        min_dv = 0
        if len(obs) <= 1:
            min_di = 20
            min_dv = 20
        else:
            min_di = self.getDistance(self.current_position, obs[0])
            for i in range(len(obs)):
                _dist = self.getDistance(self.current_position, obs[i])
                if _dist <= min_di:
                    min_di = _dist
                    min_dv = self.getDistance(self.waypoints[self.wp_index_current], obs[i])

        self.closest_obs_dist = min_di
        self.closest_wp_dist = min_dv

    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return np.sqrt(dx ** 2 + dy ** 2)

    def transformPoint(self, origin, target):
        theta = self.PI / 2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta

        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]

        return tf_point

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))

        return rtpoint

    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')

        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        return temp_waypoint
    
    def calc_abs_cord(self):
        incre = 4.71239 / 1080
        theta = (-180+idx) * incre

        x = current_x + r * np.sin(theta)
        y = current_y + r * np.cos(theta)

        abs_cord = [x,y]
        return abs_cord
        

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp += 1

            if wp_index_temp >= self.wp_num - 1:
                wp_index_temp = 0

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif (temp_distance > (self.nearest_distance + self.LOOK * 1.2)) or (
                    wp_index_temp == self.wp_index_current):
                break

        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num - 1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

    def subCallback_scan(self, scan_data):

        self.scan_range = len(scan_data)

        self.front_idx = (int(self.scan_range / 2))

        self.scan_origin = [0] * self.scan_range
        self.scan_filtered = [0] * self.scan_range

        for i in range(self.scan_range):
            self.scan_origin[i] = scan_data[i]
            self.scan_filtered[i] = scan_data[i]

        for i in range(self.scan_range - 1):
            if self.scan_origin[i] * self.FILTER_SCALE < self.scan_filtered[i + 1]:
                unit_length = self.scan_origin[i] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 1
                while j < filter_num + 1:
                    if i + j < self.scan_range:
                        if self.scan_filtered[i + j] > self.scan_origin[i]:
                            self.scan_filtered[i + j] = self.scan_origin[i]
                        else:
                            break
                    else:
                        break
                    j += 1

            elif self.scan_filtered[i] > self.scan_origin[i + 1] * self.FILTER_SCALE:
                unit_length = self.scan_origin[i + 1] * self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i - j > 0:
                        if self.scan_filtered[i - j] > self.scan_origin[i + 1]:
                            self.scan_filtered[i - j] = self.scan_origin[i + 1]
                        else:
                            break
                    else:
                        break
                    j += 1

        return self.scan_filtered

    def find_gap(self, scan):
        self.gaps = []

        i = 0

        while i < self.scan_range - self.GAP_SIZE:

            if scan[i] > self.THRESHOLD:
                start_idx_temp = i
                end_idx_temp = i
                # max_temp = scan[i]
                # max_idx_temp = i

                while ((scan[i] > self.THRESHOLD) and (i + 1 < self.scan_range)):
                    i += 1
                    # if scan[i] > max_temp:
                    #     max_temp = scan[i]
                    #     max_idx_temp = i
                if scan[i] > self.THRESHOLD:
                    i += 1
                end_idx_temp = i

                gap_temp = [0] * 3
                gap_temp[0] = start_idx_temp
                gap_temp[1] = end_idx_temp
                # gap_temp[2] = max_idx_temp
                self.gaps.append(gap_temp)
            i += 1

    def for_find_gap(self, scan):
        self.for_point = (int)(self.theta_for / self.interval)
        # [0] = start_idx, [1] = end_idx

        start_idx_temp = (self.front_idx) - self.for_point
        end_idx_temp = (self.front_idx) + self.for_point

        max_idx_temp = start_idx_temp
        max_temp = scan[start_idx_temp]

        for i in range(start_idx_temp, end_idx_temp):
            if max_temp < scan[i]:
                max_temp = scan[i]
                max_idx_temp = i
        # [0] = start_idx, [1] = end_idx, [2] = max_idx_temp
        self.for_gap[0] = start_idx_temp
        self.for_gap[1] = end_idx_temp
        self.for_gap[2] = max_idx_temp

    # ref - [0] = r, [1] = theta
    def find_best_gap(self, ref):
        num = len(self.gaps)

        if num == 0:
            return self.for_gap
        else:

            step = (int(ref[1] / self.interval))

            ref_idx = self.front_idx + step

            gap_idx = 0

            if self.gaps[0][0] > ref_idx:
                distance = self.gaps[0][0] - ref_idx
            elif self.gaps[0][1] < ref_idx:
                distance = ref_idx - self.gaps[0][1]
            else:
                distance = 0
                gap_idx = 0

            i = 1
            while (i < num):
                if self.gaps[i][0] > ref_idx:
                    temp_distance = self.gaps[i][0] - ref_idx
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                elif self.gaps[i][1] < ref_idx:
                    temp_distance = ref_idx - self.gaps[i][1]
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i

                else:
                    temp_distance = 0
                    distance = 0
                    gap_idx = i
                    break

                i += 1
            # 가장 작은 distance를 갖는 gap만 return
            return self.gaps[gap_idx]

    def find_best_point(self, best_gap):
        averaged_max_gap = np.convolve(self.scan_filtered[best_gap[0]:best_gap[1]], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + best_gap[0]

    def main_drive(self, max_gap):
        self.max_angle = (max_gap - self.front_idx) * self.interval  # (goal[2] - self.front_idx) * self.interval
        self.wp_angle = self.desired_wp_rt[1]

        # range_min_values = [0]*10
        temp_avg = 0
        dmin = 0
        for i in range(10):
            dmin += self.scan_filtered[i]

        dmin /= 10

        i = 0

        while i < self.scan_range - 7:
            j = 0
            while j < 10:
                if i + j > 1079:
                    temp_avg += 0
                else:
                    temp_avg += self.scan_filtered[i + j]
                j += 1

            temp_avg /= 10

            if dmin > temp_avg:
                if temp_avg == 0:
                    temp_avg = dmin
                dmin = temp_avg
            temp_avg = 0
            i += 3

        if dmin == 0:
            dmin = self.dmin_past

        controlled_angle = ((self.GAP_THETA_GAIN / dmin) * self.max_angle + self.REF_THETA_GAIN * self.wp_angle) / (
                self.GAP_THETA_GAIN / dmin + self.REF_THETA_GAIN)
        distance = 1.0
        # path_radius = 경로 반지름
        path_radius = distance / (2 * np.sin(controlled_angle))
        #
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

        steer = steering_angle
        speed = self.speed_control.routine(self.scan_filtered, self.current_speed, steering_angle,
                                           self.wp_index_current)
        self.dmin_past = dmin

        return steer, speed

    def _process_lidar(self, scan_data, odom_data):
        """

        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """

        scan_data = self.subCallback_scan(scan_data)
        self.current_position = [odom_data['pose_x'], odom_data['pose_y'], odom_data['pose_theta']]
        self.current_speed = odom_data['linear_vel_x']
        self.find_desired_wp()
        self.find_gap(scan_data)
        self.for_find_gap(scan_data)

        self.desired_gap = self.find_best_gap(self.desired_wp_rt)
        self.best_point = self.find_best_point(self.desired_gap)

        steer, speed = self.main_drive(self.best_point)

        return speed, steer


    def process_observation(self, ranges, ego_odom):
        if ego_odom:
            return self._process_lidar(ranges, ego_odom)
    
