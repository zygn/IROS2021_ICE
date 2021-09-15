import numpy as np
import time


class PP:
    def __init__(self, params):

        self.RACECAR_LENGTH = params.robot_length
        self.ROBOT_LENGTH = params.robot_length
        self.SPEED_MAX = params.max_speed
        self.SPEED_MIN = params.min_speed

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.ROBOT_SCALE = params.robot_scale
        self.waypoint_real_path = params.wpt_path
        self.wp_num = 1
        self.waypoint_delimeter = params.wpt_delimeter
        self.waypoints = self.get_waypoint()



        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0
        self.lookahead_desired = 0
        self.desired_point = 0
        self.actual_lookahead = 0
        self.CURRENT_WP_CHECK_OFFSET= 2
        self.transformed_desired_point = 0

        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0

        self.current_speed = 5.0
        self.scan_filtered = []
        self.scan_range = 0

    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter = self.waypoint_delimeter, dtype='float')

        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        # print("wp_num",self.wp_num)
        return temp_waypoint


    def speed_controller(self):
        current_distance = np.fabs(np.average(self.scan_filtered[499:580]))
        if np.isnan(current_distance):
            print("SCAN ERR")
            current_distance = 1.0

        if self.current_speed > 10:
            current_distance -= self.current_speed * 0.7

        maximum_speed = np.sqrt(2 * self.MU * self.GRAVITY_ACC * current_distance) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX

        if self.current_speed <= maximum_speed:
            # ACC
            if self.current_speed >= 10:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.8)
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * self.ROBOT_LENGTH)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)
        # print("speed :", set_speed, "current", maximum_speed)
        return set_speed

    def find_nearest_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp += 1
            if wp_index_temp >= len(self.waypoints) - 1:
                wp_index_temp = 0

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif temp_distance > (self.nearest_distance + self.CURRENT_WP_CHECK_OFFSET) or (
                    wp_index_temp == self.wp_index_current):
                break

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[self.wp_index_current])
        if (transformed_nearest_point[0] < 0): self.nearest_distance *= -1

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        while (1):
            if (wp_index_temp >= len(self.waypoints) - 1): wp_index_temp = 0
            distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)
            if distance >= self.lookahead_desired:
                if wp_index_temp - 2 >= 0 and wp_index_temp + 2 < len(self.waypoints) - 1:
                    self.waypoints[wp_index_temp][2] = np.arctan(
                        (self.waypoints[wp_index_temp + 2][1] - self.waypoints[wp_index_temp - 2][1]) /
                        self.waypoints[wp_index_temp + 2][0] - self.waypoints[wp_index_temp - 2][0])
                self.desired_point = self.waypoints[wp_index_temp]
                self.actual_lookahead = distance
                break
            wp_index_temp += 1

    def get_lookahead_desired(self):
        _vel = self.current_speed
        # self.lookahead_desired = 0.5 + (0.3 * _vel)
        self.lookahead_desired = 1.0 + (0.3 * _vel)

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

    def find_path(self):
        #right cornering
        if self.transformed_desired_point[0] > 0:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/(2*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = -1

        #left cornering
        else:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/((-2)*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = 1

    def setSteeringAngle(self):
        steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius)

        steer = self.steering_direction * steering_angle
        return steer

    def driving(self, scan_data, odom_data):
        """

        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """

        self.scan_range = len(scan_data)
        self.scan_filtered = [0] * self.scan_range
        for i in range(self.scan_range):
            self.scan_filtered[i] = scan_data[i]

        self.current_position = [odom_data['x'], odom_data['y'], odom_data['theta']]
        self.current_speed = odom_data['linear_vel']

        self.find_nearest_wp()
        self.get_lookahead_desired()
        self.find_desired_wp()
        self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)
        self.find_path()
        steer = self.setSteeringAngle()
        speed = self.speed_controller()



        return speed, steer
