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


    def get_lookahead_desired(self):
        _vel = self.current_speed
        # self.lookahead_desired = 0.5 + (0.3 * _vel)
        self.lookahead_desired = 1 + (0.3 * _vel)

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

    def preprocess_lidar(self, ranges):

        self.radians_per_elem = 0.00435
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def driving(self, scan_data, odom_data, transformed_desired_point):
        """
        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """

        self.current_position = [odom_data['x'], odom_data['y'], odom_data['theta']]
        self.current_speed = odom_data['linear_vel']

        self.get_lookahead_desired()
        self.transformed_desired_point = transformed_desired_point
        self.find_path()
        steer = self.setSteeringAngle()

        return steer
