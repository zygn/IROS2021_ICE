def find_path(self):
        #right cornering
        if self.transformed_nearest_point[0] > 0:
            self.goal_path_radius = pow(self.LOOK, 2)/(2*self.transformed_nearest_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_nearest_point[1]/self.goal_path_radius)
            self.steering_direction = -1

        #left cornering
        else:
            self.goal_path_radius = pow(self.LOOK, 2)/((-2)*self.transformed_nearest_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_nearest_point[1]/self.goal_path_radius)
            self.steering_direction = 1

def setSteeringAngle(self):
    steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius)

    steer = self.steering_direction * steering_angle
    return steer

def reverse_run(self):
    #near point 거꾸로 갱신시 역주행
    #목표 point 저장하고 pp로 복귀
    if self.wp_index_current != self.wp_index_current_past:
        if self.wp_index_current - self.wp_index_current_past < 0:
            self.reverse_run_cont += 1
        else:
            self.reverse_run_cont = 0

    if self.reverse_run_cont > 10:
        return True

    return False

def main_drive(self, max_gap):

    self.max_angle = (max_gap - self.front_idx) * self.interval #(goal[2] - self.front_idx) * self.interval
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
    if self.reverse_run():
        print('reverse')
        self.find_path()
        steering_angle = self.setSteeringAngle()
    else:
        steering_angle = np.arctan(self.RACECAR_LENGTH / path_radius)

    steer = steering_angle
    speed = self.speed_control.routine(self.scan_filtered, self.current_speed,steering_angle,self.wp_index_current)
    self.dmin_past = dmin
    self.wp_index_current_past = self.wp_index_current

    return steer, speed
