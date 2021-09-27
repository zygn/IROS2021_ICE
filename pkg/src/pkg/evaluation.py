import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import random
from pprint import pprint
from colorama import Fore, Style, init
import csv
import copy

init(autoreset=True)
# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# Qualifying Codes...
from pkg.planner.qualifying.fgm_conv import FGM_CONV
# On Project Codes...
from pkg.planner.fgm_overtaking import FGM
from pkg.planner.fgm_progress import FGM_p
# Combined Planner (Driver + SCs)
from pkg.planner.driver import FGM_GNU_CONV as GNU1
from pkg.planner.driver_2 import FGM_GNU_CONV as GNU2
# Test Codes...
from pkg.planner.test.wall import Wall
from pkg.planner.test.example import GapFollower


# choose your racetrack here (SOCHI, SOCHI_OBS)
RACETRACK = 'SOCHI'


def _pack_odom(obs, i):
    keys = {
        'poses_x': 'pose_x',
        'poses_y': 'pose_y',
        'poses_theta': 'pose_theta',
        'linear_vels_x': 'linear_vel_x',
        'linear_vels_y': 'linear_vel_y',
        'ang_vels_z': 'angular_vel_z',
    }
    return {single: obs[multi][i] for multi, single in keys.items()}


class GymRunner(object):

    def __init__(self, racetrack, drivers):
        self.racetrack = racetrack
        self.drivers = drivers
        self.driver_count = len(self.drivers)
        self.env = gym.make('f110_gym:f110-v0',
                            map="{}/maps/{}".format(current_dir, self.racetrack),
                            map_ext=".png", num_agents=len(self.drivers))

        self.poses = None
        self.rendering = False

        self.status = {
            'p1': {
                'name': self.drivers[0].__class__.__name__,
                'collisions': False,
                'x': 0.0,
                'y': 0.0,
                'speed': 0.0,
                'lap_counts': 0.0,
                'lap_times': 0.0,
                'finish': False
            },
            'p2': {
                'name': self.drivers[1].__class__.__name__,
                'collisions': False,
                'x': 0.0,
                'y': 0.0,
                'speed': 0.0,
                'lap_counts': 0.0,
                'lap_times': 0.0,
                'finish': False
            }
        }
        self.driver_init()

    def driver_init(self):

        if self.driver_count == 1:
            self.poses = np.array([[0.8007017, -0.2753365, 4.1421595]])
        elif self.driver_count == 2:
            self.poses = np.array([
                [0.8007017, -0.2753365, 4.1421595],
                [0.8162458, 1.1614572, 4.1446321],
            ])
        else:
            raise ValueError("Max 2 drivers are allowed")

    def update_status(self, obs, step_reward, done, info):

        for i in range(0, self.driver_count):
            self.status[f'p{i + 1}']['x'] = obs['poses_x'][i]
            self.status[f'p{i + 1}']['y'] = obs['poses_y'][i]
            self.status[f'p{i + 1}']['finish'] = info['checkpoint_done'][i]
            if self.status[f'p{i + 1}']['collisions'] == 1.0:
                pass
            else:
                self.status[f'p{i + 1}']['collisions'] = obs['collisions'][i]
            self.status[f'p{i + 1}']['lap_times'] = obs['lap_times'][i]
            self.status[f'p{i + 1}']['lap_counts'] = obs['lap_counts'][i]

    def finish_check(self, obs, done):
        if obs['lap_counts'][0] >= 2 and obs['lap_counts'][1] >= 2:
            return True
        elif np.max(obs['lap_times']) >= 300.0:
            return True
        elif obs['lap_counts'][0] >= 2 or obs['lap_counts'][1] >= 2:
            if abs(obs['lap_counts'][0] - obs['lap_counts'][1]) > 2:
                return True 
            else:
                return False
        else:
            return False

    def render(self):
        if self.rendering:
            self.env.render('human_fast')

    def reset(self):
        return self.env.reset(poses=self.poses)

    def change_drivers(self):
        pop_ = self.drivers.pop()
        self.drivers.append(pop_)

    def run(self):

        obs, step_reward, done, info = self.reset()
        self.render()
        laptime = 0.0
        start = time.time()
 
        while not done:
            actions = []
            futures = []
            with concurrent.futures.ThreadPoolExecutor() as executor:
                odom_0, odom_1 = _pack_odom(obs, 0), None
                if len(drivers) > 1:
                    odom_1 = _pack_odom(obs, 1)

                for i, driver in enumerate(drivers):
                    if i == 0:
                        ego_odom, opp_odom = odom_0, odom_1
                    else:
                        ego_odom, opp_odom = odom_1, odom_0
                    scan = obs['scans'][i]
                    if hasattr(driver, 'process_observation'):
                        futures.append(executor.submit(driver.process_observation, ranges=scan, ego_odom=ego_odom))
                    elif hasattr(driver, 'process_lidar'):
                        futures.append(executor.submit(driver.process_lidar, scan))

            for future in futures:
                speed, steer = future.result()
                actions.append([steer, speed])
            actions = np.array(actions)
            obs, step_reward, done, info = self.env.step(actions)
            laptime += step_reward
            # self.status_print(obs, step_reward, done, info)
            self.render()
            self.update_status(obs, step_reward, done, info)
            done = self.finish_check(obs, done)
            pprint([obs, info])

        return self.status


if __name__ == '__main__':

    file_name = f'log/eval_{time.strftime("%Y-%m-%d-%H-%M-%S")}.csv'
    print(f"Result will logged at {file_name}")
    csvfile = open(file_name, 'w', newline='')
    wdr = csv.writer(csvfile)
    wdr.writerow(['1p', 'laptime', 'max_speed', 'finish', 'collisions', 'x', 'y', '2p', 'laptime', 'max_speed', 'finish', 'collisions', 'x', 'y'])

    speed_params = np.linspace(10, 12, 100)
    params = {
                'debug': False,
                'speed_controller': 0,
                'racecar_length': 0.3302,
                'robot_scale': 0.2032,
                'max_speed': 15.0,
                'min_speed': 5.0,
                'pi': 3.141592,
                'mu': 0.523,
                'g': 9.81,
                'look': 2.5,
                'threshold': 4.5,
                'gap_size': 1,
                'filter_scale': 1.1,
                'gap_theta_gain': 20.0,
                'ref_theta_gain': 1.5,
                'best_point_conv_size': 120,
                'sus_a': 1.25,
                'sus_b': 0.522222222222222,
                'braking_a': -2.0,
                'braking_b': 1.05555555555556,
                'waypoint_delim': ',',
                # 'waypoint_path': '/catkin_ws/src/pkg/src/pkg/SOCHI_for_pp.csv'  # for ROS ENVIRONMENT
                'waypoint_path': 'pkg/SOCHI_for_pp.csv'  # for Python main ENVIRONMENT
            }
    params_2 = copy.copy(params)

    for i in range(0, 100):
        default_speed = params['max_speed']
        params_2['max_speed'] = random.choice(speed_params)

        # drivers = [FGM_p(), Wall(params_2['max_speed'])]
        drivers = [FGM_p(), GapFollower(params_2['max_speed'])]
        print(f"Processing {i}-0 : {drivers[0].__class__.__name__} vs {drivers[1].__class__.__name__}(MAX_SPEED: {params_2['max_speed']})")

        runner1 = GymRunner(RACETRACK, drivers)
        val = runner1.run()

        csv_val = [drivers[0].__class__.__name__, val['p1']['lap_times'], default_speed, val['p1']['finish'], val['p1']['collisions'], val['p1']['x'], val['p1']['y'], drivers[1].__class__.__name__, val['p2']['lap_times'], params_2['max_speed'], val['p2']['finish'], val['p2']['collisions'], val['p2']['x'], val['p2']['y']]
        wdr.writerow(csv_val)
        csv_val = []

        # drivers = [Wall(params_2['max_speed']), FGM_p()]
        drivers = [GapFollower(params_2['max_speed']), FGM_p()]
        print(f"Processing {i}-1 : {drivers[0].__class__.__name__}(MAX_SPEED: {params_2['max_speed']}) vs {drivers[1].__class__.__name__}")
        
        runner2 = GymRunner(RACETRACK, drivers)
        val = runner2.run()

        csv_val = [drivers[0].__class__.__name__, val['p1']['lap_times'], params_2['max_speed'], val['p1']['finish'], val['p1']['collisions'], val['p1']['x'], val['p1']['y'], drivers[1].__class__.__name__, val['p2']['lap_times'], default_speed, val['p2']['finish'], val['p2']['collisions'], val['p2']['x'], val['p2']['y']]
        wdr.writerow(csv_val)
        csv_val = []

    csvfile.close()