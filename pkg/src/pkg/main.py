from math import fabs
import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import random
import copy

# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# import your drivers here
from pkg.planner.driver import FGM_GNU_CONV as GNU
from pkg.planner.driver_2 import FGM_GNU_CONV as GNU2

from pkg.planner.fgm_overtaking import FGM
from pkg.planner.fgm_progress import FGM_p



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

    def run(self):
        # load map
        env = gym.make('f110_gym:f110-v0',
                       map="{}/maps/{}".format(current_dir, RACETRACK),
                       map_ext=".png", num_agents=len(drivers))

        # specify starting positions of each agent
        driver_count = len(drivers)
        if driver_count == 1:
            poses = np.array([[0.8007017, -0.2753365, 4.1421595]])
            poses = np.array([[0.8162458, 1.1614572, 4.1446321]])
        elif driver_count == 2:
            poses = np.array([
                [0.8007017, -0.2753365, 4.1421595],
                [0.8162458, 1.1614572, 4.1446321],
            ])
        else:
            raise ValueError("Max 2 drivers are allowed")

        obs, step_reward, done, info = env.reset(poses=poses)
        env.render()
        # print(obs)
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
            # obs, step_reward, _, info = env.step(actions)
            obs, step_reward, done, info = env.step(actions)
            laptime += step_reward

            env.render(mode='human_fast')
            

        print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)


if __name__ == '__main__':
    params_1p = {
                'debug': False,
                'speed_controller': 1,
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


    params_2p = copy.copy(params_1p)
    params_2p['speed_controller'] = 0

    drivers = [FGM_p(params_1p), FGM_p(params_2p)]
    

    print(f"{drivers[0].__class__.__name__} vs {drivers[1].__class__.__name__}")
    runner = GymRunner(RACETRACK, drivers)
    runner.run()
