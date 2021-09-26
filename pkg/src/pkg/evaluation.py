import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import random
from pprint import pprint
from colorama import Fore, Style

# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# import your drivers here
from pkg.planner.drivers import FGM_GNU_CONV
from pkg.planner.fgm_conv import FGM_CONV
from pkg.planner.sample import DisparityExtender, GapFollower

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

        self.env = gym.make('f110_gym:f110-v0',
                       map="{}/maps/{}".format(current_dir, self.racetrack),
                       map_ext=".png", num_agents=len(self.drivers))

        self.poses = None
        self.num_agents = len(self.drivers)
        self.rendering = False

        self.driver_init()

    def driver_init(self):
        driver_count = len(self.drivers)
        if driver_count == 1:
            self.poses = np.array([[0.8007017, -0.2753365, 4.1421595]])
        elif driver_count == 2:
            self.poses = np.array([
                [0.8007017, -0.2753365, 4.1421595],
                [0.8162458, 1.1614572, 4.1446321],
            ])
        else:
            raise ValueError("Max 2 drivers are allowed")

    def make_result(self, obs, step_reward, done, info):
        new_str = ""
        if self.num_agents == 1:
            pass
        if self.num_agents == 2:
            for i in range(0,2):
                new_str += f"Planner {i+1}: {self.drivers[i].__class__.__name__}\n"
                new_str += f"\tLap Counts: {obs['lap_counts'][i]}\n"
                new_str += f"\tLap Times: {obs['lap_times'][i]}\n"
                new_str += f"\tCollisions: {obs['collisions'][i]}\n"
                new_str += f"\tCheckpoint Done: {info['checkpoint_done'][i]}\n\n"
        print(Fore.RED + new_str)
        
    def render(self):
        if self.rendering:
            self.env.render()

    def reset(self):
        return self.env.reset(poses=self.poses)

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
            self.render()
        
        self.make_result(obs, step_reward, done, info)
        return {'observation': obs, 'rate': 1/step_reward, 'done': done, 'checkpoint_done': info['checkpoint_done']}


if __name__ == '__main__':
    drivers = [FGM_GNU_CONV(), GapFollower()]
    runner = GymRunner(RACETRACK, drivers)
    result = runner.run()
