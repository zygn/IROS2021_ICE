import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import random
from pprint import pprint
from colorama import Fore, Style, init

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
                'finish': False
            },
            'p2': {
                'name': self.drivers[1].__class__.__name__,
                'collisions': False,
                'x': 0.0,
                'y': 0.0,
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

    def status_print(self, obs, step_reward, done, info):
        def check(val):
            if val == 1. or val == True:
                return Fore.RED + f"True" + Fore.RESET
            elif val == 0. or val == False:
                return f"False"

        for i in range(0, self.driver_count):
            new_str = f"Planner {i}: {self.drivers[i].__class__.__name__}\n"
            new_str += f"\tSpeed: {obs['linear_vels_x'][i]}\n"
            new_str += f"\tPoses: {[obs['poses_x'][i], obs['poses_y'][i]]}\n"
            new_str += f"\tLap Counts: {obs['lap_counts'][i]}\n"
            new_str += f"\tCollision: {check(obs['collisions'][i])}\n"

            print(new_str)

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
            # self.status_print(obs, step_reward, done, info)
            self.render()
            self.update_status(obs, step_reward, done, info)
        return self.status


if __name__ == '__main__':
    speed_params = np.linspace(6, 10, 100)

    for i in range(0, 100):
        speed_p = random.choice(speed_params)
        # FGM_p vs Something
        drivers = [FGM_p(), Wall(speed_p)]
        runner1 = GymRunner(RACETRACK, drivers)
        pprint(runner1.run())

        # Something vs FGM_p
        drivers = [Wall(speed_p), FGM_p()]
        runner2 = GymRunner(RACETRACK, drivers)
        pprint(runner2.run())
