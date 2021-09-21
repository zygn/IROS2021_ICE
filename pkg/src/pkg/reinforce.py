import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import csv
from tqdm import tqdm
# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# import your drivers here
from pkg.drivers import FGM_GNU_CONV as Driver

# choose your racetrack here (SOCHI, SOCHI_OBS)


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
        elif driver_count == 2:
            poses = np.array([
                [0.8007017, -0.2753365, 4.1421595],
                [0.8162458, 1.1614572, 4.1446321],
            ])
        else:
            raise ValueError("Max 2 drivers are allowed")

        obs, step_reward, done, info = env.reset(poses=poses)

        laptime = 0.0

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
            obs, step_reward, done, info = env.step(actions)
            laptime += step_reward


        return laptime, info['checkpoint_done'][0]


if __name__ == '__main__':

    file_name = f'log/reinforce_{time.strftime("%Y-%m-%d-%H-%M-%S")}.csv'
    print(f"Result will logged at {file_name}")
    csvfile = open(file_name, 'w', newline='')
    wdr = csv.writer(csvfile)
    wdr.writerow(["laptime", "obs_laptime", "obs_done", "laptime", "done", "gap_size", "conv_size", "braking_a", "sus_b"])


    params = {
        'racecar_length': 0.3302,
        'robot_length': 0.3302,
        'robot_scale': 0.2032,
        'max_speed': 15.0,
        'min_speed': 5.0,
        'pi': 3.141592,
        'mu': 0.523,
        'g': 9.81,
        'look': 2.0,
        'threshold': 6.0,
        'gap_size': 1,
        'filter_scale': 1.1,
        'gap_theta_gain': 20.0,
        'ref_theta_gain': 1.5,
        'best_point_conv_size': 80,
        'sus_a': 0.3,
        'sus_b': 0.511111,
        'braking_a': 0.0,
        'braking_b': 1.0,
        'waypoint_delim': ',',
        # 'waypoint_path': '/catkin_ws/src/pkg/src/pkg/SOCHI_for_pp.csv'  # for ROS ENVIRONMENT
        'waypoint_path': 'pkg/SOCHI_for_pp.csv'  # for Python main ENVIRONMENT
    }

    gap_size_range = np.linspace(20, 60, 10)
    conv_size_range = np.linspace(60, 160, 10)
    braking_a_range = np.linspace(2, 0, 10)
    sus_b_range = np.linspace(0.2, 0.9, 8)

    for gap_size in gap_size_range:
        for conv_size in conv_size_range:
            for brk_a in braking_a_range:
                for sus_b in sus_b_range:

                    RACETRACK = 'SOCHI_OBS'
                    params['gap_size'] = gap_size
                    params['best_point_conv_size'] = int(conv_size)
                    params['braking_a'] = brk_a
                    params['sus_b'] = sus_b

                    print(f"Processing: {gap_size}|{int(conv_size)}|{brk_a}|{sus_b}", end="")

                    drivers = [Driver(params)]
                    runner = GymRunner(RACETRACK, drivers)
                    obs_laptime, obs_done = runner.run()

                    if obs_done == True:
                        RACETRACK = 'SOCHI'
                        runner = GymRunner(RACETRACK, drivers)
                        laptime, done = runner.run()
                        if done == True:  # Non-OBS, OBS Passed
                            print(f"\rProcessing: {gap_size}|{int(conv_size)}|{brk_a}|{sus_b}...Passed")
                            wdr.writerow([laptime, obs_laptime, obs_done, laptime, done, gap_size, int(conv_size), brk_a, sus_b])
                        else:
                            print(f"\rProcessing: {gap_size}|{int(conv_size)}|{brk_a}|{sus_b}...Failed")  # Non-OBS Failed, OBS Passed
                    else:
                        print(f"\rProcessing: {gap_size}|{int(conv_size)}|{brk_a}|{sus_b}...Failed")
                        done = False  # Failed
                        laptime = None

                    print(f"\tSOCHI_OBS: {obs_done}, {obs_laptime}")
                    print(f"\tSOCHI: {done}, {laptime}")


    csvfile.close()