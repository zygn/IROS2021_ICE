import gym
import time
import yaml
import numpy as np
import csv
import copy
from matplotlib import pyplot as plt
from argparse import Namespace

from planner.fgm_conv import FGM_CONV
from planner.fgm_gnu import FGM_GNU
from planner.fgm_gnu_conv import FGM_GNU_CONV
from planner.fgm_pp_v1 import FGM_PP_V1
from planner.fgm_pp_v2 import FGM_PP_V2
from planner.fgm_stech import FGM_STECH
from planner.fgm_stech_conv import FGM_STECH_CONV
from planner.pp import PP

if __name__ == '__main__':

    with open('sim_params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    # we will find best parameter

    conf_temp = copy.copy(conf)

    max_speed_range = np.linspace(15, 20, 10)
    sus_a_range = np.linspace(0.3, 0.8, 10)
    sus_b_range = np.linspace(0.2, 0.9, 10)
    braking_a_range = np.linspace(-2, 0, 10)
    braking_b_range = np.linspace(1, 1.5, 10)


    if conf.debug['logging']:
        file_name = f'log/reinforce_{time.strftime("%Y-%m-%d-%H-%M-%S")}.csv'
        print(f"Result will logged at {file_name}")
        csvfile = open(file_name, 'w', newline='')
        wdr = csv.writer(csvfile)
        wdr.writerow(["algorithm", "laptime", "finish", "collision", "max_speed", "sus_a", "sus_b", "braking_a", "braking_b"])

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))

    done_i = 0
    for max_speed in max_speed_range:
        for sus_a in sus_a_range:
            for sus_b in sus_b_range:
                for brk_a in braking_a_range:
                    for brk_b in braking_b_range:
                        done_i += 1

                        conf_temp.max_speed = max_speed
                        conf_temp.sus_a = sus_a
                        conf_temp.sus_b = sus_b
                        conf_temp.braking_a = brk_a
                        conf_temp.braking_b = brk_b

                        print(f"{done_i} Times Situation Start...")
                        print(f"Constants:")
                        print(f"\tMax Speed: {max_speed}")
                        print(f"\tSuspension_Alpha: {sus_a}")
                        print(f"\tSuspension_Beta: {sus_b}")
                        print(f"\tBraking_Alpha: {brk_a}")
                        print(f"\tBraking_Beta: {brk_b}")

                        planner = FGM_GNU_CONV(conf_temp)

                        laptime = 0.0
                        plot_list = []
                        data_list = []

                        while not done:
                            scan_data = obs['scans'][0]
                            odom_data = {
                                'x': obs['poses_x'][0],
                                'y': obs['poses_y'][0],
                                'theta': obs['poses_theta'][0],
                                'linear_vel': obs['linear_vels_x'][0]
                            }

                            speed, steer = planner.driving(scan_data, odom_data)
                            obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
                            laptime += step_reward
                            collision = True if obs['collisions'][0] == 1. else False

                            if conf.debug['gui_render']:
                                env.render(mode='human')

                            env_speed = obs['linear_vels_x'][0]
                            pln_speed = speed

                        avg_speed = np.mean(data_list)

                        if conf.debug['logging']:
                            wdr.writerow([planner.__class__.__name__, laptime, done, collision, max_speed, sus_a, sus_b, brk_a, brk_b])

                        print(f"Result:")
                        print(f"\tSim elasped time: {laptime}")
                        print(f"\tCollision: {collision}\n\n")

                        obs, step_reward, done, info = env.reset(
                            np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))

    if conf.debug['logging']:
        csvfile.close()