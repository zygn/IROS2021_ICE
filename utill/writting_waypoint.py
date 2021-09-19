import gym
import time
import yaml
import numpy as np
import csv
from matplotlib import pyplot as plt
from argparse import Namespace

# from planner.legacy.odg_pf import ODGPF
from planner.fgm_conv import FGM_CONV
from planner.fgm_gnu_conv import FGM_GNU_CONV

if __name__ == '__main__':

    with open('sim_params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    # conf.map_path = f"{conf.map_path}"
    # conf.wpt_path = f"{conf.wpt_path}"
    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))
    
    if conf.debug['gui_render']:
        env.render()

    pln_list = [FGM_CONV(conf),FGM_GNU_CONV(conf)]
    
    for planner in pln_list:
        obs, step_reward, done, info = env.reset(np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))
        start = time.time()
        file_name = f'waypoint_{time.strftime("%Y-%m-%d-%H-%M-%S")}.csv'
        print(f"Result will logged at {file_name}")
        csvfile = open(file_name, 'w', newline='')
        wdr = csv.writer(csvfile)

        plot_interval = 0
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
            wdr.writerow([obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0]])

            collision = True if obs['collisions'][0] == 1. else False
            done = True if obs['lap_counts'] >= 1. else False

            env.render(mode='human_fast')

        csvfile.close()