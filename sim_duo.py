import gym
import time
import yaml
import numpy as np
import csv
from matplotlib import pyplot as plt
from argparse import Namespace

from planner.fgm_stech import FGM
from planner.fgm_gnu import FGM_GNU
from planner.odg_pf import ODGPF
from planner.odg_gnu import ODGGNU
from planner.fgm_conv import FGM_CONV
from planner.fgm_stech_conv import FGM as FGM_STECH_CONV

if __name__ == '__main__':

    with open('sim_params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)
    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=2)
    env_init = np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']],
                         [conf.p2['sx'], conf.p2['sy'], conf.p2['stheta']]])

    obs, step_reward, done, info = env.reset(env_init)

    if conf.debug['logging']:
        file_name = f'log/log_duo_{time.strftime("%Y-%m-%d-%H-%M-%S")}.csv'
        print(f"Result will logged at {file_name}")
        csvfile = open(file_name, 'w', newline='')
        wdr = csv.writer(csvfile)

    if conf.debug['gui_render']:
        env.render()

    conf_temp = conf
    conf_temp.speed_controller = 1

    planner = [FGM_CONV(conf), FGM_STECH_CONV(conf)]

    laptime = 0.0
    start = time.time()
    done_i = 0

    plot_interval = 0

    data = {
        'p1': {
            'algorithm': planner[0].__class__.__name__,
            'env_speed': [],
            'planner_speed': [],
            'max_speed': float,
            'min_speed': float,
            'avg_speed': float,
            'collision': False
        },
        'p2': {
            'algorithm': planner[1].__class__.__name__,
            'env_speed': [],
            'planner_speed': [],
            'max_speed': float,
            'min_speed': float,
            'avg_speed': float,
            'collision': False
        }
    }

    ax1_plot = []
    ax2_plot = []

    while not done:
        scan_data = {
            'p1': obs['scans'][0],
            'p2': obs['scans'][1]
        }
        odom_data = {
            'p1': {
                'x': obs['poses_x'][0],
                'y': obs['poses_y'][0],
                'theta': obs['poses_theta'][0],
                'linear_vel': obs['linear_vels_x'][0]
            },
            'p2': {
                'x': obs['poses_x'][1],
                'y': obs['poses_y'][1],
                'theta': obs['poses_theta'][1],
                'linear_vel': obs['linear_vels_x'][1]
            }
        }
        collision = obs['collisions']

        p1_speed, p1_steer = planner[0].driving(scan_data['p1'], odom_data['p1'])
        p2_speed, p2_steer = planner[1].driving(scan_data['p2'], odom_data['p2'])

        env_step = np.array([[p1_steer, p1_speed],
                             [p2_steer, p2_speed]])

        obs, step_reward, done, info = env.step(env_step)

        laptime += step_reward

        data['p1']['planner_speed'].append(p1_speed)
        data['p1']['env_speed'].append(obs['linear_vels_x'][0])
        data['p1']['max_speed'] = np.max(data['p1']['env_speed'])
        data['p1']['min_speed'] = np.min(data['p1']['env_speed'])
        data['p1']['avg_speed'] = np.mean(data['p1']['env_speed'])
        data['p1']['collision'] = True if obs['collisions'][0] == 1. else False

        data['p2']['planner_speed'].append(p2_speed)
        data['p2']['env_speed'].append(obs['linear_vels_x'][1])
        data['p2']['max_speed'] = np.max(data['p2']['env_speed'])
        data['p2']['min_speed'] = np.min(data['p2']['env_speed'])
        data['p2']['avg_speed'] = np.mean(data['p2']['env_speed'])
        data['p2']['collision'] = True if obs['collisions'][1] == 1. else False

        ax1_plot.append([obs['linear_vels_x'][0], obs['linear_vels_x'][1]])
        ax2_plot.append([p1_speed, p2_speed])

        if conf.debug['gui_render']:
            env.render(mode='human_fast')

        plot_interval += 1

        if conf.debug['plotting'] and plot_interval % 25 == 0:
            plt.subplot(2, 1, 1)
            plt.grid()
            plt.ylim(top=conf.max_speed)
            plt.title("Environment Speed")
            plt.plot(ax1_plot)
            plt.legend(['P1 Speed', 'P2 Speed'])

            plt.subplot(2, 1, 2)
            plt.grid()
            plt.ylim(top=conf.max_speed)
            plt.title("Planner Speed")
            plt.plot(ax2_plot)
            plt.legend(['P1 Speed', 'P2 Speed'])

            plt.pause(0.005)
            plt.clf()

        if len(ax1_plot) >= 1000 or len(ax2_plot) >= 1000:
            del ax1_plot[0]
            del ax2_plot[0]

    real_elapsed_time = time.time() - start

    if conf.debug['logging']:
        wdr.writerow(["algorithm", "laptime", "finish", "collision", "collision_reset", "max_speed", "avg_speed"])
        wdr.writerow([data['p1']['algorithm'], laptime, done, data['p1']['collision'], done_i, data['p1']['max_speed'], data['p1']['collision']])
        wdr.writerow([data['p2']['algorithm'], laptime, done, data['p1']['collision'], done_i, data['p2']['max_speed'], data['p2']['collision']])
        csvfile.close()

    print(f"Head-to-Head: {data['p1']['algorithm']} vs {data['p2']['algorithm']}")
    print(f"\tSim elasped time: {laptime}")
    print(f"\tReal elasped time: {real_elapsed_time}")

    print(f"Planner 1:")
    print(f"\tEnvironment Max Speed: {data['p1']['max_speed']}")
    print(f"\tEnvironment Average Speed: {data['p1']['avg_speed']}")
    print(f"\tCollision: {data['p1']['collision']}")

    print(f"Planner 2:")
    print(f"\tEnvironment Max Speed: {data['p2']['max_speed']}")
    print(f"\tEnvironment Average Speed: {data['p2']['avg_speed']}")
    print(f"\tCollision: {data['p2']['collision']}")
