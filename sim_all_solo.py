import gym
import time
import yaml
import csv
import numpy as np
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

    if conf.debug['logging']:
        file_name = f'log/all_sc{conf.speed_controller}_{time.strftime("%Y-%m-%d-%H-%M-%S")}.csv'
        print(f"Result will logged at {file_name}")
        csvfile = open(file_name, 'w', newline='')
        wdr = csv.writer(csvfile)
        wdr.writerow(["algorithm", "laptime", "scores", "finish", "collision", "collision_reset", "max_speed", "avg_speed"])

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))

    if conf.debug['gui_render']:
        env.render()

    for pln in [FGM_CONV, FGM_GNU, FGM_GNU_CONV, FGM_PP_V1, FGM_PP_V2, FGM_STECH, FGM_STECH_CONV, PP]:
        planner = pln(conf)
        laptime = 0.0
        start = time.time()
        done_i = 0
        collision = False
        score = 0.0

        plot_interval = 0
        plot_list = []
        data_list = []
        print(f"Algorithm: {planner.__class__.__name__}")

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

            if conf.debug['gui_render']:
                env.render(mode='human_fast')

            env_speed = obs['linear_vels_x'][0]
            pln_speed = speed
            plot_interval += 1
            plot_list.append([env_speed, pln_speed])
            data_list.append(env_speed)

            if conf.debug['plotting'] and plot_interval % 10 == 0:
                plt.title(f"Env Speed: {env_speed}")
                plt.grid()
                plt.plot(plot_list)
                plt.legend(['Env Speed', 'Planner Speed'])
                plt.pause(0.005)
                plt.clf()

            if len(plot_list) >= 1000:
                del(plot_list[0])

            if done and not info['checkpoint_done'][0]:
                collision = True
                if done_i < conf.debug['collision_interval'] and conf.debug['collision_reset']:
                    done_i += 1
                    real_elapsed_time = time.time() - start
                    max_speed = np.max(data_list)
                    avg_speed = np.mean(data_list)

                    print(f"\tEnvironment Max Speed: {max_speed}")
                    print(f"\tEnvironment Average Speed: {avg_speed}")
                    print(f"\tSim elasped time: {laptime}")
                    print(f"\tReal elasped time: {real_elapsed_time}")
                    print(f"\tCollision: {collision}")
                    print(f"\tCollision Count: {done_i}")
                    print("\t\tReset Environment..!\n\n")

                    # Reset Environment!
                    obs, step_reward, done, info = env.reset(np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))
                    laptime = 0.0
                    start = time.time()
                    collision = False
                    plot_interval = 0
                    plot_list = []
                    data_list = []

        real_elapsed_time = time.time()-start
        max_speed = np.max(data_list)
        avg_speed = np.mean(data_list)
        score = round(100/laptime, 4)
        if conf.debug['logging']:
            wdr.writerow([planner.__class__.__name__, laptime, score, done, collision, done_i, max_speed, avg_speed])

        print(f"\tEnvironment Max Speed: {max_speed}")
        print(f"\tEnvironment Average Speed: {avg_speed}")
        print(f"\tSim elasped time: {laptime}")
        print(f"\tReal elasped time: {real_elapsed_time}")
        print(f"\tSubmission Scores: {score}")
        print(f"\tCollision: {collision}\n\n")

        # 다음 플래너로 넘어가면서 env reset.
        obs, step_reward, done, info = env.reset(np.array([[conf.p1['sx'], conf.p1['sy'], conf.p1['stheta']]]))

    if conf.debug['logging']:
        csvfile.close()