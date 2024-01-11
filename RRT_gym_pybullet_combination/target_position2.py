"""
Script that combines everything
"""


import time
import argparse
import numpy as np
from scipy import interpolate
from Smoothpath import smooth_path, plot_smoothed_path
from RTT_star import pathSearch, parse_urdf
import pybullet as p

from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import DroneModel, Physics
# from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from RRT_gym_pybullet_combination.aviaries.CustomAviary import CustomAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
# from gym_pybullet_drones.utils.Logger import Logger


DEFAULT_DRONE = DroneModel('cf2p')
DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False




def run(
        drone=DEFAULT_DRONE,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VIDEO,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        plot=True,
        colab=DEFAULT_COLAB
    ):

    #### Define obstacles and waypoints ########################
    startpos = (0., 0., 4.)
    endpos = (3., 3., 0.)
    urdf_path = "../RRT_gym_pybullet_combination/obstacles/random_rubble2.urdf"
    obstacles = parse_urdf(urdf_path)
    n_iter = 200
    radius = 1.5
    stepSize = 0.7


    waypoints = pathSearch(startpos, endpos,obstacles, n_iter, radius, stepSize)


    path_smooth = smooth_path(waypoints)
    # plot_smoothed_path(waypoints, path_smooth)


    #### Initialize the simulation #############################
    INIT_XYZS = np.array([path_smooth[0]])
    env = CustomAviary(drone_model=drone,
                     num_drones=1,
                     initial_xyzs=INIT_XYZS,
                     physics=Physics.PYB_DW,
                     neighbourhood_radius=10,
                     pyb_freq=simulation_freq_hz,
                     ctrl_freq=control_freq_hz, #the frequency at which control commands are applied to the simulated drones or agents within the environment?
                     gui=gui,
                     record=record_video,
                     obstacles=True
                     )

    # obstacle_id = p.loadURDF("../RRT_gym_pybullet_combination/obstacles/box2.urdf")
    # position = [0,0,0]
    # orientation = [0,0,0,1]
    # p.resetBasePositionAndOrientation(obstacle_id, position, orientation)


    #### Initialize the trajectories ###########################
    PERIOD = 30
    NUM_WP = control_freq_hz*PERIOD
    TARGET_POS = np.zeros((NUM_WP, 3))




    interp_func = interpolate.interp1d(
        np.linspace(0, 1, num=len(path_smooth)),
        path_smooth,
        axis=0,
        kind='linear'
    )
    new_indices = np.linspace(0, 1, num=NUM_WP)
    # print(path_smooth)
    TARGET_POS = interp_func(new_indices)
    # print(TARGET_POS)
    # TARGET_POS = TARGET_POS[:NUM_WP]

    wp_counter = 0  #As it's only a single drone, no need to keep multiple counters



    #### Initialize the logger #################################
    # logger = Logger(logging_freq_hz=control_freq_hz,
    #                 num_drones=1,
    #                 duration_sec=duration_sec,
    #                 output_folder=output_folder,
    #                 colab=colab
    #                 )

    #### Initialize the controllers ############################
    ctrl = DSLPIDControl(drone_model=drone)

    #### Run the simulation ####################################
    action = np.zeros((1,4))
    START = time.time()
    for i in range(0, NUM_WP):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        action[0, :], _, _ = ctrl.computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[0],
                                                                target_pos=TARGET_POS[wp_counter],
                                                                )

        #### Go to the next way point and loop #####################
        wp_counter = (wp_counter + 1) if wp_counter < (NUM_WP - 1) else 0

        #### Log the simulation ####################################
        # logger.log(drone=0,
        #             timestamp=i/env.CTRL_FREQ,
        #             state=obs[0],
        #             control=np.hstack([TARGET_POS[wp_counter], np.zeros(9)])
        #             )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    # logger.save()
    # logger.save_as_csv("dw") # Optional CSV save

    #### Plot the simulation results ###########################
    # if plot:
    #     logger.plot()


if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Downwash example script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONE,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 10)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))