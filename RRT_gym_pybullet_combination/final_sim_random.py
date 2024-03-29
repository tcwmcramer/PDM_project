"""
Script that combines everything
"""

import os
import time
import argparse
import numpy as np
import threading
import random
from scipy import interpolate
from Smoothpath import smooth_path, plot_smoothed_path
from RTT_star import pathSearch, all_urdf
import pybullet as p

from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

from aviaries.CustomAviary import CustomAviary
from random_rubble_gen import generate_urdf_files
from pillars_gen import generate_multiple_pillar_urdf_files

# Constants
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
    # --------------------- Generate random rubbles and write to the 'obstacles' folder -----------------------------#

    num_files = 5
    num_cubes = 10
    output_directory = "obstacles"

    os.makedirs(output_directory, exist_ok=True)

    generate_multiple_pillar_urdf_files(num_files, num_cubes, output_directory)

    num_shapes = 10
    size_bounds = "0.2,0.5"
    orientation_bounds = "-1.0,1.0"
    num_runs = 5

    generate_urdf_files(num_runs, num_shapes, size_bounds, orientation_bounds, output_directory)

    # ------------------------------------- Define obstacles and waypoints -------------------------------------#

    startpos = (
        6,
        random.uniform(0, 5),
        random.uniform(0, 4)
    )

    endpos = (
        -1,
        random.uniform(0, 5),
        2
    )
    obstacles = all_urdf()
    n_iter = 200
    radius = 1.5
    stepsize = 0.7

    waypoints = pathSearch(startpos, endpos, obstacles, n_iter, radius, stepsize)
    path_smooth = smooth_path(waypoints)
    plot = plot_smoothed_path(waypoints, path_smooth, obstacles)

    # ---------------------------------------------- Initialize the simulation ----------------------------------------#

    INIT_XYZS = np.array([path_smooth[0]])
    env = CustomAviary(drone_model=drone,
                       num_drones=1,
                       initial_xyzs=INIT_XYZS,
                       physics=Physics.PYB_DW,
                       neighbourhood_radius=10,
                       pyb_freq=simulation_freq_hz,
                       ctrl_freq=control_freq_hz,
                       gui=gui,
                       record=record_video,
                       obstacles=True
                       )

    # ------------------------------------------- Initialize the trajectories -----------------------------------------#

    matplotlib_thread = threading.Thread(target=plot)

    # Start the threads
    matplotlib_thread.start()

    PERIOD = 30
    NUM_WP = control_freq_hz * PERIOD

    interp_func = interpolate.interp1d(
        np.linspace(0, 1, num=len(path_smooth)),
        path_smooth,
        axis=0,
        kind='linear'
    )
    new_indices = np.linspace(0, 1, num=NUM_WP)

    TARGET_POS = interp_func(new_indices)

    wp_counter = 0

    # -------------------------------- Initialize the controllers --------------------------------------#

    ctrl = DSLPIDControl(drone_model=drone)

    # --------------------------------------------- Run the simulation -----------------------------------------------#

    action = np.zeros((1, 4))
    START = time.time()
    for i in range(0, NUM_WP):

        # Step the simulation
        obs, reward, terminated, truncated, info = env.step(action)

        # Compute control for the current way point
        action[0, :], _, _ = ctrl.computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                          state=obs[0],
                                                          target_pos=TARGET_POS[wp_counter],
                                                          )

        # Go to the next way point and loop
        wp_counter = (wp_counter + 1) if wp_counter < (NUM_WP - 1) else 0

        # Printout
        env.render()

        # Sync the simulation
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    # Wait for the plotting thread to finish
    matplotlib_thread.join()

    # Close the environment
    env.close()

if __name__ == "__main__":
    # Define and parse (optional) arguments for the script
    parser = argparse.ArgumentParser(description='Downwash example script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone', default=DEFAULT_DRONE, type=DroneModel, help='Drone model (default: CF2X)',
                        metavar='', choices=DroneModel)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, help='Whether to use PyBullet GUI (default: True)',
                        metavar='')
    parser.add_argument('--record_video', default=DEFAULT_RECORD_VIDEO, type=str2bool,
                        help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int,
                        help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int,
                        help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int,
                        help='Duration of the simulation in seconds (default: 10)', metavar='')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str,
                        help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=bool,
                        help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))

