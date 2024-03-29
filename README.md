# PDM_project

# RRT-gym-pybullet-combination

This project demonstrates the integration of an informed Rapidly Exploring Random Trees (RRT) path planning algorithm into a PyBullet environment using a custom Aviary. The simulated environment includes multiple randomly generated obstacles and flying waypoints for a drone to navigate through. The drone's trajectory is controlled using DSLPIDControl.

## Table of Contents

- [Introduction](#introduction)
- [Requirements](#requirements)
- [Installation](#installation)
- [Code Explanation](#code-explanation)
  - [File Structure](#file-structure)
  - [Code Comments](#code-comments)
- [Acknowledgments](#acknowledgments)

## Introduction

The project combines path planning algorithms, PyBullet simulation, and drone control to showcase a simulated environment with dynamic obstacles and trajectory planning for a drone. The main script, `final_sim_random.py`, initializes the environment, generates random obstacles, plans waypoints, and simulates drone navigation.

## Requirements

The code has undergone testing on both MacOS and Windows. To run the code, ensure that you have the following prerequisites installed:

- Python
- PyBullet
- NumPy
- SciPy

## Installation

The following is taken from pybullet-drones to complete the installation and clone the repository on a chosen local directory.

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones/

conda create -n drones python=3.10
conda activate drones

pip3 install --upgrade pip
pip3 install -e . # if needed, `sudo apt install build-essential` to install `gcc` and build `pybullet`
```

The following is needed to clone this repository onto the chosen local directory

```bash
cd .. 
git clone https://github.com/tcwmcramer/PDM_project.git
cd PDM_project/RRT_gym_pybullet_combination
$ python ./final_sim_random.py
```

## Code Explanation

### File Structure

- `final_sim_random.py`: Main script orchestrating the simulation, obstacle generation, and drone control.
- `Smoothpath.py`: Contains functions for smoothing paths using cubic spline interpolation.
- `RTT_star.py`: Implements the (informed) Rapidly Exploring Random Trees (RRT) path planning algorithm.
- `CustomAviary.py`: Custom PyBullet Aviary for drone simulation.
- `random_rubble_gen.py`: Generates random obstacle configurations.
- `pillars_gen.py`: Generates pillar-like obstacle configurations.

### Code Comments

The code includes comments to explain key functions and sections, providing clarity on the purpose and functionality of each component. Please refer to the code comments for detailed explanations.

## Acknowledgments

This project utilizes the following libraries and existing algorithms:

- [PyBullet](https://pybullet.org/): A physics engine for simulating robot dynamics.
- [gym-pybullet-drones](https://utiasdsl.github.io/gym-pybullet-drones/): PyBullet Gymnasium environments for single and multi-agent reinforcement learning of quadcopter control.
- [RRT_star.py](https://gist.github.com/fanjin-z/58e5eaa27a3dc004c3526ea82a92de80): Based on the Rapidly Exploring Random Trees (RRT) algorithm for path planning from Fanjing Zeng 2019 under MIT License.

