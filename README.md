# PDM_project

# RRT-gym-pybullet-combination

This project demonstrates the integration of Rapidly Exploring Random Trees (RRT) path planning algorithm into a PyBullet environment using a custom Aviary. The simulated environment includes multiple randomly generated obstacles and flying waypoints for a drone to navigate through. The drone's trajectory is controlled using DSLPIDControl.

## Table of Contents

- [Introduction](#introduction)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Code Explanation](#code-explanation)
  - [File Structure](#file-structure)
  - [Code Comments](#code-comments)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Introduction

The project combines path planning algorithms, PyBullet simulation, and drone control to showcase a simulated environment with dynamic obstacles and trajectory planning for a drone. The main script, `target_position2.py`, initializes the environment, generates random obstacles, plans waypoints, and simulates drone navigation.

## Requirements

- Python
- PyBullet
- NumPy
- SciPy

## Installation

```bash
# Clone the repository
git clone https://github.com/your-username/your-repo.git

# Navigate to the project directory
cd your-repo

# Install dependencies
pip install -r requirements.txt
```


## Code Explanation

### File Structure

- `target_position2.py`: Main script orchestrating the simulation, obstacle generation, and drone control.
- `Smoothpath.py`: Contains functions for smoothing paths using cubic spline interpolation.
- `RTT_star.py`: Implements the (informed) Rapidly Exploring Random Trees (RRT) path planning algorithm.
- `CustomAviary.py`: Custom PyBullet Aviaries for drone simulation.
- `random_rubble_gen.py`: Generates random obstacle configurations.
- `pillars_gen.py`: Generates pillar-like obstacle configurations.

### Code Comments

The code includes comments to explain key functions and sections, providing clarity on the purpose and functionality of each component. Please refer to the code comments for detailed explanations.
