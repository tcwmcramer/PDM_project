import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from RRT_gym_pybullet_combination.RTT_star import pathSearch
from RTT_star import all_urdf

def smooth_path(waypoints, num_points=100):
    # Transpose to have coordinates in separate arrays
    waypoints = np.array(waypoints).T
    t = np.arange(len(waypoints[0]))

    # Create cubic spline functions for each dimension (x, y, z)
    cs_x = CubicSpline(t, waypoints[0])
    cs_y = CubicSpline(t, waypoints[1])
    cs_z = CubicSpline(t, waypoints[2])

    # Specify the number of points for the interpolated path
    t_new = np.linspace(t.min(), t.max(), num_points)

    # Interpolate the path
    path_smooth = np.column_stack((cs_x(t_new), cs_y(t_new), cs_z(t_new)))
    return path_smooth

def plot_smoothed_path(waypoints, path_smooth, obstacles):
    # Convert waypoints to a NumPy array
    waypoints = np.array(waypoints)

    # Plot the original waypoints and the smoothed path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c='blue', marker='o', label='Original Waypoints')
    ax.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], color='red', linewidth=2, label='Smoothed Path')
    for obs in obstacles:
        # Add a sphere to the environment
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = obs[0] + obs[3] * np.outer(np.cos(u), np.sin(v))
        y = obs[1] + obs[3] * np.outer(np.sin(u), np.sin(v))
        z = obs[2] + obs[3] * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='b', alpha=0.2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

if __name__ == '__main__':
    startpos = (5., 5., 3.)
    endpos = (0., 0., 0.)
    # urdf_path = "../RRT_gym_pybullet_combination/obstacles/random_rubble_1.urdf"  # Update with your actual URDF file path
    obstacles = all_urdf()
    last_ellipsoid = None
    radius = 1.5
    n_iter = 200
    stepSize = 0.7

    # Example usage with an unknown number of waypoints
    waypoints = pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize)



    path_smooth = smooth_path(waypoints)

    def compute_distance(positions):
        distance = 0.0
        for i in range(1, len(positions)):
            delta_position = positions[i] - positions[i - 1]
            distance += np.linalg.norm(delta_position)
        return distance

    # Assuming TARGET_POS is a NumPy array of shape (NUM_WP, 3)
    flown_distance = compute_distance(path_smooth)

    print("Total Flown Distance:", flown_distance)

    plot_smoothed_path(waypoints, path_smooth, obstacles)