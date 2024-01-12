import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from RRT_gym_pybullet_combination.RTT_star import pathSearch


# Function to smooth a path using cubic spline interpolation
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


# Function to plot the original waypoints and the smoothed path
def plot_smoothed_path(waypoints, path_smooth, obstacles):
    # Convert waypoints to a NumPy array
    waypoints = np.array(waypoints)

    # Plot the original waypoints and the smoothed path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c='blue', marker='o', label='Original Waypoints')
    ax.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], color='red', linewidth=2, label='Smoothed Path')

    # Plot obstacles as transparent spheres
    for obs in obstacles:
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = obs[0] + obs[3] * np.outer(np.cos(u), np.sin(v))
        y = obs[1] + obs[3] * np.outer(np.sin(u), np.sin(v))
        z = obs[2] + obs[3] * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='b', alpha=0.2)

    # Set labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()


if __name__ == '__main__':
    # Define start position, end position, obstacles, and parameters
    startpos = (0., 0., 0.)
    endpos = (5., 5., 0.)
    obstacles = [(1., 1., 1.), (2., 2., 2.)]
    n_iter = 200
    radius = 1.5
    stepSize = 0.7

    # Use the pathSearch function to generate waypoints
    waypoints = pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize)

    # Smooth the path using cubic spline interpolation
    path_smooth = smooth_path(waypoints)

    # Plot the original waypoints and the smoothed path with obstacles
    plot_smoothed_path(waypoints, path_smooth, obstacles)
