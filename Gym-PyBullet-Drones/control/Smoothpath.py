import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from RTT_star import pathSearch
from mpl_toolkits.mplot3d import Axes3D

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

def plot_smoothed_path(waypoints, path_smooth):
    # Convert waypoints to a NumPy array
    waypoints = np.array(waypoints)

    # Plot the original waypoints and the smoothed path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c='blue', marker='o', label='Original Waypoints')
    ax.plot(path_smooth[:, 0], path_smooth[:, 1], path_smooth[:, 2], color='red', linewidth=2, label='Smoothed Path')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


startpos = (0., 0., 0.)
endpos = (5., 5., 5.)
obstacles = [(1., 1., 1.), (2., 2., 2.)]
n_iter = 200
radius = 1.5
stepSize = 0.7

# Example usage with an unknown number of waypoints
waypoints = pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize)



path_smooth = smooth_path(waypoints)
plot_smoothed_path(waypoints, path_smooth)