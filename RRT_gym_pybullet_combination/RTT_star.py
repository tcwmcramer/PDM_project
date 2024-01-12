'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.
'''

import os
import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib
import xml.etree.ElementTree as ET

class Line():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


def Intersection(line, center, radius):
    ''' Check line-sphere (circle) intersection '''
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a);
    t2 = (-b - np.sqrt(discriminant)) / (2 * a);

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True



def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def isInObstacle(vex, obstacles):
    for obs in obstacles:
        if distance(obs[:3], vex) < obs[3]:
            return True
    return False


def isThruObstacle(line, obstacles):
    for obs in obstacles:
        if Intersection(line, obs[:3], obs[3]):
            return True
    return False


def nearest(G, vex, obstacles):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1], nearvex[2]+dirn[2])
    return newvex


def window(startpos, endpos):
    ''' Define seach window - 2 times of start to end rectangle'''
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    depth = endpos[2] - startpos[2]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    winz = startpos[2] - (depth / 2.)
    return winx, winy, winz, width, height, depth


def isInWindow(pos, winx, winy, winz, width, height, depth):
    ''' Restrict new vertex insides search window'''
    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height and \
        winz < pos[2] < winz+depth:
        return True
    else:
        return False


class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        rx = random()
        ry = random()
        rz = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        posz = self.startpos[2] - (self.sz / 2.) + rz * self.sz * 2
        return posx, posy, posz

    def randomPositionWithinInformedSet(self, center, x_axis, x_radius, y_radius, z_radius):
        ''' Generate a random position within the informed set (ellipsoid) '''
        # Generate random spherical coordinates
        phi = 2 * np.pi * np.random.uniform(0, 1)
        theta = np.arccos(2 * np.random.uniform(0, 1) - 1)

        # Convert spherical coordinates to Cartesian coordinates
        spherical_point = np.array([x_radius * np.sin(theta) * np.cos(phi),
                                    y_radius * np.sin(theta) * np.sin(phi),
                                    z_radius * np.cos(theta)])

        # Rotate the spherical point to align with the x-axis direction
        rotation_matrix = self.rotationMatrixFromVector(np.array([1, 0, 0]), x_axis)
        rotated_point = np.dot(rotation_matrix, spherical_point)

        # Translate the rotated point to the informed set center
        position = rotated_point + center

        return tuple(position)

    def rotationMatrixFromVector(self, v1, v2):
        ''' Calculate the rotation matrix that rotates v1 to v2 '''
        axis = np.cross(v1, v2)
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        return self.rotationMatrixFromAxisAngle(axis, angle)

    def rotationMatrixFromAxisAngle(self, axis, angle):
        ''' Calculate the rotation matrix from an axis and an angle '''
        axis /= np.linalg.norm(axis)
        a = np.cos(angle / 2.0)
        b, c, d = -axis * np.sin(angle / 2.0)
        return np.array([[a**2 + b**2 - c**2 - d**2, 2 * (b*c - a*d), 2 * (b*d + a*c)],
                         [2 * (b*c + a*d), a**2 + c**2 - b**2 - d**2, 2 * (c*d - a*b)],
                         [2 * (b*d - a*c), 2 * (c*d + a*b), a**2 + d**2 - b**2 - c**2]])

def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):
    ''' RRT algorithm '''
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            #print('success')
            # break
    return G


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    ''' RRT star algorithm '''
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        ground_threshold = 0
        # Ensure that the random position does not go below the ground
        randvex = (randvex[0], randvex[1], max(randvex[2], ground_threshold))

        if isInObstacle(randvex, obstacles):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter) #Check if the new vertex allows others to have a shorter distance
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            print('success')
            break
    return G

def initialize_informed_set(startpos, endpos, initial_radius_fraction):
    center = np.array([(start + end) / 2.0 for start, end in zip(startpos, endpos)])
    x_axis = np.array(endpos) - np.array(startpos)
    x_axis /= np.linalg.norm(x_axis)
    initial_radius = distance(startpos, endpos) * initial_radius_fraction

    return center, x_axis, initial_radius, initial_radius, initial_radius

def point_in_ellipsoid(point, a, b, c):
    # Check if a point is inside the ellipsoid
    x, y, z = point
    equation_result = (x**2 / a**2) + (y**2 / b**2) + (z**2 / c**2)
    return equation_result <= 1

def update_informed_set(graph,
                        informed_set_center,
                        informed_set_x_axis,
                        informed_set_x_radius,
                        informed_set_y_radius,
                        informed_set_z_radius,
                        iterations=50):

    center, x_axis, x_radius, y_radius, z_radius = (informed_set_center,
                                                    informed_set_x_axis,
                                                    informed_set_x_radius,
                                                    informed_set_y_radius,
                                                    informed_set_z_radius)

    best_path = dijkstra(graph)

    # Check and reduce y-radius
    for _ in range(iterations):
        if all(point_in_ellipsoid(point, x_radius, y_radius, z_radius) for point in best_path):
            y_radius *= 0.99
            print('y adjusted')
        else:
            break  # Exit loop if any point falls outside the ellipsoid

    # Check and reduce z-radius
    for _ in range(iterations):
        if all(point_in_ellipsoid(point, x_radius, y_radius, z_radius) for point in best_path):
            z_radius *= 0.99
            print('z adjusted')
        else:
            break  # Exit loop if any point falls outside the ellipsoid

    # Check and reduce x-radius
    for _ in range(iterations):
        if all(point_in_ellipsoid(point, x_radius, y_radius, z_radius) for point in best_path):
            x_radius *= 0.99
            print('x adjusted')
        else:
            break  # Exit loop if any point falls outside the ellipsoid


    return center, x_axis, x_radius, y_radius, z_radius

def RRT_star_informed(startpos, endpos, obstacles, n_iter, radius, stepSize, initial_radius_fraction):
    G = Graph(startpos, endpos)

    # Initialize the ellipsoid variables
    last_ellipsoid = None

    (informed_set_center,
     informed_set_x_axis,
     informed_set_x_radius,
     informed_set_y_radius,
     informed_set_z_radius) = initialize_informed_set(startpos, endpos, initial_radius_fraction)
    print(informed_set_x_radius,informed_set_y_radius,informed_set_z_radius)

    for _ in range(n_iter):
        # Biased Sampling towards Informed Set
        if random() < .8:
            randvex = G.randomPositionWithinInformedSet(informed_set_center,
                                                        informed_set_x_axis,
                                                        informed_set_x_radius,
                                                        informed_set_y_radius,
                                                        informed_set_z_radius)
            ground_threshold = 0
            randvex = (randvex[0], randvex[1], max(randvex[2], ground_threshold))
        else:
            randvex = G.randomPosition()
            ground_threshold = 0
            randvex = (randvex[0], randvex[1], max(randvex[2], ground_threshold))

        if isInObstacle(randvex, obstacles):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx] + dist)
            except:
                G.distances[endidx] = G.distances[newidx] + dist

            (informed_set_center,
             informed_set_x_axis,
             informed_set_x_radius,
             informed_set_y_radius,
             informed_set_z_radius) = update_informed_set(G,
                                                          informed_set_center,
                                                          informed_set_x_axis,
                                                          informed_set_x_radius,
                                                          informed_set_y_radius,
                                                          informed_set_z_radius)

            last_ellipsoid = (informed_set_center,
                              informed_set_x_axis,
                              informed_set_x_radius,
                              informed_set_y_radius,
                              informed_set_z_radius)
            print(last_ellipsoid)

            G.success = True
            print('success')
    return G, last_ellipsoid


def dijkstra(G):
    '''
    Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)



def plot(G, obstacles, path=None, informed_ellipsoid=None):
    '''
    Plot RRT, obstacles and shortest path
    '''
    #matplotlib.use('Qt5Agg')

    px = [x for x, y, z in G.vertices]
    py = [y for x, y, z in G.vertices]
    pz = [z for x, y, z in G.vertices]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for obs in obstacles:
        # Add a sphere to the environment
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = obs[0] + obs[3] * np.outer(np.cos(u), np.sin(v))
        y = obs[1] + obs[3] * np.outer(np.sin(u), np.sin(v))
        z = obs[2] + obs[3] * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='b', alpha=0.2)

    # Plot RRT vertices
    ax.scatter(px, py, pz, c='cyan', marker='o')

    # Plot start and end positions
    ax.scatter(G.startpos[0], G.startpos[1], G.startpos[2], c='black', marker='s')
    ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='black', marker='s')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = Line3DCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = Line3DCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    # Plot final ellipsoid if provided
    if informed_ellipsoid is not None:
        center, x_axis, x_radius, y_radius, z_radius = informed_ellipsoid
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = center[0] + x_radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + y_radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + z_radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='red', alpha=0.2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star_informed(startpos, endpos, obstacles, n_iter, radius, stepSize)
    # G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)

    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path
    else:
        print('NO PATH FOUND')
        return None


def parse_urdf(urdf_file):
    # Load the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Create an empty matrix
    matrix = []

    # Iterate through each link in the URDF
    for link in root.findall(".//link"):
        link_name = link.get('name')

        # Extract collision information
        collision = link.find(".//collision")
        if collision is not None:
            collision_geometry = collision.find(".//geometry/*")
            if collision_geometry is not None:
                if collision_geometry.tag == 'sphere':
                    radius_collision = float(collision_geometry.get('radius'))
                # elif collision_geometry.tag == 'cylinder':
                #     radius_collision = float(collision_geometry.get('radius'))
                elif collision_geometry.tag == 'box':
                    # Calculate the radius for the sphere around the box
                    size = [float(s) for s in collision_geometry.get('size').split()]
                    diagonal_length = np.linalg.norm(size) / 2.0
                    radius_collision = diagonal_length
                else:
                    # Handle other geometry types if necessary
                    radius_collision = None
            else:
                radius_collision = None

            # Extract origin information
            origin = collision.find(".//origin")
            if origin is not None:
                xyz = [float(x) for x in origin.get('xyz').split()]
            else:
                xyz = [0, 0, 0]

            # Add the information to the matrix
            matrix.append([xyz[0], xyz[1], xyz[2], radius_collision])

    # Convert the matrix to a NumPy array for easier manipulation
    matrix = np.array(matrix)

    return matrix


def all_urdf():


    current_directory = os.getcwd()

    # Construct the path to the "obstacles" folder
    obstacles_folder = os.path.join(current_directory, "obstacles")

    # Get a list of all files in the "obstacles" folder
    all_files = os.listdir(obstacles_folder)

    # Filter out only the URDF files
    urdf_files = [file for file in all_files if file.endswith(".urdf")]

    obstacles = np.array([])

    for urdf_file in urdf_files:
        # Construct the full path to the URDF file
        urdf_path = os.path.join(obstacles_folder, urdf_file)
        matrix = parse_urdf(urdf_path)
        if not obstacles.size:
            obstacles = matrix
        else:
            obstacles = np.vstack((obstacles, matrix))
    return obstacles


if __name__ == '__main__':

    startpos = (5., 5., 3.)
    endpos = (0., 0., 0.)
    #urdf_path = "../RRT_gym_pybullet_combination/obstacles/random_rubble_1.urdf"  # Update with your actual URDF file path
    obstacles = all_urdf()
    last_ellipsoid = None
    radius = 1.5
    n_iter = 200
    stepSize = 0.7

    # G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    # G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)
    G, last_ellipsoid = RRT_star_informed(startpos, endpos, obstacles, n_iter, radius, stepSize, initial_radius_fraction=2.5)
    if G.success:
        path = dijkstra(G)
        print(path)
        plot(G, obstacles, path=path, informed_ellipsoid=last_ellipsoid)
    else:
        print('NO PATH FOUND')