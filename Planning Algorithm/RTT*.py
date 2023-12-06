import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Node:
    def __init__(self, position):
        self.position = np.array(position)
        self.parent = None
        self.cost = 0.0

def distance(node1, node2):
    return np.linalg.norm(node1.position - node2.position)

def random_sample(bounds):
    return [np.random.uniform(bounds[i][0], bounds[i][1]) for i in range(3)]

def nearest_neighbor(tree, sample):
    distances = [distance(node, sample) for node in tree]
    return tree[np.argmin(distances)]

def steer(from_node, to_node, max_distance):
    d = distance(from_node, to_node)
    if d < max_distance:
        return to_node.position
    else:
        direction = (to_node.position - from_node.position) / d
        return from_node.position + direction * max_distance

def is_collision_free(from_point, to_point):
    # Add your collision checking logic here
    return True

def rrt_star(start, goal, bounds, max_iter, max_distance):
    tree = [start]

    for _ in range(max_iter):
        sample = Node(random_sample(bounds))
        nearest = nearest_neighbor(tree, sample)

        new_node_position = steer(nearest, sample, max_distance)
        new_node = Node(new_node_position)

        if is_collision_free(nearest.position, new_node.position):
            near_nodes = [node for node in tree if distance(node, new_node) < max_distance]

            # Connect to the nearest neighbor with minimum cost
            min_cost_node = min(near_nodes, key=lambda x: x.cost + distance(x, new_node))
            new_node.parent = min_cost_node
            new_node.cost = min_cost_node.cost + distance(min_cost_node, new_node)

            # Rewire the tree
            for node in near_nodes:
                if node.cost > new_node.cost + distance(node, new_node) and \
                   is_collision_free(node.position, new_node.position):
                    node.parent = new_node
                    node.cost = new_node.cost + distance(node, new_node)

            tree.append(new_node)

    return tree

def plot_tree(tree, goal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in tree:
        if node.parent:
            ax.plot([node.parent.position[0], node.position[0]],
                    [node.parent.position[1], node.position[1]],
                    [node.parent.position[2], node.position[2]], color='black')

    ax.scatter(*goal.position, color='red', label='Goal')
    ax.scatter(*start.position, color='green', label='Start')

    plt.legend()
    plt.show()

# Example usage
start = Node([0, 0, 0])
goal = Node([10, 10, 10])
bounds = [(0, 15), (0, 15), (0, 15)]
max_iter = 500
max_distance = 1.0

tree = rrt_star(start, goal, bounds, max_iter, max_distance)
plot_tree(tree, goal)
