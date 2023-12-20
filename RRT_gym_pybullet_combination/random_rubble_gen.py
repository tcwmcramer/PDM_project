import sys
import random
import math
import os

def generate_urdf(num_shapes, position_bounds, size_bounds, orientation_bounds):
    print("position_bounds:", position_bounds)
    print("size_bounds:", size_bounds)
    print("orientation_bounds:", orientation_bounds)
    urdf = '<?xml version=\'1.0\' encoding=\'utf-8\'?>\n<robot name="rubble">\n'

    size_bounds_list = size_bounds.split(',')
    if len(size_bounds_list) != 2:
        raise ValueError("Invalid size_bounds format. Use 'min,max'")
    try:
        size_min = float(size_bounds_list[0])
        size_max = float(size_bounds_list[1])
    except ValueError:
        raise ValueError(f"Invalid size_bounds values: {size_bounds_list[0]}, {size_bounds_list[1]}")

    position_bounds_list = position_bounds.split(',')
    if len(position_bounds_list) != 2:
        raise ValueError("Invalid position_bounds format. Use 'min,max'")
    try:
        pos_min = float(position_bounds_list[0])
        pos_max = float(position_bounds_list[1])
    except ValueError:
        raise ValueError(f"Invalid position_bounds values: {position_bounds_list[0]}, {position_bounds_list[1]}")

    for i in range(num_shapes):
        # Randomly choose a shape (box, sphere, cylinder)
        shape = random.choice(["box", "sphere", "cylinder"])

        # Randomly generate size parameters
        size = [random.uniform(size_min, size_max) for _ in range(3)]

        # Randomly generate orientation (roll, pitch, yaw)
        rpy = [random.uniform(-math.pi, math.pi) for _ in range(3)]

        # Randomly generate position (x, y, z)
        xyz = [random.uniform(pos_min, pos_max) for _ in range(3)]

        # Determine geometry based on the chosen shape
        if shape == "box":
            geometry = f'<box size="{size[0]} {size[1]} {size[2]}"/>'
        elif shape == "sphere":
            geometry = f'<sphere radius="{size[0]}"/>'
        elif shape == "cylinder":
            geometry = f'<cylinder length="{size[0]}" radius="{size[1]}"/>'

        # Append the generated link to the URDF string
        urdf += f'<link name="rubble_link_{i + 1}_visual">\n'
        urdf += '  <visual>\n'
        urdf += f'    <geometry>{geometry}</geometry>\n'
        urdf += f'    <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}" />\n'
        urdf += '    <material name="rubble_material">\n'
        urdf += '      <color rgba="0.5 0.5 0.5 1" />\n'
        urdf += '    </material>\n'
        urdf += '  </visual>\n'
        urdf += '</link>\n'

        urdf += f'<link name="rubble_link_{i + 1}_collision">\n'
        urdf += '  <collision>\n'
        urdf += f'    <geometry>{geometry}</geometry>\n'
        urdf += f'    <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}" />\n'
        urdf += '  </collision>\n'
        urdf += '</link>\n'

    urdf += '</robot>'
    return urdf

if __name__ == "__main__":

    # Set the values directly in the script
    num_shapes = 15
    position_bounds = "-2.0,2.0"
    size_bounds = "2.2,2.8"
    orientation_bounds = "-1.0,1.0"
    output_directory = "../Gym-PyBullet-Drones/assets"

    # Set the output path to the desired directory and file name

    urdf_content = generate_urdf(num_shapes, position_bounds, size_bounds, orientation_bounds)

    # # Parse command-line arguments
    # if len(sys.argv) != 5:
    #     print("Usage: python script.py num_shapes position_bounds size_bounds orientation_bounds")
    #     sys.exit(1)
    #
    # num_shapes = int(sys.argv[1])
    # position_bounds = sys.argv[2]
    # size_bounds = sys.argv[3]
    # orientation_bounds = sys.argv[4]
    #
    # urdf_content = generate_urdf(num_shapes, position_bounds, size_bounds, orientation_bounds)
    output_path = os.path.join(output_directory, "random_rubble.urdf")

    with open(output_path, "w") as urdf_file:
        urdf_file.write(urdf_content)

