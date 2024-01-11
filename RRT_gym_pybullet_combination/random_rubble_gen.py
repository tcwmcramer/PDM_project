import sys
import random
import math
import os

def generate_urdf_content(num_shapes, position_bounds, size_bounds, orientation_bounds):
    print("position_bounds:", position_bounds)
    print("size_bounds:", size_bounds)
    print("orientation_bounds:", orientation_bounds)
    urdf = '<?xml version="1.0"?>\n'
    urdf += '<robot name="rubble_robot" xmlns:xacro="http://ros.org/wiki/xacro">\n'
    urdf += '  <!-- Define materials if needed -->\n'
    urdf += '  <!-- <material name="rubble_material">\n'
    urdf += '    <color rgba="0.5 0.5 0.5 1" />\n'
    urdf += '  </material> -->\n\n'

    # Define root link
    urdf += '  <link name="base_link">\n'
    urdf += '    <inertial>\n'
    urdf += '      <mass value="0.0"/>\n'
    urdf += '      <origin xyz="0.0 0.0 0.0"/>\n'
    urdf += '      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>\n'
    urdf += '    </inertial>\n'
    urdf += '  </link>\n'

    joint_tags = ''  # To store joint tags separately

    for i in range(num_shapes):
        # Randomly choose a shape (box or sphere)
        shape = random.choice(["box", "sphere"])

        # Randomly generate size parameters
        size = [random.uniform(float(size_bounds.split(',')[0]), float(size_bounds.split(',')[1])) for _ in range(3)]

        # Randomly generate orientation (roll, pitch, yaw)
        rpy = [random.uniform(float(orientation_bounds.split(',')[0]), float(orientation_bounds.split(',')[1])) for _ in range(3)]

        # Randomly generate position (x, y, z)
        xyz = [random.uniform(float(position_bounds.split(',')[0]), float(position_bounds.split(',')[1])) for _ in range(3)]

        # Determine geometry based on the chosen shape
        if shape == "box":
            geometry = f'<box size="{size[0]} {size[1]} {size[2]}"/>'
        elif shape == "sphere":
            geometry = f'<sphere radius="{size[0]}"/>'

        # Add link to the URDF
        urdf += f'  <!-- Define link {i+1} -->\n'
        urdf += f'  <link name="rubble_link_{i+1}_collision">\n'
        urdf += '    <inertial>\n'
        urdf += '      <mass value="0.0"/>\n'
        urdf += '      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />\n'
        urdf += '      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>\n'
        urdf += '    </inertial>\n'
        urdf += '    <collision>\n'
        urdf += f'      <geometry>{geometry}</geometry>\n'
        urdf += f'      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}" />\n'
        urdf += '    </collision>\n'
        urdf += '  </link>\n'

        # Add joint to the joint_tags
        joint_tags += f'    <joint name="base_to_link_{i+1}" type="fixed">\n'
        joint_tags += f'      <parent link="base_link"/>\n'
        joint_tags += f'      <child link="rubble_link_{i+1}_collision"/>\n'
        joint_tags += '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        joint_tags += '    </joint>\n'

    # Combine links and joint tags
    urdf += joint_tags

    # Close root link
    urdf += '</robot>\n'

    return urdf

def generate_urdf_files(num_runs, num_shapes, size_bounds, orientation_bounds, output_directory):
    for run in range(num_runs):
        # Generate random position bounds between 0 and 5 for each run
        position_bounds = f"{random.uniform(0, 5)},{random.uniform(0, 5)}"

        # Generate URDF content
        urdf_content = generate_urdf_content(num_shapes, position_bounds, size_bounds, orientation_bounds)

        # Set the output path to the desired directory and file name
        output_path = os.path.join(output_directory, f"random_rubble_{run + 1}.urdf")

        # Write the URDF content to the file
        with open(output_path, "w") as urdf_file:
            urdf_file.write(urdf_content)

if __name__ == "__main__":
    # Set the common values for other parameters
    num_shapes = 15
    size_bounds = "0.2,0.5"
    orientation_bounds = "-1.0,1.0"
    output_directory = "obstacles"
    num_runs = 5

    # Call the function to generate URDFs
    generate_urdf_files(num_runs, num_shapes, size_bounds, orientation_bounds, output_directory)