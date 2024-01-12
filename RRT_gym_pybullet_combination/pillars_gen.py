import os
import math
import random

def generate_pillar_urdf_file(num_cubes, start_position=None):


    urdf = '<?xml version="1.0"?>\n'
    urdf += '<robot name="stacked_cubes">\n'

    # Set random values for yaw and pitch between 0 and 90 degrees
    roll = 0  # in degrees
    pitch = random.uniform(0, 90)
    yaw = random.uniform(0, 360)  # in degrees

    for i in range(num_cubes):


        # Calculate the offset based on the orientation
        xp = 0.3 * math.sin(math.radians(pitch))
        offset_x = i * xp * math.cos(math.radians(yaw))
        offset_y = i * xp * math.sin(math.radians(yaw))
        offset_z = i * 0.3 * math.cos(math.radians(pitch))

        if start_position is not None:
            offset_x += start_position[0]
            offset_y += start_position[1]

        urdf += f'  <!-- Link cube_{i} with center point and size information -->\n'
        urdf += f'  <link name="cube_{i}">\n'
        urdf += '    <visual>\n'
        urdf += '      <geometry>\n'
        urdf += '        <box size="0.3 0.3 0.3"/>\n'
        urdf += '      </geometry>\n'
        urdf += f'      <origin xyz="{offset_x} {offset_y} {offset_z}" rpy="{math.radians(roll)} {math.radians(pitch)} {math.radians(yaw)}"/>\n'
        urdf += '      <material name="dark_grey">\n'
        urdf += '        <color rgba="0.2 0.2 0.2 1"/>\n'
        urdf += '      </material>\n'
        urdf += '    </visual>\n'
        urdf += '    <inertial>\n'
        urdf += '      <mass value="0.0"/>\n'
        urdf += '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        urdf += '      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>\n'
        urdf += '    </inertial>\n'
        urdf += '    <!-- Collision information -->\n'
        urdf += '    <collision>\n'
        urdf += '      <geometry>\n'
        urdf += '        <box size="0.3 0.3 0.3"/>\n'
        urdf += '      </geometry>\n'
        urdf += f'      <origin xyz="{offset_x} {offset_y} {offset_z}" rpy="{math.radians(roll)} {math.radians(pitch)} {math.radians(yaw)}"/>\n'
        urdf += '    </collision>\n'
        urdf += '  </link>\n'

        if i > 0:
            urdf += f'  <joint name="joint_{i}" type="fixed">\n'
            urdf += f'    <parent link="cube_{i-1}"/>\n'
            urdf += f'    <child link="cube_{i}"/>\n'
            urdf += f'    <origin xyz="{0} {0} {0}" rpy="0 0 0"/>\n'
            urdf += '  </joint>\n'

    urdf += '</robot>\n'

    return urdf
    # with open(output_file, "w") as urdf_file:
    #     urdf_file.write(urdf)

def generate_multiple_pillar_urdf_files(num_files, num_cubes, output_directory):
    for i in range(num_files):
        start_position = (random.uniform(0, 5), random.uniform(0, 5))

        # Generate URDF content
        urdf_content = generate_pillar_urdf_file(num_cubes, start_position)

        # Set the output path to the desired directory and file name
        output_path = os.path.join(output_directory, f"pillar_{i + 1}.urdf")

        # Write the URDF content to the file
        with open(output_path, "w") as urdf_file:
            urdf_file.write(urdf_content)


if __name__ == "__main__":
    num_files = 5
    num_cubes = 10
    output_directory = "obstacles"

    os.makedirs(output_directory, exist_ok=True)

    generate_multiple_pillar_urdf_files(num_files, num_cubes, output_directory)
