""" Make bounding box"""


import os

def process_urdf_files(directory_path):
    """
    Process all .urdf files in the specified directory.

    Parameters:
    - directory_path: The path to the directory containing .urdf files.

    Returns:
    - None (or you can modify the function to return some result based on your needs).
    """
    # Check if the directory exists
    if not os.path.exists(directory_path):
        print(f"Error: Directory '{directory_path}' does not exist.")
        return

    # List all files in the directory
    files = os.listdir(directory_path)

    # Filter files with .urdf extension
    urdf_files = [file for file in files if file.endswith(".urdf")]

    # Process each .urdf file
    for urdf_file in urdf_files:
        urdf_file_path = os.path.join(directory_path, urdf_file)

        # Your processing logic here
        print(f"Processing {urdf_file_path}")

        # For example, you might want to read and parse the URDF file
        # with an XML parser like ElementTree or BeautifulSoup
        # For simplicity, let's just print the content of each file
        with open(urdf_file_path, 'r') as file:
            print(file.read())

#TODO: lo

# Example usage:
directory_path = '/path/to/your/urdf/files'
process_urdf_files(directory_path)

def obs_boundingbox():
