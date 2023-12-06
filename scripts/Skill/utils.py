import os, shutil
from pathlib import Path

# Get Package Path
PACKAGE_PATH = str(Path(__file__).resolve().parents[2])

def get_ros2_workspace_path() -> str:

    """ Get ROS2 Workspace Path Function """

    # Reverse Search for the 'build', 'install', 'log', 'src' directories
    current_path = os.path.dirname(os.path.abspath(__file__))
    while current_path != '/' and not all(dir in os.listdir(current_path) for dir in ['build', 'install', 'log', 'src']):
        current_path = os.path.dirname(current_path)

    return current_path

def delete_pycache_folders(verbose:bool=False):

    """ Delete Python `__pycache__` Folders Function """
  
    if verbose: print('\n\nDeleting `__pycache__` Folders...\n\n')

    # Walk Through the Project Folders
    for root, dirs, files in os.walk(PACKAGE_PATH):

        if "__pycache__" in dirs:

            # Get `__pycache__` Path
            pycache_folder = os.path.join(root, "__pycache__")
            if verbose: print(f"{pycache_folder}")

            # Delete `__pycache__`
            try: shutil.rmtree(pycache_folder)
            except Exception as e: print(f"An error occurred while deleting {pycache_folder}: {e}")

    if verbose: print('\n\n')
