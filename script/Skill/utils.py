import os, shutil

# Project Folder (ROOT Project Location)
FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__),"../.."))

def delete_pycache_folders(verbose:bool=False):

  """ Delete Python `__pycache__` Folders Function """
  
  if verbose: print('\n\nDeleting `__pycache__` Folders...\n\n')

  # Walk Through the Project Folders
  for root, dirs, files in os.walk(FOLDER):

    if "__pycache__" in dirs:

      # Get `__pycache__` Path
      pycache_folder = os.path.join(root, "__pycache__")
      if verbose: print(f"{pycache_folder}")

      # Delete `__pycache__`
      try: shutil.rmtree(pycache_folder)
      except Exception as e: print(f"An error occurred while deleting {pycache_folder}: {e}")

  if verbose: print('\n\n')
