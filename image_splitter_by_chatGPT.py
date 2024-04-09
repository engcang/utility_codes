import os
import shutil
import random
from glob import glob
import sys

# Get paths and the number of files for the first group from command line arguments
if len(sys.argv) != 5:
    print("Usage: python3 image_splitter.py original_folder destination_folder1 destination_folder2 first_folder_file_number")
    sys.exit(1)

source_folder = sys.argv[1]
destination_folder1 = sys.argv[2]
destination_folder2 = sys.argv[3]
number_of_files_in_group1 = int(sys.argv[4])

# Create destination folders if they don't exist
os.makedirs(destination_folder1, exist_ok=True)
os.makedirs(destination_folder2, exist_ok=True)

# Read the list of JPG files from the source folder
file_paths = glob(os.path.join(source_folder, '*.jpg'))

# Shuffle the list of files randomly
random.shuffle(file_paths)

# Split the files into two groups based on the specified number for the first group
files_group1 = file_paths[:number_of_files_in_group1]
files_group2 = file_paths[number_of_files_in_group1:]

# Move the first group of files to destination folder 1
for file_path in files_group1:
    shutil.move(file_path, os.path.join(destination_folder1, os.path.basename(file_path)))

# Move the second group of files to destination folder 2
for file_path in files_group2:
    shutil.move(file_path, os.path.join(destination_folder2, os.path.basename(file_path)))

print("File movement complete!")
