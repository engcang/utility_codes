import csv
import numpy as np
from scipy.spatial.transform import Rotation as R   

def read_csv(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            timestamp = float(row[0]) * 1e-9  # Convert timestamp to seconds
            matrix_values = np.array([float(val) for val in row[1:]]).reshape(3, 4)
            rotation_matrix = matrix_values[:, :3]
            translation_vector = matrix_values[:, 3]
            data.append((timestamp, rotation_matrix, translation_vector))
    return data

def save_as_tum(poses, output_file):
    with open(output_file, 'w') as out_file:
        for pose in poses:
            timestamp, rotation_matrix, translation_vector = pose
            rotation_quat = R.from_matrix(rotation_matrix).as_quat()
            out_file.write(f'{timestamp:.9f} ' + ' '.join(map(str, translation_vector)) + ' ' + ' '.join(map(str, rotation_quat)) + '\n')

csv_file_path = 'DCC03/global_pose.csv'
output_txt_file = 'DCC03/tum_gt.csv'

poses = read_csv(csv_file_path)
save_as_tum(poses, output_txt_file)
