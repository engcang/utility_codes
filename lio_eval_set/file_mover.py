import os
import shutil
import sys

def move_files(folder1, folder2, prefix):
    # folder1 안에 있는 모든 폴더를 탐색
    for subdir in os.listdir(folder1):
        subdir_path = os.path.join(folder1, subdir)
        if os.path.isdir(subdir_path):
            # 원래 파일들과 코드999에서 생성된 파일들의 경로를 리스트에 추가
            files_to_move = [
                (os.path.join(subdir_path, 'leica_pose.csv'), 'leica_pose.csv'),
                (os.path.join(subdir_path, 'predict_odom.csv'), 'predict_odom.csv'),
                (os.path.join(subdir_path, 'cpu_usage.csv'), 'cpu_usage.csv'),
                (os.path.join(subdir_path, 'average_cpu_usage.txt'), 'average_cpu_usage.txt'),
                (os.path.join(subdir_path, 'calculation_time.csv'), 'calculation_time.csv'),
                (os.path.join(subdir_path, 'average_calculation_time.txt'), 'average_calculation_time.txt'),
                (os.path.join(subdir_path, 'localizability_x.csv'), 'localizability_x.csv'),
                (os.path.join(subdir_path, 'localizability_y.csv'), 'localizability_y.csv'),
                (os.path.join(subdir_path, 'localizability_z.csv'), 'localizability_z.csv'),
                (os.path.join(subdir_path, 'point_number.csv'), 'point_number.csv'),
                (os.path.join(subdir_path, 'velocity.csv'), 'velocity.csv'),
                (os.path.join(subdir_path, 'acc_bias.csv'), 'acc_bias.csv')
            ]

            # destination 폴더 경로 생성
            destination_folder = os.path.join(folder2, f"{prefix}{subdir}")

            # 파일 옮기기
            for src_file, dest_filename in files_to_move:
                if os.path.exists(src_file):
                    shutil.move(src_file, os.path.join(destination_folder, dest_filename))
                    print(f"Moved {src_file} to {destination_folder}")

if __name__ == "__main__":
    # 명령줄 인수로 folder1, folder2, prefix를 받음
    if len(sys.argv) != 4:
        print("Usage: python script.py <folder1> <folder2> <prefix>")
        sys.exit(1)

    folder1 = sys.argv[1]
    folder2 = sys.argv[2]
    prefix = sys.argv[3]

    move_files(folder1, folder2, prefix)
