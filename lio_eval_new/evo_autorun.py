import os
import subprocess
import pandas as pd

# 폴더 경로 설정
base_folder = "./"

# 사용할 알고리즘 프리픽스 목록
prefixes = ["kiss_icp", "genz_icp", "dlo", "fast_lio", "dlio", "ig_lio", "ada_lio"]

# 결과를 저장할 데이터 프레임
results_df = pd.DataFrame()

# 각 폴더에 대해 반복
for folder in os.listdir(base_folder):
    folder_path = os.path.join(base_folder, folder)
    if os.path.isdir(folder_path):
        row = {"Folder Name": folder}
        # 각 프리픽스에 대해 반복
        for prefix in prefixes:
            odom_file = f"{prefix}_odom.csv"
            odom_path = os.path.join(folder_path, odom_file)
            ground_truth_file = f"{folder}.txt"
            ground_truth_path = os.path.join(folder_path, ground_truth_file)

            # evo_ape 명령어 실행
            command = f"evo_ape tum {ground_truth_path} {odom_path} -a --t_max_diff 0.2"
            process = subprocess.run(command, shell=True, text=True, capture_output=True)
            output = process.stdout
            
            # RMSE 값 추출 또는 오류 로깅
            if "rmse" in output:
                rmse_line = [line for line in output.split('\n') if "rmse" in line][0]
                rmse_value = float(rmse_line.split()[1])
                row[prefix] = round(rmse_value, 4)
            else:
                row[prefix] = "Error: rmse not found, check output"

        # 결과 데이터 프레임에 행 추가
        results_df = pd.concat([results_df, pd.DataFrame([row])], ignore_index=True)

# 결과 저장
results_df.set_index("Folder Name", inplace=True)
results_df.to_csv("rmse_results.csv", sep= ' ', float_format='%.4f')
print("RMSE results are saved to rmse_results.csv.")
