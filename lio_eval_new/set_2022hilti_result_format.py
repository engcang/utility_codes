import os
import shutil

def copy_and_rename_files(source_folders, destination_base, prefix):
    # destination_base 경로 생성
    if not os.path.exists(destination_base):
        os.makedirs(destination_base)

    for folder in source_folders:
        # 각 폴더에서 프리픽스_odom.csv 파일 경로 설정
        source_file = os.path.join(folder, f"{prefix}_odom.csv")

        # 파일 존재 여부 확인
        if os.path.exists(source_file):
            # destination 폴더 생성
            folder_name = os.path.basename(folder)
            destination_folder = os.path.join(destination_base, prefix)
            if not os.path.exists(destination_folder):
                os.makedirs(destination_folder)

            # 파일 복사 및 이름 변경
            destination_file = os.path.join(destination_folder, f"{folder_name}.txt")
            shutil.copy(source_file, destination_file)
            print(f"{source_file} -> {destination_file} 복사 완료")
        else:
            print(f"파일을 찾을 수 없습니다: {source_file}")

# 사용 예시
source_folders = ["2022hilti/exp02_construction_multilevel", "2022hilti/exp03_construction_stairs", "2022hilti/exp07_long_corridor", "2022hilti/exp11_lower_gallery"]
destination_base = "2022hilti"
prefix = "baseline_naloam"  # 원하는 프리픽스

copy_and_rename_files(source_folders, destination_base, prefix)
