import os
import shutil
import sys

# 터미널에서 루트 디렉토리와 prefix를 sys.argv로 입력받음
if len(sys.argv) != 3:
    print("사용법: python your_script.py <루트 디렉토리> <prefix>")
    sys.exit(1)

root_dir = sys.argv[1]  # 첫 번째 인자로 루트 디렉토리 경로
prefix = sys.argv[2]    # 두 번째 인자로 prefix

# 루트 디렉토리에서 prefix로 시작하는 폴더들 찾기
source_dirs = [d for d in os.listdir(root_dir) if d.startswith(prefix) and os.path.isdir(os.path.join(root_dir, d))]

if not source_dirs:
    print(f"Error: {prefix}로 시작하는 폴더가 없습니다.")
    sys.exit(1)

# 각 source_dir 폴더에서 작업을 수행
for source_folder in source_dirs:
    # prefix를 제거한 폴더 이름 추출 (ex. slict_base1 -> base1)
    folder_name = source_folder.replace(prefix, '')
    txt_file = f"{folder_name}.txt"

    # 원본 txt 파일 경로 (source_folder 내부의 txt 파일)
    source_file_path = os.path.join(root_dir, source_folder, txt_file)

    if os.path.exists(source_file_path):
        # 루트 디렉토리에서 해당 folder_name을 포함하는 다른 폴더들 찾기 (자기 자신 제외)
        target_dirs = [d for d in os.listdir(root_dir) if folder_name in d and os.path.isdir(os.path.join(root_dir, d)) and d != source_folder]

        if not target_dirs:
            print(f"Error: {folder_name}를 포함한 타겟 폴더가 없습니다.")
            continue

        # 각 target_dir에 파일 복사
        for target_dir in target_dirs:
            target_path = os.path.join(root_dir, target_dir, txt_file)  # 파일명은 그대로 유지
            shutil.copy(source_file_path, target_path)
            print(f"파일 {source_file_path}이(가) {target_path}에 복사되었습니다.")
    else:
        print(f"{source_file_path} 파일이 {source_folder} 폴더에 존재하지 않습니다.")

