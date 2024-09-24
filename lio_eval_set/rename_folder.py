import os
import sys

def rename_folders_with_prefix(directory, prefix):
    # 주어진 디렉토리 내의 폴더 목록을 가져옵니다
    for folder_name in os.listdir(directory):
        folder_path = os.path.join(directory, folder_name)
        
        # 폴더인지 확인하고, prefix로 시작하는지 확인
        if os.path.isdir(folder_path) and folder_name.startswith(prefix):
            # prefix를 제거한 새로운 폴더 이름 생성
            new_folder_name = folder_name[len(prefix):]
            new_folder_path = os.path.join(directory, new_folder_name)
            
            # 폴더 이름 변경
            os.rename(folder_path, new_folder_path)
            print(f'Renamed: {folder_name} -> {new_folder_name}')

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("사용법: python script.py <폴더 경로> <제거할 프리픽스>")
        sys.exit(1)
    
    # 터미널에서 받은 인자 처리
    directory_path = sys.argv[1]
    prefix_to_remove = sys.argv[2]
    
    rename_folders_with_prefix(directory_path, prefix_to_remove)

