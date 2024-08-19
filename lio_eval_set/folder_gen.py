import os
import sys

# 명령줄 인수로 prefix를 받습니다.
if len(sys.argv) != 4:
    print("사용법: python folder_gen.py folder1 folder2 <prefix>")
    sys.exit(1)

prefix = sys.argv[3]
folder1 = sys.argv[1]
folder2 = sys.argv[2]

# folder1 안의 모든 폴더 이름을 가져옵니다
folder_names = [f for f in os.listdir(folder1) if os.path.isdir(os.path.join(folder1, f))]

# folder2 안에 prefix_폴더이름으로 새로운 폴더를 생성합니다
for folder_name in folder_names:
    new_folder_path = os.path.join(folder2, f'{prefix}{folder_name}')
    os.makedirs(new_folder_path, exist_ok=True)

print("폴더 생성 완료!")

