import os

def process_files(folder_path):
    # 폴더 내의 모든 txt 파일을 순회
    for filename in os.listdir(folder_path):
        if filename.endswith('.txt'):
            file_path = os.path.join(folder_path, filename)
            print(file_path)
            new_lines = []
            
            with open(file_path, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    parts = line.split()
                    class_id = int(parts[0])
                    
                    # class id 조건에 따라 필터링 및 수정
                    if class_id == 0:
                        new_lines.append(line)  # class id 0인 경우 그대로 유지
                    elif class_id in [4, 5]:
                        parts[0] = '1'  # class id가 4나 5인 경우 1로 수정
                        new_lines.append(' '.join(parts) + '\n')
                    elif class_id in [1, 2, 3, 6, 7]:
                        continue  # class id가 1, 2, 3, 6, 7인 경우 제거

            # 수정된 내용으로 파일 덮어쓰기
            with open(file_path, 'w') as file:
                file.writelines(new_lines)

# 사용자로부터 폴더 경로 입력 받기
folder_path = input("폴더 경로를 입력하세요: ")
process_files(folder_path)
