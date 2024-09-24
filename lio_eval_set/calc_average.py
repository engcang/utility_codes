import os
import sys

def read_value_from_file(filepath, keyword):
    with open(filepath, 'r') as file:
        content = file.read().strip()
        # keyword로 값을 추출
        value = float(content.split(keyword)[-1].strip().replace('%', ''))
    return value

def calculate_average_for_files(root_dir, prefix):
    cpu_usage_sum = 0
    calc_time_sum = 0
    folder_count = 0
    
    # 루트 디렉토리 내 프리픽스에 맞는 폴더들에 대해 반복
    for folder_name in os.listdir(root_dir):
        folder_path = os.path.join(root_dir, folder_name)
        
        if os.path.isdir(folder_path) and folder_name.startswith(prefix):
            # 해당 폴더에 average_cpu_usage.txt와 average_calculation_time.txt가 있는지 확인
            cpu_usage_file = os.path.join(folder_path, 'average_cpu_usage.txt')
            calc_time_file = os.path.join(folder_path, 'average_calculation_time.txt')

            if os.path.exists(cpu_usage_file) and os.path.exists(calc_time_file):
                cpu_usage = read_value_from_file(cpu_usage_file, "Average CPU usage: ")
                calc_time = read_value_from_file(calc_time_file, "Average Calculation Time: ")

                cpu_usage_sum += cpu_usage
                calc_time_sum += calc_time
                folder_count += 1

    if folder_count == 0:
        print("해당 프리픽스에 맞는 폴더가 없습니다.")
        return None, None

    # 평균 계산
    avg_cpu_usage = cpu_usage_sum / folder_count
    avg_calc_time = calc_time_sum / folder_count

    return avg_cpu_usage, avg_calc_time

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("사용법: python script.py [루트 디렉토리] [프리픽스]")
        sys.exit(1)

    root_dir = sys.argv[1]
    prefix = sys.argv[2]

    avg_cpu_usage, avg_calc_time = calculate_average_for_files(root_dir, prefix)

    if avg_cpu_usage is not None and avg_calc_time is not None:
        print(f"평균 CPU 사용량: {avg_cpu_usage}%")
        print(f"평균 계산 시간: {avg_calc_time}")

