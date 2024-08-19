import psutil
import time

def get_total_cpu_usage(process_names, interval=1.0):
    """두 개의 프로세스 이름의 CPU 사용량을 측정하여 합산 및 평균을 반환."""
    total_cpu_usages = []
    
    try:
        while True:
            cpu_usage_sum = 0
            process_count = 0
            
            for process_name in process_names:
                target_process = None
                # 모든 프로세스 검색
                for proc in psutil.process_iter(['pid', 'name']):
                    if process_name in proc.info['name']:
                        target_process = proc
                        break
                
                if target_process:
                    # CPU 사용량 측정
                    cpu_usage = target_process.cpu_percent(interval=interval)
                    print(f"CPU Usage for {process_name}: {cpu_usage}%")
                    cpu_usage_sum += cpu_usage
                    process_count += 1
                else:
                    print(f"Process '{process_name}' not found.")
            
            if process_count > 0:
                total_cpu_usages.append(cpu_usage_sum)
                average_total_cpu_usage = sum(total_cpu_usages) / len(total_cpu_usages)
                print(f"Total CPU Usage: {cpu_usage_sum}%")
                print(f"Average Total CPU Usage: {average_total_cpu_usage}%")
            else:
                print("No matching processes found.")
            
            time.sleep(interval)
    
    except KeyboardInterrupt:
        print("Measurement stopped by user.")
    
    if total_cpu_usages:
        final_average_total_cpu_usage = sum(total_cpu_usages) / len(total_cpu_usages)
        return final_average_total_cpu_usage
    else:
        return None

# 사용 예시:
process_names = ["fast_lio_multi", ]  # 두 개의 프로세스 이름
average_total_cpu_usage = get_total_cpu_usage(process_names, interval=1.0)
if average_total_cpu_usage is not None:
    print(f"Final Average Total CPU usage for '{process_names}': {average_total_cpu_usage}%")
else:
    print("No CPU usage data collected.")
