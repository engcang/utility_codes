import os
import subprocess
import time
import psutil
import sys
import signal
from multiprocessing import Process

def terminate_process_and_children(p):
    """프로세스와 모든 자식 프로세스를 종료"""
    try:
        ps_command = subprocess.Popen(f"ps -o pid --ppid {p.pid} --noheaders", shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        ps_command.wait()

        for pid_str in ps_output.decode().split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGTERM)
        
        p.terminate()
        p.wait()
    except Exception as e:
        print(f"Error terminating process {p.pid}: {e}")

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Terminating processes...')
    for p in processes:
        terminate_process_and_children(p)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

processes = []

def run_roslaunch():
    """roslaunch 명령어를 실행"""
    roslaunch_cmd = ["vglrun", "roslaunch", "slict", "run_ntuviral.launch"]
    roslaunch_process = subprocess.Popen(roslaunch_cmd, preexec_fn=os.setsid)
    processes.append(roslaunch_process)
    return roslaunch_process

def play_bag_file(bag_file_path):
    """rosbag play 명령어를 실행"""
    play_bag_cmd = ["rosbag", "play", bag_file_path, "--clock"]
    play_bag_process = subprocess.Popen(play_bag_cmd, preexec_fn=os.setsid)
    processes.append(play_bag_process)
    return play_bag_process

def run_rostopic_echo(topic, output_file):
    """rostopic echo 명령어를 실행"""
    rostopic_cmd = ["rostopic", "echo", "-p", "--nostr", "--noarr", topic]
    with open(output_file, 'w') as f:
        rostopic_process = subprocess.Popen(rostopic_cmd, stdout=f, preexec_fn=os.setsid)
    processes.append(rostopic_process)
    return rostopic_process

def monitor_cpu_usage(process_name, interval=1.0):
    """특정 프로세스의 CPU 사용량을 모니터링하여 평균 계산"""
    cpu_usages = []
    try:
        for proc in psutil.process_iter(['pid', 'name']):
            if process_name in proc.info['name']:  # 이름에 process_name이 포함되어 있는지 확인
                target_proc = psutil.Process(proc.info['pid'])
                while target_proc.is_running():
                    cpu_usage = target_proc.cpu_percent(interval=interval)
                    cpu_usages.append(cpu_usage)
                    time.sleep(interval)
    except psutil.NoSuchProcess:
        pass

    if cpu_usages:
        average_cpu_usage = sum(cpu_usages) / len(cpu_usages)
        return average_cpu_usage
    else:
        return None

def main(root_directory, topic_name, target_node_name):
    for folder in os.listdir(root_directory):
        folder_path = os.path.join(root_directory, folder)
        
        if os.path.isdir(folder_path):
            bag_file_path = os.path.join(folder_path, f"{folder}.bag")
            
            if os.path.exists(bag_file_path):
                # 1. roslaunch 실행
                roslaunch_process = run_roslaunch()
                time.sleep(5)  # roslaunch가 완전히 실행될 시간을 줌

                # 4. rostopic echo (topic) 실행
                predict_odom_file = os.path.join(folder_path, "predict_odom.csv")
                predict_odom_process = run_rostopic_echo(topic_name, predict_odom_file)

                # 5. rostopic echo (/leica/pose/relative) 실행
                leica_pose_file = os.path.join(folder_path, "leica_pose.csv")
                leica_pose_process = run_rostopic_echo("/leica/pose/relative", leica_pose_file)

                # 2. CPU 모니터링 시작 (별도의 프로세스로)
                monitor_process = Process(target=monitor_cpu_usage, args=(target_node_name,))
                monitor_process.start()
                processes.append(monitor_process)

                # 3. rosbag play 실행
                time.sleep(5)  # roslaunch가 완전히 실행될 시간을 줌
                play_bag_process = play_bag_file(bag_file_path)

                # 6. rosbag play 종료 대기
                play_bag_process.wait()

                # 8. CPU 모니터링 종료 및 평균 계산
                monitor_process.terminate()
                monitor_process.join()

                # 7. roslaunch, rostopic echo 프로세스 종료
                terminate_process_and_children(roslaunch_process)
                terminate_process_and_children(predict_odom_process)
                terminate_process_and_children(leica_pose_process)

                average_cpu_usage = monitor_cpu_usage(target_node_name)
                if average_cpu_usage is not None:
                    cpu_file_path = os.path.join(folder_path, "cpu.txt")
                    with open(cpu_file_path, 'w') as cpu_file:
                        cpu_file.write(f"Average CPU usage for {target_node_name}: {average_cpu_usage:.2f}%\n")
                else:
                    print(f"Could not monitor CPU usage for {target_node_name}")

if __name__ == "__main__":
    root_directory = "/home/mason/bags/ntu_viral"  # 루트 디렉토리 경로를 지정
    topic_name = "/opt_odom"  # topic 이름을 지정
    target_node_name = "slict_estimator"  # 모니터링할 노드의 이름
    main(root_directory, topic_name, target_node_name)

