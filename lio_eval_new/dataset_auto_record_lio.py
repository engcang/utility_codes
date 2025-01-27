import os
import subprocess
import time
import sys
import signal
import rospy
from std_msgs.msg import Empty

processes = []
callback_received_ready_to_terminate = False

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
    global processes
    print('You pressed Ctrl+C! Terminating processes...')
    for p in processes:
        terminate_process_and_children(p)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def no_odom_callback(data):
    global callback_received_ready_to_terminate
    """ /no_odom 토픽 메시지를 받으면 프로세스 종료 """
    rospy.loginfo("Received /no_odom message. Ready to terminate processes...")
    callback_received_ready_to_terminate = True

def run_roslaunch(package_name, launch_file_name):
    global processes
    """roslaunch 명령어를 실행"""
    roslaunch_cmd = ["roslaunch", package_name, launch_file_name]
    roslaunch_process = subprocess.Popen(roslaunch_cmd, preexec_fn=os.setsid)
    processes.append(roslaunch_process)
    return roslaunch_process

def play_bag_file(bag_file_path):
    global processes
    """rosbag play 명령어를 실행"""
    play_bag_cmd = ["rosbag", "play", bag_file_path, "--clock"]
    play_bag_process = subprocess.Popen(play_bag_cmd, preexec_fn=os.setsid)
    processes.append(play_bag_process)
    return play_bag_process

def play_bag_file_offset(bag_file_path, offset):
    global processes
    """rosbag play 명령어를 실행"""
    play_bag_cmd = ["rosbag", "play", bag_file_path, "--clock", "-s", str(offset)]
    play_bag_process = subprocess.Popen(play_bag_cmd, preexec_fn=os.setsid)
    processes.append(play_bag_process)
    return play_bag_process

def run_monitor_code(folder_path, algorithm_name, target_process_name, target_process_name2, odom_topic_name):
    global processes
    """모니터 코드를 실행"""
    code999_cmd = ["python", "cpu_calct_loc_ptnum_monitor.py", folder_path, algorithm_name, target_process_name, target_process_name2, odom_topic_name, "/calc_time", "/localizability"]
    code999_process = subprocess.Popen(code999_cmd, preexec_fn=os.setsid)
    processes.append(code999_process)
    return code999_process

def run_livox_convert_code():
    global processes
    """모니터 코드를 실행"""
    livox_convert_cmd = ["python", "livox_to_ouster.py"]
    livox_convert_process = subprocess.Popen(livox_convert_cmd, preexec_fn=os.setsid)
    processes.append(livox_convert_process)
    return livox_convert_process

def main(root_directory, file_save_root_directory, package_name, launch_file_name, algorithm_name, odom_topic_name, target_process_name, target_process_name2):
    global callback_received_ready_to_terminate
    # ROS 노드 초기화
    rospy.init_node('dataset_auto_record_lio', anonymous=True)
    rospy.Subscriber("/no_odom", Empty, no_odom_callback)
    
    for folder in sorted(os.listdir(root_directory)):
        folder_path = os.path.join(root_directory, folder)
        file_save_path = os.path.join(file_save_root_directory, folder)
        if os.path.isdir(folder_path):
            processes.clear()  # Clear the processes list at the start of each iteration
            bag_file_path = os.path.join(folder_path, f"{folder}.bag")

            velodyne_roslaunch_process = None
            livox_convert_process = None
            play_bag_process = None
            
            if os.path.exists(bag_file_path):
                # 1. roslaunch 실행
                if "subt" in folder_path or "subt" in file_save_path:
                    velodyne_roslaunch_process = run_roslaunch("velodyne_pointcloud", "VLP16_points.launch")
                    time.sleep(2)
                if "direct_lidar" in package_name or "kiss_icp" in package_name or "genz_icp" in package_name:
                    livox_convert_process = run_livox_convert_code()
                    time.sleep(2)
                roslaunch_process = run_roslaunch(package_name, launch_file_name)
                time.sleep(2)  # roslaunch가 완전히 실행될 시간을 줌

                # 3번과 4번 사이에 모니터 코드 실행
                code999_process = run_monitor_code(file_save_path, algorithm_name, target_process_name, target_process_name2, odom_topic_name)
                
                # 4. rosbag play 실행
                time.sleep(2)  # roslaunch가 완전히 실행될 시간을 줌
                if "subt" in folder_path and "Long_Corridor" in bag_file_path:
                    play_bag_process = play_bag_file_offset(bag_file_path, 1.5)
                else:
                    play_bag_process = play_bag_file(bag_file_path)

                # 5. rosbag play 종료 대기
                play_bag_process.wait()

                # 6. Bag 파일 재생이 끝나면 3초 대기 후 프로세스 종료
                rospy.sleep(2)
                if velodyne_roslaunch_process is not None:
                    terminate_process_and_children(velodyne_roslaunch_process)
                if livox_convert_process is not None:
                    terminate_process_and_children(livox_convert_process)
                terminate_process_and_children(roslaunch_process)
                terminate_process_and_children(code999_process)

if __name__ == "__main__":
    if len(sys.argv) != 9:
        print("Usage: dataset_auto_record_lio.py <root_directory> <file_save_root_directory> <package_name> <launch_file_name> <odom_topic_name> <target_process_name> <target_process_name2> <algorithm_name>")
        sys.exit(1)
    
    root_directory = sys.argv[1]  # Bag파일이 위치한 루트 디렉토리 경로를 지정
    file_save_root_directory = sys.argv[2]  # 결과 파일 저장 루트 디렉토리 경로를 지정

    package_name = sys.argv[3]  # 패키지 이름을 지정
    launch_file_name = sys.argv[4]  # 런치 파일 이름을 지정
    odom_topic_name = sys.argv[5]  # topic 이름을 지정
    target_process_name = sys.argv[6]  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    target_process_name2 = sys.argv[7]  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    algorithm_name = sys.argv[8]  # 알고리즘 이름을 지정, 결과 파일 이름에 사용됨

    main(root_directory, file_save_root_directory, package_name, launch_file_name, algorithm_name, odom_topic_name, target_process_name, target_process_name2)