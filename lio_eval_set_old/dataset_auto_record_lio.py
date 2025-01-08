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

def run_roslaunch():
    global processes
    """roslaunch 명령어를 실행"""
    #roslaunch_cmd = ["roslaunch", "locus", "hilti2021.launch"]
    #roslaunch_cmd = ["roslaunch", "slict", "run_hilti2021.launch"]
    # roslaunch_cmd = ["roslaunch", "mloam", "mloam_ntuviral.launch"]
    #roslaunch_cmd = ["roslaunch", "ma_lio", "hilti2021.launch"]
    roslaunch_cmd = ["roslaunch", "fast_lio_multi", "run.launch"]
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

def run_rostopic_echo(topic, output_file):
    global processes
    """rostopic echo 명령어를 실행"""
    rostopic_cmd = ["rostopic", "echo", "-p", "--nostr", "--noarr", topic]
    with open(output_file, 'w') as f:
        rostopic_process = subprocess.Popen(rostopic_cmd, stdout=f, preexec_fn=os.setsid)
    processes.append(rostopic_process)
    return rostopic_process

def run_monitor_code(folder_path, target_process_name, target_process_name2, odom_topic_name):
    global processes
    """모니터 코드를 실행"""
    code999_cmd = ["python", "cpu_calct_loc_ptnum_monitor.py", folder_path, '', target_process_name, target_process_name2, odom_topic_name, "/calc_time", "/localizability", "/point_number", "/velocity", "/acc_bias"]
    code999_process = subprocess.Popen(code999_cmd, preexec_fn=os.setsid)
    processes.append(code999_process)
    return code999_process

def main(root_directory, odom_topic_name, gt_topic_name, target_process_name, target_process_name2):
    global callback_received_ready_to_terminate
    # ROS 노드 초기화
    rospy.init_node('dataset_auto_record_lio', anonymous=True)
    rospy.Subscriber("/no_odom", Empty, no_odom_callback)
    
    for folder in os.listdir(root_directory):
        folder_path = os.path.join(root_directory, folder)
        if os.path.isdir(folder_path):
            processes.clear()  # Clear the processes list at the start of each iteration
            bag_file_path = os.path.join(folder_path, f"{folder}.bag")
                            
            if os.path.exists(bag_file_path):
                # 1. roslaunch 실행
                roslaunch_process = run_roslaunch()
                time.sleep(3)  # roslaunch가 완전히 실행될 시간을 줌

                # 2. rostopic echo (topic) 실행
                predict_odom_file = os.path.join(folder_path, "predict_odom.csv")
                predict_odom_process = run_rostopic_echo(odom_topic_name, predict_odom_file)

                # 3. rostopic echo (/leica/pose/relative) 실행
                leica_pose_file = os.path.join(folder_path, "leica_pose.csv")
                leica_pose_process = run_rostopic_echo(gt_topic_name, leica_pose_file)

                # 3번과 4번 사이에 모니터 코드 실행
                code999_process = run_monitor_code(folder_path, target_process_name, target_process_name2, odom_topic_name)
                
                # 4. rosbag play 실행
                time.sleep(3)  # roslaunch가 완전히 실행될 시간을 줌
                play_bag_process = play_bag_file(bag_file_path)

                # 5. rosbag play 종료 대기
                play_bag_process.wait()

                # 6. /no_odom 메시지를 대기하다가 수신 시 프로세스 종료
                while not callback_received_ready_to_terminate:
                    rospy.sleep(1)
                # 콜백 수신 후 프로세스 종료
                if callback_received_ready_to_terminate:
                    terminate_process_and_children(roslaunch_process)
                    terminate_process_and_children(predict_odom_process)
                    terminate_process_and_children(leica_pose_process)
                    terminate_process_and_children(code999_process)
                    callback_received_ready_to_terminate = False # 플래그 리셋

if __name__ == "__main__":
    root_directory = "/home/mason/bags/2021hilti"  # 루트 디렉토리 경로를 지정
    gt_topic_name = "/leica/pose/relative"  # topic 이름을 지정

    # async, bundle, ours
    odom_topic_name = "/Odometry"  # topic 이름을 지정
    target_process_name = "fast_lio"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    target_process_name2 = "ASUDFASUFDSUFSDAFU"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    
    # MA-LIO
    #odom_topic_name = "/Odometry"  # topic 이름을 지정
    #target_process_name = "malio_mapping"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    #target_process_name2 = "ASUDFASUFDSUFSDAFU"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!

    # slict1
    #odom_topic_name = "/opt_odom"  # topic 이름을 지정
    #target_process_name = "slict_esti"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    #target_process_name2 = "slict_sensor"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!

    # locus2
    # odom_topic_name = "/ntuviral/locus/odometry"  # topic 이름을 지정
    #odom_topic_name = "/hilti2021/locus/odometry"  # topic 이름을 지정
    #target_process_name = "locus"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    #target_process_name2 = "ASUDFASUFDSUFSDAFU"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    
    # mloam
    # odom_topic_name = "/laser_map_high_frec"  # topic 이름을 지정
    # target_process_name = "mloam"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!
    # target_process_name2 = "lidar_mapper"  # 모니터링할 노드의 이름: launch파일에서 node의 type!!!

    main(root_directory, odom_topic_name, gt_topic_name, target_process_name, target_process_name2)
