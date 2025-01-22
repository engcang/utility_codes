#!/bin/bash

# root_directory Bag파일이 위치한 루트 디렉토리 경로를 지정
# file_save_root_directory 결과 파일 저장 루트 디렉토리 경로를 지정

# package_name roslaunch 패키지 이름을 지정
# launch_file_name 런치 파일 이름을 지정
# odom_topic_name topic 이름을 지정
# target_process_name 모니터링할 노드의 이름: launch파일에서 node의 type!!!
# target_process_name2 모니터링할 노드의 이름: launch파일에서 node의 type!!!
# algorithm_name 알고리즘 이름을 지정, 결과 파일 이름에 사용됨
    
# 실행에 사용할 매개변수 설정
ROOT_DIRECTORIES=(
    "/media/mason/Datasets/ntu_again"
    "/media/mason/Datasets/ntu_kiss"
)
SAVE_DIRECTORIES=(
    "/media/mason/Datasets/results_eval_single/ntu_viral1"
    "/media/mason/Datasets/results_eval_single/ntu_viral1"
)
PACKAGE_NAMES=(
    "kiss_icp"
    "kiss_icp"
)
LAUNCH_FILE_NAMES=(
    "ntu1.launch"
    "ntu1.launch"
)
ODOM_TOPIC_NAMES=(
    "/kiss/odometry"
    "/kiss/odometry"
)
TARGET_PROCESS_NAMES=(
    "odometry_"
    "odometry_"
)
TARGET_PROCESS_NAMES_2=(
    "XXXXX"
    "XXXXX"
)
ALGORITHM_NAMES=(
    "kiss_icp_"
    "kiss_icp_"
)
# ROOT_DIRECTORIES=(
#     "/media/mason/Datasets/subt_mrs/ugv"
#     "/media/mason/Datasets/subt_mrs/handheld"
#     "/media/mason/Datasets/subt_mrs/longcorridor"
#     "/media/mason/Datasets/subt_mrs/multifloor"
#     "/media/mason/Datasets/NewerCollegeDataset"
#     "/media/mason/Datasets/MulRan"
#     "/media/mason/Datasets/2022hilti"
#     "/media/mason/Datasets/2021hilti"
#     "/media/mason/Datasets/2021hilti"
# )
# SAVE_DIRECTORIES=(
#     "/media/mason/Datasets/results_eval_single/subt"
#     "/media/mason/Datasets/results_eval_single/subt"
#     "/media/mason/Datasets/results_eval_single/subt"
#     "/media/mason/Datasets/results_eval_single/subt"
#     "/media/mason/Datasets/results_eval_single/ncd"
#     "/media/mason/Datasets/results_eval_single/mulran"
#     "/media/mason/Datasets/results_eval_single/2022hilti"
#     "/media/mason/Datasets/results_eval_single/2021hilti"
#     "/media/mason/Datasets/results_eval_single/2021hilti_os"
# )
# PACKAGE_NAMES=(
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
#     "kiss_icp"
# )
# LAUNCH_FILE_NAMES=(
#     "subt.launch"
#     "subt.launch"
#     "subt.launch"
#     "subt.launch"
#     "ncd.launch"
#     "mulran.launch"
#     "2022hilti.launch"
#     "2021hilti.launch"
#     "2021hilti_os.launch"
# )
# ODOM_TOPIC_NAMES=(
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
#     "/kiss/odometry"
# )
# TARGET_PROCESS_NAMES=(
#     "odometry_"
#     "odometry_"
#     "odometry_"
#     "odometry_"
#     "odometry_"
#     "odometry_"
#     "odometry_"
#     "odometry_"
#     "odometry_"
# )
# TARGET_PROCESS_NAMES_2=(
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
#     "XXXXX"
# )
# ALGORITHM_NAMES=(
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
#     "kiss_icp_"
# )

# 배열 크기 확인
length=${#ROOT_DIRECTORIES[@]}

# 순서쌍에 대해 반복 실행
for ((i=0; i<$length; i++)); do
    echo "Running experiment with:"
    echo "Root Directory: ${ROOT_DIRECTORIES[i]}"
    echo "Save Directory: ${SAVE_DIRECTORIES[i]}"
    echo "Package Name: ${PACKAGE_NAMES[i]}"
    echo "Launch File Name: ${LAUNCH_FILE_NAMES[i]}"
    echo "Odom Topic Name: ${ODOM_TOPIC_NAMES[i]}"
    echo "Target Process Name 1: ${TARGET_PROCESS_NAMES[i]}"
    echo "Target Process Name 2: ${TARGET_PROCESS_NAMES_2[i]}"
    echo "Algorithm Name: ${ALGORITHM_NAMES[i]}"

    # 파이썬 스크립트 실행
    python dataset_auto_record_lio.py \
        "${ROOT_DIRECTORIES[i]}" \
        "${SAVE_DIRECTORIES[i]}" \
        "${PACKAGE_NAMES[i]}" \
        "${LAUNCH_FILE_NAMES[i]}" \
        "${ODOM_TOPIC_NAMES[i]}" \
        "${TARGET_PROCESS_NAMES[i]}" \
        "${TARGET_PROCESS_NAMES_2[i]}" \
        "${ALGORITHM_NAMES[i]}"

    echo "Experiment ${i} completed. Moving to the next..."
    sleep 5  # 각 실행 사이에 대기 (필요 시 삭제 가능)
done

echo "All experiments completed!"
