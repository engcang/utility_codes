# LIO Evaluation scripts 사용법
- NTU_VIRAL dataset, NewerCollegeDataset을 예시로 설명

<br>

# 주의 사항
- 메인으로 실행되는 dataset_auto_record_lio.py 파일 내에 다음과 같은 부분이 있다.
- 즉, 아래 3번에서 설명하는 개별 알고리즘 실행 스크립트에서, 데이터셋 폴더 이름이나 파일 저장 경로에 `subt`가 포함되면 `velodyne_pointcloud` 패키지의 `VLP16_points.launch` 변환 코드를 자동 실행하게 되어있다.
- 해당 패키지를 설치하든가, 아니면 주석해야함.
- 마찬가지로 실행하는 패키지 이름에 `direct_lidar`, `kiss_icp`, `genz_icp`가 포함되면 livox_to_ouster.py 파일이 실행된다.
- `livox_ros_driver` 패키지를 설치하든가, 아니면 주석해야함. 또, 이왕 쓸거면 livox_to_ouster.py 코드 내에서 입력과 출력하는 토픽의 이름을 잘 확인해서 수정
    ```python
    if "subt" in folder_path or "subt" in file_save_path:
        velodyne_roslaunch_process = run_roslaunch("velodyne_pointcloud", "VLP16_points.launch")
        time.sleep(2)
    if "direct_lidar" in package_name or "kiss_icp" in package_name or "genz_icp" in package_name:
        livox_convert_process = run_livox_convert_code()
        time.sleep(2)
    ```
- SubtMRS 데이터셋은 같은 데이터셋이어도 시퀀스별로 extrinsic 등 파라미터가 달라져서 알고리즘의 config 파일을 다르게 설정해야한다.
- 이걸 유념해서 launch 파일도 여러개 만들고, extrinsic이 같은 시퀀스끼리 묶어서 잘 보관해두어야 3번에서 말하는 개별 알고리즘 실행 스크립트에서 정상적으로 알고리즘을 실행하고, 결과를 기록한다. 

<br>
<br>

### 0. 실행 구조
- roscore를 터미널 1에 실행 시킨다.
- run_everything.sh 스크립트 파일을 터미널 2에 실행 시킨다.
- run_everything.sh 스크립트 파일은 각 알고리즘의 run 파일을 실행 시킨다. 예) run_fast_lio.sh, run_ada_lio.sh
- 각 알고리즘의 run 파일은 각 알고리즘을 원하는 데이터셋 * 모든 시퀀스에 대해 실행하고, 동시에 위치 추정치, 연산 시간, CPU 소모량을 기록한다.
    - 각 run 파일에서는 dataset_auto_record_lio.py 파일을 실행시켜서 알고리즘 수행, bag 파일 재생, 로깅 코드를 실행한다
        - dataset_auto_record_lio.py 파일에서 cpu_calct_loc_ptnum_monitor.py 파일을 실행해서 로깅을 수행한다.
    - 연산 시간의 경우, 알고리즘의 소스코드를 수정해서 연산 시간을 ms 단위로 /calc_time이라는 std_msgs/Float32 type의 토픽으로 쏴주어야한다.
- 이후, 각 데이터셋별로 모든 시퀀스 * 모든 알고리즘에 대해 자동으로 `evo`를 실행해서 APE나 RPE 값을 표로 생성하는 코드 evo_autorun.py가 있으므로 해당 코드를 활용하면 된다.

### 1. 데이터셋 폴더 준비
- 데이터셋은 각 데이터셋별로 /media/mason/Datasets/ntu_viral와 같이 한 폴더 안에 시퀀스 이름으로 위치 시킨다.
    - 즉, /media/mason/Datasets/ntu_viral/eee_01, /media/mason/Datasets/ntu_viral/eee_02, /media/mason/Datasets/ntu_viral/eee_03 이런식으로..
    - 그리고 /media/mason/Datasets/NewerCollegeDataset/01_short_experiment, /media/mason/Datasets/NewerCollegeDataset/02_long_experiment 이런식으로...
- 그리고 각 폴더 안에는 폴더이름.bag 으로 bag파일을 위치한다.
    - /media/mason/Datasets/ntu_viral/eee_01/eee_01.bag 이런식으로...
    - /media/mason/Datasets/NewerCollegeDataset/01_short_experiment/01_short_experiment.bag 이런식으로...
- 꼭 시퀀스 이름일 필요는 없고, 이름은 내 마음대로 지정해도 된다. 중요한 것은 데이터셋/폴더이름/폴더이름.bag 구조이다.
    - /media/mason/Datasets/dataset1/sequence1/sequence1.bag
    - /media/mason/Datasets/dataset1/sequence2/sequence2.bag
    - /media/mason/Datasets/dataset1/any_name/any_name.bag

### 2. 결과 폴더 준비
- 데이터셋별로 위치 추정 알고리즘을 돌린 결과도 /media/mason/Datasets/results_eval_single/ncd와 같이 한 폴더 안에 시퀀스 이름으로 저장되게 만들어두었다.
    - 즉, /media/mason/Datasets/results_eval_single/ncd/01_short_experiment, /media/mason/Datasets/results_eval_single/ncd/02_long_experiment와 같이 폴더를 만들어두어야 한다. 
    - ***폴더도 자동으로 만들게 코드 고치면 되는데 왜 안했을까...***
- 여기서 중요한 것은 각 시퀀스 폴더 이름이 데이터셋 폴더 준비 단계에서 생성한 시퀀스 폴더 이름과 같아야 한다는 것이다.
    - 그리고 데이터셋별로 같은 폴더에 있으면 된다. 즉, 데이터셋/폴더이름 구조이다.
    - 즉, /아무런_폴더나_경로여도_됨/데이터셋/01_short_experiment, /아무런_폴더나_경로여도_됨/데이터셋/02_long_experiment

### 3. 개별 알고리즘의 스크립트 파일 준비 및 이해
- run_fast_lio.sh 파일을 예시로 설명.
- 따로 설명 하지 않을테니, 아래 주석을 읽어보기 바람.
    ```sh
    #!/bin/bash
        
    # Bag파일들이 위치한 각 데이터셋의 루트 경로, 해당 경로 내에 있는 모든 시퀀스에 대해 자동 수행함
    ROOT_DIRECTORIES=(
        "/media/mason/Datasets/NewerCollegeDataset"
        "/media/mason/Datasets/ntu_viral"
    )
    # 결과 파일을 저장할 데이터셋별 경로, 해당 경로 내에 시퀀스이름으로 폴더가 있어야함
    SAVE_DIRECTORIES=(
        "/media/mason/Datasets/results_eval_single/ncd"
        "/media/mason/Datasets/results_eval_single/ntu_viral"
    )
    # 패키지 이름, 즉 roslaunch fast_lio ncd.launch로 터미널에서 실행할 때, 인식되는 패키지 이름 -> fast_lio
    PACKAGE_NAMES=(
        "fast_lio"
        "fast_lio"
    )
    # 각 데이터셋에 대해 실행할 런치 파일 이름
    LAUNCH_FILE_NAMES=(
        "ncd.launch"
        "ntu.launch"
    )
    # 알고리즘에서 출력하는 nav_msgs/Odometry type의 topic 이름. 이 topic을 subscribe해서 결과를 기록한다.
    ODOM_TOPIC_NAMES=(
        "/Odometry"
        "/Odometry"
    )
    # CPU 사용량을 모니터링 하기 위한 노드 이름, 이 이름은 launch file에서 node type과 동일하다.
    # <node pkg="fast_lio" type="fastlio_mapping" name="fast_lio_node"/> 일때, type에 해당하는 fastlio_mapping
    TARGET_PROCESS_NAMES=(
        "fastlio_mapping"
        "fastlio_mapping"
    )
    # 한 알고리즘에서 여러 개의 노드를 수행하는 경우가 있어서 두 개 노드를 로깅하기 위함. 없는 경우 프로세스 중에 존재하지 않을만한 문자열 입력
    TARGET_PROCESS_NAMES_2=(
        "XXXXXXXXXXXXXXX"
        "XXXXXXXXXXXXXXX"
    )
    # save_directories/시퀀스이름/알고리즘이름odom.csv 형식으로 저장됨. 즉, 원하는 prefix를 입력. 이 때, _를 뒤에 붙여야함.
    ALGORITHM_NAMES=(
        "baseline_"
        "baseline_"
    )

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
    ```

### 4. 전체 실행 스크립트의 준비와 이해
- 전체 알고리즘 실행 스크립트는 아주 간단하다. 따로 설명이 필요 없지 싶다.
- 아래와 같이 실행을 원하는 스크립트들을 전부 적어주면 알아서 실행된다.
    ```sh
    #!/bin/bash

    ./run_fast_lio.sh
    sleep 3
    ./run_ada_lio.sh
    sleep 3
    ./run_kiss_icp.sh

    echo "All experiments completed!"
    ```

<br>
<br>

### 5. 자동 결과 분석 코드
- 위의 0~4번대로 수행하면 결과 저장 폴더에 각 알고리즘별로 알고리즘_odom.csv라는 이름으로 위치 추정치가 TUM 형식으로 기록된다.
- 자동 결과 분석 코드를 사용하기 전, `EVO`가 설치되어있는지, 가상환경에 설치했다면 activate 했는지 확인하자.
- python evo_autorun.py을 수행하면 해당 파일이 위치한 폴더안에 있는 모든 시퀀스 이름의 폴더에 대해, 지정해둔 prefix들에 대해 prefix_odom.csv 파일을 읽어와서 `EVO`의 APE 혹은 RPE를 계산해서 표 형태로 csv 파일로 기록한다.
    - 즉, 해당 파일이 위치한 폴더에 01_short_experiment, 02_short_experiment, 05_quad_with_dynamic 폴더가 존재.
        - 예) /media/mason/Datasets/results_eval_single/ncd 폴더 안에 evo_autorun.py 파일과 각 폴더들이 존재.
    - 각 폴더 안에는 tum_gt.csv 라는 이름으로 ground truth가 TUM 형식으로 기록되어있어야 함.
    - TUM 형식의 ground truth를 생성하는 코드는 ncd_gt_to_tum.py 등의 파일을 읽어보고 적절히 실행하면 된다.
- 필요에 따라 해당 파일 내에 base_folder와 prefixes 변수의 값을 수정하면 된다.
- 추가로, 연산 시간이나 CPU 소모량도 함께 기록되어있다면 python calc_average.py 폴더이름 prefix_를 실행하면 해당 알고리즘의 해당 폴더 안에 있는 각 시퀀스별로 소모한 CPU 소모량을 읽어와서 평균을 계산해준다.
    - 연산 시간의 경우, 해당 알고리즘별로 코드를 수정해서 std_msgs/Float32 type으로 /calc_time이라는 이름의 topic에 ms 단위로 연산 시간을 publish하게 수정해야한다.
    - https://github.com/engcang/slam-application 사이트에 각 폴더 안에 있는 알고리즘의 소스코드 참고.
