#!/bin/bash

# 현재 디렉토리에서 실행 가능한 모든 .sh 파일 탐색 및 실행
for script in ./*.sh; do
    if [[ -x "$script" ]]; then  # 실행 가능한 파일인지 확인
        echo "Running: $script"
        "$script"
        sleep 5  # 실행 사이의 대기 시간
        echo "Completed: $script"
    else
        echo "Skipping: $script (not executable)"
    fi
done

echo "All experiments completed!"