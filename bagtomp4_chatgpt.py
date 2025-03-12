import os
import sys
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from tqdm import tqdm

def rosbag_to_mp4(bag_file, topic_name, output_file="output.mp4", fps=30):
    # ROSBag 열기
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    
    # 이미지 프레임 수집
    frames = []
    timestamps = []
    
    for topic, msg, t in tqdm(bag.read_messages(topics=[topic_name]), desc="Reading ROSBag"):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frames.append(cv_image)
            timestamps.append(t.to_sec())
        except Exception as e:
            print(f"Error processing frame: {e}")
    
    bag.close()

    if not frames:
        print("No frames extracted from ROSBag.")
        return

    # 해상도 설정 (첫 번째 프레임 기준)
    height, width, _ = frames[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    # 프레임을 동영상으로 저장
    for frame in tqdm(frames, desc="Writing Video"):
        out.write(frame)

    out.release()
    print(f"Video saved as {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("사용법: python code.py <rosbag 파일> <토픽 이름>")
        sys.exit(1)

    bag_file = sys.argv[1]
    topic_name = sys.argv[2]
    
    output_file = os.path.splitext(bag_file)[0] + ".mp4"
    rosbag_to_mp4(bag_file, topic_name, output_file)

