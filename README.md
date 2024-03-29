# utility_codes
A collection of various utility codes coded myself(most) and collected by Googling

<br>

+ cv_cam: OpenCV camera code C++ 
+ cv_cam_ros: OpenCV camera code for ROS C++
+ CV_Cam.py: OpenCV camera code Python
+ cv_cam_ros.py: OpenCV camera code Python for ROS publish with /dev/video 
+ ros_picam.py: OpenCV camera code Python for ROS publish with nvargus options (e.g. more compatible for picamera, CSI cameras also)
+ cv_cam_16bit.py: OpenCV camera code Python for 16bit(14bit) images e.g., thermal cameras
+ cam_16bit_ros.py: OpenCV camera code ROS Python for 16bit(14bit) images e.g., thermal cameras
+ ros_img_callback_save.py: callback ROS image and save as .jpg
+ image_GPS_log.py: Write GPS data to image with EXIF library
+ rgb2rgba_texture.py: rgb 3 channel image to rgba 4 channel image to use it as texture in Gazebo model

---

<br>

+ cv_images_to_mp4.py: read images in folder and save as mp4, OpenCV Python
+ mp4_to_ros.py: read mp4 using OpenCV and publish as ROS topic
+ compressed_to_raw.py: compressed image to raw image ROS(ROS basic package already exists but...)
+ raw_to_compressed.py: raw image to compressed image ROS(ROS basic package already exists but...)
+ rgb2gray.py: rgb to gray image OpenCV and ROS publish

---

<br>

+ mp4tobag.py: read mp4 using OpenCV and save as ROS bag file
+ bag2img.py: read rosbag and save image files
+ rosbag_topic_name_changer.py: renaming code for topic in ROSbag file
+ csvtobag_IMU.py: read csv file of IMU data and save as ROS bagfile
+ path_bag_to_txt.py: read ROS bag files with nav_msgs/Path and write .csv file for pose estimation performance comparison in EVO or rpg-evaluation
+ nav_odometry_bag_to_txt.py: read ROS bag files with nav_msgs/Odometry and write .csv file for pose estimation performance comparison in EVO or rpg-evaluation
+ merge_all_bagfiles.py: merge all bag files in the directory

---

<br>

+ octomap_grapher: OctoMap subscribing and then calculating the volume mapped
+ octopus: OctoMap build codes

---


<br>

+ gazebo_gt_pub.py: subscribe gazebo/model_states (GT) and publish every GT as PoseStamped topic
+ gazebo_to_path_and_path_frame.py: gazebo/model_states ground truth position to nav_msgs/path topic for visualizing and pose estimation performance
+ rviz_path.py: subscribe gazebo/model_states ground truth position to publish it as nav_msgs/Path topic and also publising PX4-SITL drone pose, and Marker
+ dae_line_remover.py: line removing code for .dae extension file made by Google Sketchup to use it in Gazebo

---


<br>

+ marker_path_length.py: length of marker in ROS
+ nav_path_length.py: length of nav_msgs/Path in ROS

---


<br>

+ tr_broadcaster.py: ros tf message broadcasting code in python
+ tf_and_vision.zip: getting /tf message and sending it to /mavros/vision_pose/pose to fly without GPS
+ gt_vision_pose.py: gazebo/model_states ground truth to /mavros/vision_pose/pose for PX4-SITL, instead of GPS
+ slam_vision_pose.py: subscribe SLAM result Odometry and publish it to /mavros/vision_pose/pose for fly without GPS

---


<br>

+ ser_pub_odom_sub_cmd.py: USB to USB ubuntu serial communication to send odometry from ROSBOT and receive command
+ ser_sub_odom_pub_cmd.py: USB to USB ubuntu serial communication to receive odometry from ROSBOT and send command to it
+ ser_rec_test.py: USB to USB ubuntu serial communication test code, receving
+ ser_trans_test.py: USB to USB ubuntu serial communication test code, transmitting

---


<br>

+ filename_convert.py: file renaming code
+ image_name_converting.py: image name converting
+ image_name_converting_and_resizing.py: image name converting and resizing
+ labeling_edit.py: labelled file edit for training YOLO
+ control_jetbot_imu.py: jetbot control code using only IMU since it does not have encoders
+ Runge_Kutta.m: runge-kutta method code for MATLAB
+ python_terminal.py: process open and kill with Python
+ bbox_vis.py: visualize 3d bounding box in ROS for rviz

---

