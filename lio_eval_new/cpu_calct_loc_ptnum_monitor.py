import rospy
import psutil
import csv
import time
import os
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3

class CPUUsageRecorder:
    def __init__(self, process_names, interval=1.0, save_path='.', prefix='', odom_topic='/odom', calc_time_topic='/calc_time', localizability_topic='/localizability'):
        self.process_names = process_names
        self.interval = interval
        self.save_path = save_path
        self.prefix = prefix
        self.odom_topic = odom_topic
        self.calc_time_topic = calc_time_topic
        self.localizability_topic = localizability_topic
        
        self.total_cpu_usages = []
        self.calculation_times = []
        self.last_odom_time = None
        self.last_cpu_record_time = None

        # Create CSV files with prefix
        self.odom_file = open(os.path.join(self.save_path, f'{self.prefix}odom.csv'), 'w', newline='')
        self.cpu_usage_file = open(os.path.join(self.save_path, f'{self.prefix}cpu_usage.csv'), 'w', newline='')
        self.calc_time_file = open(os.path.join(self.save_path, f'{self.prefix}calculation_time.csv'), 'w', newline='')
        self.localizability_file = open(os.path.join(self.save_path, f'{self.prefix}localizability.csv'), 'w', newline='')

        self.csv_writer_odom = csv.writer(self.odom_file, delimiter=' ')
        self.csv_writer_cpu = csv.writer(self.cpu_usage_file)
        self.csv_writer_calc_time = csv.writer(self.calc_time_file)
        self.csv_writer_localizability = csv.writer(self.localizability_file)

        self.csv_writer_cpu.writerow(['Time', 'CPU Usage (%)'])
        self.csv_writer_calc_time.writerow(['Time', 'Calculation Time'])

    def get_process_cpu_usage(self):
        cpu_usage_sum = 0
        process_count = 0
        
        for process_name in self.process_names:
            target_process = None
            for proc in psutil.process_iter(['pid', 'name']):
                if process_name in proc.info['name']:
                    target_process = proc
                    break
            
            if target_process:
                cpu_usage = target_process.cpu_percent(interval=None)
                cpu_usage_sum += cpu_usage
                process_count += 1
        
        if process_count > 0:
            return cpu_usage_sum
        return None

    def odom_callback(self, data):
        self.first_odom_callback_in = True
        current_time = time.time()    
        # Check if 1 second has passed since the last CPU usage record
        if self.last_cpu_record_time is None or (current_time - self.last_cpu_record_time) >= 1.0:
            cpu_usage = self.get_process_cpu_usage()
            if cpu_usage is not None:
                self.total_cpu_usages.append(cpu_usage)
                self.csv_writer_cpu.writerow([current_time, cpu_usage])
            self.last_cpu_record_time = current_time
        self.last_odom_time = current_time

        # Extract odometry data and write to TUM format
        timestamp = data.header.stamp.to_sec()  # Use data's timestamp
        tx = data.pose.pose.position.x
        ty = data.pose.pose.position.y
        tz = data.pose.pose.position.z
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        if timestamp > 0:
            self.csv_writer_odom.writerow([timestamp, tx, ty, tz, qx, qy, qz, qw])

    def calculation_time_callback(self, data):
        if self.last_odom_time is not None:
            self.calculation_times.append(data.data)
            self.csv_writer_calc_time.writerow([self.last_odom_time, data.data])
    
    def localizability_callback(self, data):
        if self.last_odom_time is not None:
            self.csv_writer_localizability.writerow([self.last_odom_time, data.x, data.y, data.z])
    
    def start(self):
        rospy.init_node('cpu_usage_recorder', anonymous=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.calc_time_topic, Float32, self.calculation_time_callback)
        rospy.Subscriber(self.localizability_topic, Vector3, self.localizability_callback)
        # In the __init__ method, add the publisher for the Empty message
        self.no_odom_publisher = rospy.Publisher('/no_odom', Empty, queue_size=10)
        
        try:
            while not rospy.is_shutdown():
                time.sleep(self.interval)
        except KeyboardInterrupt:
            rospy.loginfo("Measurement stopped by user.")
        finally:
            self.cpu_usage_file.close()
            self.calc_time_file.close()
            self.localizability_file.close()
            self.save_final_results()
    
    def save_final_results(self):
        if self.total_cpu_usages:
            final_average_cpu_usage = sum(self.total_cpu_usages) / len(self.total_cpu_usages)
            with open(os.path.join(self.save_path, f'{self.prefix}average_cpu_usage.txt'), 'w') as f:
                f.write(f"Average CPU usage: {final_average_cpu_usage}%\n")
        if self.calculation_times:
            final_average_calc_time = sum(self.calculation_times) / len(self.calculation_times)
            with open(os.path.join(self.save_path, f'{self.prefix}average_calculation_time.txt'), 'w') as f:
                f.write(f"Average Calculation Time: {final_average_calc_time}\n")

if __name__ == '__main__':
    if len(sys.argv) != 8:
        print("Usage: cpu_usage_recorder.py <save_path> <prefix> <process_name1> <process_name2> <odom_topic> <calc_time_topic> <localizability_topic>")
        sys.exit(1)
    
    save_path = sys.argv[1]
    prefix = sys.argv[2]
    process_names = [sys.argv[3], sys.argv[4]]  # Always expect two process names
    odom_topic = sys.argv[5]
    calc_time_topic = sys.argv[6]
    localizability_topic = sys.argv[7]

    recorder = CPUUsageRecorder(process_names, interval=1.0, save_path=save_path, prefix=prefix, odom_topic=odom_topic, 
                                calc_time_topic=calc_time_topic, localizability_topic=localizability_topic)
    recorder.start()
