import rospy
import psutil
import csv
import time
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Empty

class CPUUsageRecorder:
    def __init__(self, process_names, interval=1.0, save_path='.', prefix='', odom_topic='/odom', calc_time_topic='/calc_time', localizability_topic='/localizability', point_num_topic='/point_number'):
        self.process_names = process_names
        self.interval = interval
        self.save_path = save_path
        self.prefix = prefix
        self.odom_topic = odom_topic
        self.calc_time_topic = calc_time_topic
        self.localizability_topics = [f"{localizability_topic}_x", f"{localizability_topic}_y", f"{localizability_topic}_z"]
        self.point_num_topic = point_num_topic
        
        self.total_cpu_usages = []
        self.calculation_times = []
        self.localizability_values_x = []
        self.localizability_values_y = []
        self.localizability_values_z = []
        self.point_numbers = []
        self.last_odom_time = None
        self.last_cpu_record_time = None

        # Create CSV files with prefix
        self.cpu_usage_file = open(os.path.join(self.save_path, f'{self.prefix}cpu_usage.csv'), 'w', newline='')
        self.calc_time_file = open(os.path.join(self.save_path, f'{self.prefix}calculation_time.csv'), 'w', newline='')
        self.localizability_file_x = open(os.path.join(self.save_path, f'{self.prefix}localizability_x.csv'), 'w', newline='')
        self.localizability_file_y = open(os.path.join(self.save_path, f'{self.prefix}localizability_y.csv'), 'w', newline='')
        self.localizability_file_z = open(os.path.join(self.save_path, f'{self.prefix}localizability_z.csv'), 'w', newline='')
        self.point_num_file = open(os.path.join(self.save_path, f'{self.prefix}point_number.csv'), 'w', newline='')

        self.csv_writer_cpu = csv.writer(self.cpu_usage_file)
        self.csv_writer_calc_time = csv.writer(self.calc_time_file)
        self.csv_writer_localizability_x = csv.writer(self.localizability_file_x)
        self.csv_writer_localizability_y = csv.writer(self.localizability_file_y)
        self.csv_writer_localizability_z = csv.writer(self.localizability_file_z)
        self.csv_writer_point_num = csv.writer(self.point_num_file)

        self.csv_writer_cpu.writerow(['Time', 'CPU Usage (%)'])
        self.csv_writer_calc_time.writerow(['Time', 'Calculation Time'])
        self.csv_writer_localizability_x.writerow(['Time', 'Localizability X'])
        self.csv_writer_localizability_y.writerow(['Time', 'Localizability Y'])
        self.csv_writer_localizability_z.writerow(['Time', 'Localizability Z'])
        self.csv_writer_point_num.writerow(['Time', 'Point Number'])

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
    
    def calculation_time_callback(self, data):
        if self.last_odom_time is not None:
            self.calculation_times.append(data.data)
            self.csv_writer_calc_time.writerow([self.last_odom_time, data.data])
    
    def localizability_callback_x(self, data):
        if self.last_odom_time is not None:
            self.localizability_values_x.append(data.data)
            self.csv_writer_localizability_x.writerow([self.last_odom_time, data.data])
    
    def localizability_callback_y(self, data):
        if self.last_odom_time is not None:
            self.localizability_values_y.append(data.data)
            self.csv_writer_localizability_y.writerow([self.last_odom_time, data.data])
    
    def localizability_callback_z(self, data):
        if self.last_odom_time is not None:
            self.localizability_values_z.append(data.data)
            self.csv_writer_localizability_z.writerow([self.last_odom_time, data.data])
    
    def point_number_callback(self, data):
        if self.last_odom_time is not None:
            self.point_numbers.append(data.data)
            self.csv_writer_point_num.writerow([self.last_odom_time, data.data])

    def start(self):
        rospy.init_node('cpu_usage_recorder', anonymous=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.calc_time_topic, Float32, self.calculation_time_callback)
        rospy.Subscriber(self.localizability_topics[0], Float32, self.localizability_callback_x)
        rospy.Subscriber(self.localizability_topics[1], Float32, self.localizability_callback_y)
        rospy.Subscriber(self.localizability_topics[2], Float32, self.localizability_callback_z)
        rospy.Subscriber(self.point_num_topic, Float32, self.point_number_callback)
        # In the __init__ method, add the publisher for the Empty message
        self.no_odom_publisher = rospy.Publisher('/no_odom', Empty, queue_size=10)
        
        try:
            while not rospy.is_shutdown():
                current_time = time.time()
                
                if self.last_odom_time and (current_time - self.last_odom_time > 1.0):
                    rospy.loginfo("No Odometry data received for 1 second, stopping measurement.")
                    empty_msg = Empty()
                    self.no_odom_publisher.publish(empty_msg)  # Publish an Empty message
                    break
                
                time.sleep(self.interval)
        
        except KeyboardInterrupt:
            rospy.loginfo("Measurement stopped by user.")
        
        finally:
            self.cpu_usage_file.close()
            self.calc_time_file.close()
            self.localizability_file_x.close()
            self.localizability_file_y.close()
            self.localizability_file_z.close()
            self.point_num_file.close()
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
        
        if self.localizability_values_x:
            final_average_localizability_x = sum(self.localizability_values_x) / len(self.localizability_values_x)
            with open(os.path.join(self.save_path, f'{self.prefix}average_localizability_x.txt'), 'w') as f:
                f.write(f"Average Localizability X: {final_average_localizability_x}\n")

        if self.localizability_values_y:
            final_average_localizability_y = sum(self.localizability_values_y) / len(self.localizability_values_y)
            with open(os.path.join(self.save_path, f'{self.prefix}average_localizability_y.txt'), 'w') as f:
                f.write(f"Average Localizability Y: {final_average_localizability_y}\n")

        if self.localizability_values_z:
            final_average_localizability_z = sum(self.localizability_values_z) / len(self.localizability_values_z)
            with open(os.path.join(self.save_path, f'{self.prefix}average_localizability_z.txt'), 'w') as f:
                f.write(f"Average Localizability Z: {final_average_localizability_z}\n")

        if self.point_numbers:
            final_average_point_num = sum(self.point_numbers) / len(self.point_numbers)
            with open(os.path.join(self.save_path, f'{self.prefix}average_point_number.txt'), 'w') as f:
                f.write(f"Average Point Number: {final_average_point_num}\n")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) != 9:
        print("Usage: cpu_usage_recorder.py <save_path> <prefix> <process_name1> <process_name2> <odom_topic> <calc_time_topic> <localizability_topic> <point_num_topic>")
        sys.exit(1)
    
    save_path = sys.argv[1]
    prefix = sys.argv[2]
    process_names = [sys.argv[3], sys.argv[4]]  # Always expect two process names
    odom_topic = sys.argv[5]
    calc_time_topic = sys.argv[6]
    localizability_topic = sys.argv[7]
    point_num_topic = sys.argv[8]

    recorder = CPUUsageRecorder(process_names, interval=1.0, save_path=save_path, prefix=prefix, odom_topic=odom_topic, calc_time_topic=calc_time_topic, localizability_topic=localizability_topic, point_num_topic=point_num_topic)
    recorder.start()
