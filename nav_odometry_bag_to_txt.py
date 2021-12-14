import os, sys
import time
import rosbag

if (len(sys.argv) == 2):
    listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	#get list of only bag files in current dir.
    numberOfFiles = str(len(listOfBagFiles))
    print "reading " + numberOfFiles + " bagfiles in current directory: \n"
    for f in listOfBagFiles:
        print f
    print "\n press ctrl+c in the next 1 seconds to cancel \n"
    time.sleep(1)
    desired_topic_name = sys.argv[1]
else:
    print ("argument is not supported yet, usage: python nav_odometry_bag_to_txt.py /topic_name --> automatically find bag files in the same folder and generate csv files")
    sys.exit(1)


if not os.path.exists("csv_files"):
    os.makedirs("csv_files")
    print("csv_files folder generated...")
else:
    print("csv_files folder already exists, using that folder...")
    
count = 0
for bagFile in listOfBagFiles:
    count += 1
    print "reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile
    #access bag
    bag = rosbag.Bag(bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename

    topic_name = desired_topic_name.split('/')
    if len(topic_name)>2:
        f=open("csv_files/" + bagName[:-4] + "_" + topic_name[1] + "_" + topic_name[2] + '.csv', "a")
    else:
        f=open("csv_files/" + bagName[:-4] + "_" + topic_name[1] + '.csv', "a")

    for subtopic, msg, t in bag.read_messages():
        if subtopic==desired_topic_name:
            tmp_time = float((msg.header.stamp).secs) + float((msg.header.stamp).nsecs)/1000000000.0
            f.write("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n"%(tmp_time , msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.z, msg.pose.pose.orientation.y, msg.pose.pose.orientation.w))
    f.close()
    bag.close()
print "Done reading all " + numberOfFiles + " bag files."
