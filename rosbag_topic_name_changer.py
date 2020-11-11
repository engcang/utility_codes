from rosbag import Bag
with Bag('Y.bag', 'w') as Y:
    for topic, msg, t in Bag('X.bag'):
        if topic == '/a_topic':
            Y.write('/b_topic', msg, t)
        else:
            Y.write(topic, msg, t)

#evo_traj bag test.bag /tf:world.uav/imu_check /tf:world.msckf -v --full_check -p --save_plot plot
#evo_ape bag test.bag /tf:world.uav/imu_check /tf:world.msckf -v -p --save_plot plot --t_max_diff 0.1 --save_results <algorithm name>
#evo_res results/*.zip -p --save_table results/table.csv
#evo_res <algorithm name> -p --use_filenames --save_table results/table.csv


'''
evo_traj bag test.bag /tf:world.uav/imu_check /tf:world.msckf -v --full_check -p --save_as_bag  ==> saved as geometry/PoseStamped
then use rosbag_topic_name_changer.py => again evo_ape or rpe or anything else.
'''
