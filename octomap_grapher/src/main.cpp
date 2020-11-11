///// common headers for depth_to_pcl/octo and local_planner
#include <iostream> //cout
#include <ros/ros.h>
#include <math.h> // pow
#include <chrono> 

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std::chrono; 
using namespace std;

string octomap_topic_name;
double octo_resolution = 0.5;
octomap_msgs::Octomap octo_ros;


#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}
high_resolution_clock::time_point start; //global, to use tic()
void tic(){
   start = high_resolution_clock::now();
}
double toc(){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   // ROS_INFO("%.3f ms spent", duration.count()/1000000.0);
   return duration.count()/1000000.0;
}




void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg){
	octo_ros=*msg;

    octomap::OcTree *m_octree = new octomap::OcTree(octo_resolution);
    // octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(octo_ros);
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octo_ros);
    m_octree = dynamic_cast<octomap::OcTree*>(tree);

	int occ=0;
	int free=0;

    for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
      if(m_octree->isNodeOccupied(*it))
      {
  	  	occ++;
      }
      else
      {
      	free++;
      }
    }
    cout << toc() << "," << occ*0.5*0.5*0.5 << "," << free*0.5*0.5*0.5 << "," << (occ+free)*0.5*0.5*0.5 << endl;

    if (m_octree){
      delete m_octree;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_grapher");
    ros::NodeHandle n;

    n.param<std::string>("/octomap_topic_name", octomap_topic_name, "/global_planner_node/octomap_full");
    n.param("/octomap_resolution", octo_resolution, 0.5);

    ros::Subscriber octo_sub = n.subscribe<octomap_msgs::Octomap>(octomap_topic_name, 10, octomap_cb);
    tic();
    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::spin();

    return 0;
}