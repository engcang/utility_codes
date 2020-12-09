///// common headers for depth_to_pcl/octo and local_planner
#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <fstream> // ofstream, file writing
#include <math.h> // pow
#include <vector>
#include <chrono> 
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// headers for local_planner + controller
#include <tf2_msgs/TFMessage.h> //for tf between frames

///// headers for depto_to_pcl + octo
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

octomap::OcTree *m_octree;
double octomap_resolution, octomap_hit, octomap_miss, map_min_z, map_max_z;
int free_oc=0, occ_oc=0;

ros::Publisher pcl_pub, octo_pub;
bool tf_check=false, body_t_cam_check=false;
Matrix4f map_t_cam = Matrix4f::Identity();
Matrix4f map_t_body = Matrix4f::Identity();
Matrix4f body_t_cam = Matrix4f::Identity();

pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl, depth_cvt_pcl_map;
high_resolution_clock::time_point oc_start_t;




sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZ> cloudresult;
  pcl::fromROSMsg(cloudmsg,cloudresult);
  return cloudresult;
}





void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id=="map" && msg->transforms[l].child_frame_id=="base_link"){
      ///// for tf between map and body
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      map_t_body(0,0) = m[0][0];
      map_t_body(0,1) = m[0][1];
      map_t_body(0,2) = m[0][2];
      map_t_body(1,0) = m[1][0];
      map_t_body(1,1) = m[1][1];
      map_t_body(1,2) = m[1][2];
      map_t_body(2,0) = m[2][0];
      map_t_body(2,1) = m[2][1];
      map_t_body(2,2) = m[2][2];

      map_t_body(0,3) = msg->transforms[l].transform.translation.x;
      map_t_body(1,3) = msg->transforms[l].transform.translation.y;
      map_t_body(2,3) = msg->transforms[l].transform.translation.z;
      map_t_body(3,3) = 1.0;
    }
    if (msg->transforms[l].header.frame_id=="base_link" && !body_t_cam_check){
      tf::Quaternion q2(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m2(q2);
      body_t_cam(0,0) = m2[0][0];
      body_t_cam(0,1) = m2[0][1];
      body_t_cam(0,2) = m2[0][2];
      body_t_cam(1,0) = m2[1][0];
      body_t_cam(1,1) = m2[1][1];
      body_t_cam(1,2) = m2[1][2];
      body_t_cam(2,0) = m2[2][0];
      body_t_cam(2,1) = m2[2][1];
      body_t_cam(2,2) = m2[2][2];

      body_t_cam(0,3) = msg->transforms[l].transform.translation.x;
      body_t_cam(1,3) = msg->transforms[l].transform.translation.y;
      body_t_cam(2,3) = msg->transforms[l].transform.translation.z;
      body_t_cam(3,3) = 1.0;

      body_t_cam_check = true; // body_t_cam is fixed!!!
    }
  }
  map_t_cam = map_t_body * body_t_cam ;
  tf_check=true;
}

void depth_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (tf_check){
        depth_cvt_pcl = cloudmsg2cloud(*msg);
        pcl::transformPointCloud(depth_cvt_pcl, depth_cvt_pcl_map, map_t_cam);
        std::vector<int> indexx;
        pcl::removeNaNFromPointCloud(depth_cvt_pcl_map, depth_cvt_pcl_map, indexx);

        octomap::Pointcloud temp_pcl; 
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = depth_cvt_pcl_map.begin(); it!=depth_cvt_pcl_map.end(); ++it){
          temp_pcl.push_back(it->x, it->y, it->z);
        } 
        m_octree->insertPointCloud(temp_pcl, octomap::point3d(map_t_cam(0,3),map_t_cam(1,3),map_t_cam(2,3))); 


        pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
        free_oc=0; occ_oc=0;
        for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
          if(m_octree->isNodeOccupied(*it))
          {
            free_oc++;
            if (it.getCoordinate().z()<map_max_z)
              octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
          }
          else
          {
            occ_oc++;
          }
        }
        pcl_pub.publish(cloud2msg(*octo_pcl_pub, "map"));
        octomap_msgs::Octomap octo_ros;
        octomap_msgs::fullMapToMsg(*m_octree, octo_ros);
        octo_pub.publish(octo_ros);

        auto oc_stop = high_resolution_clock::now();
        ofstream foutC("/home/mason/test.csv", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(3);
        foutC << duration_cast<microseconds>(oc_stop - oc_start_t).count()/1000000.0 << ",";
        foutC << occ_oc*octomap_resolution*octomap_resolution*octomap_resolution << ","
              << free_oc*octomap_resolution*octomap_resolution*octomap_resolution << ","
              << (free_oc+occ_oc)*octomap_resolution*octomap_resolution*octomap_resolution << endl;
        foutC.close();
    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "octopus");
    ros::NodeHandle nh("~");

    nh.param("/octomap_resolution", octomap_resolution, 0.3);
    nh.param("/octomap_hit", octomap_hit, 0.8);
    nh.param("/octomap_miss", octomap_miss, 0.45);
    nh.param("/map_min_z", map_min_z, 0.0);
    nh.param("/map_max_z", map_max_z, 2.3);

    m_octree = new octomap::OcTree(octomap_resolution);
    m_octree->setProbHit(octomap_hit);
    m_octree->setProbMiss(octomap_miss);
    oc_start_t = high_resolution_clock::now();

    ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::PointCloud2>("/d455/depth/pointcloud", 10, depth_callback);
    ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, tf_callback);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/octopus", 10);
    octo_pub = nh.advertise<octomap_msgs::Octomap>("/aeplanner/octomap_full", 10);


    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

