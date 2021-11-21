#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <math.h>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;
using namespace Eigen;

class URL_tf{
  public:
    ros::NodeHandle nh;
    ros::Subscriber subb;
    ros::Publisher pubb;
    Matrix4f cam_t_body = Matrix4f::Identity();
    Matrix4f map_t_body = Matrix4f::Identity();
    Matrix4f cam_frame_t_cam = Matrix4f::Identity();
    geometry_msgs::PoseStamped pub_pose;
    string output_topic;

    void tf_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    URL_tf(ros::NodeHandle& n) : nh(n){
      nh.param<std::string>("/output_topic", output_topic, "/mavros/vision_pose/pose");
      pubb = nh.advertise<geometry_msgs::PoseStamped>(output_topic, 20);
      subb = nh.subscribe<geometry_msgs::PoseStamped> ("/vins/vins_poses", 20,  &URL_tf::tf_cb, this);

      pub_pose.header.frame_id = "map";

      tf::Quaternion q( 1.0, 0.0, 0.0, 0.0 );
      tf::Matrix3x3 m(q);
      cam_t_body(0,0) = m[0][0];
      cam_t_body(0,1) = m[0][1];
      cam_t_body(0,2) = m[0][2];
      cam_t_body(1,0) = m[1][0];
      cam_t_body(1,1) = m[1][1];
      cam_t_body(1,2) = m[1][2];
      cam_t_body(2,0) = m[2][0];
      cam_t_body(2,1) = m[2][1];
      cam_t_body(2,2) = m[2][2];
      cam_t_body(0,3) = 0.0;
      cam_t_body(1,3) = 0.0;
      cam_t_body(2,3) = 0.0;
      cam_t_body(3,0) = 0.0;
      cam_t_body(3,1) = 0.0;
      cam_t_body(3,2) = 0.0;
      cam_t_body(3,3) = 1.0;
      ROS_WARN("URL tf_and_vision package's tf_node, Starting node...");
    }
};

void URL_tf::tf_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Quaternion q2(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m2(q2);
  cam_frame_t_cam(0,0) = m2[0][0];
  cam_frame_t_cam(0,1) = m2[0][1];
  cam_frame_t_cam(0,2) = m2[0][2];
  cam_frame_t_cam(1,0) = m2[1][0];
  cam_frame_t_cam(1,1) = m2[1][1];
  cam_frame_t_cam(1,2) = m2[1][2];
  cam_frame_t_cam(2,0) = m2[2][0];
  cam_frame_t_cam(2,1) = m2[2][1];
  cam_frame_t_cam(2,2) = m2[2][2];

  cam_frame_t_cam(0,3) = msg->pose.position.x;
  cam_frame_t_cam(1,3) = msg->pose.position.y;
  cam_frame_t_cam(2,3) = msg->pose.position.z;
  cam_frame_t_cam(3,3) = 1.0;

//  map_t_body = cam_t_body * cam_frame_t_cam;
  map_t_body =  cam_frame_t_cam * cam_t_body;
  //map_t_body.block(0,0,3,3) = cam_frame_t_cam.block(0,0,3,3);
  //map_t_body.block(0,3,3,1) = cam_frame_t_cam.block(0,3,3,1) + cam_frame_t_cam.block(0,0,3,3) * cam_t_body.block(0,3,3,1);
  
  tf::Matrix3x3 m3;
  m3[0][0] = map_t_body(0,0);
  m3[0][1] = map_t_body(0,1);
  m3[0][2] = map_t_body(0,2);
  m3[1][0] = map_t_body(1,0);
  m3[1][1] = map_t_body(1,1);
  m3[1][2] = map_t_body(1,2);
  m3[2][0] = map_t_body(2,0);
  m3[2][1] = map_t_body(2,1);
  m3[2][2] = map_t_body(2,2);

  tf::Quaternion q3;
  m3.getRotation(q3);

  pub_pose.header.stamp = ros::Time::now(); // important
//  pub_pose.pose.position.x = map_t_body(0,3);
//  pub_pose.pose.position.y = map_t_body(1,3);
//  pub_pose.pose.position.z = map_t_body(2,3);
  pub_pose.pose.position.x = msg->pose.position.x;
  pub_pose.pose.position.y = msg->pose.position.y;
  pub_pose.pose.position.z = msg->pose.position.z;

  pub_pose.pose.orientation.x = q3.getX();
  pub_pose.pose.orientation.y = q3.getY();
  pub_pose.pose.orientation.z = q3.getZ();
  pub_pose.pose.orientation.w = q3.getW();
  pubb.publish(pub_pose);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "URL_tf_package");
  ros::NodeHandle n("~");

  URL_tf url_tf(n);

  signal(SIGINT, signal_handler);

  ros::spin();

  return 0;
}
