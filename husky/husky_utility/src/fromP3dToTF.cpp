#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <fstream>
#include <random>

std::string parent_frame = "world";
std::string child_frame = "base_link";
std::string topic = "odom";

void odom_callback(const nav_msgs::OdometryConstPtr& odom){

  // 发布tf
  static tf::TransformBroadcaster br;
  tf::Transform tf;
  geometry_msgs::Pose odom_pose = odom->pose.pose;
  tf::poseMsgToTF(odom_pose, tf);
  tf::StampedTransform stamped_tf(tf, odom->header.stamp, parent_frame, child_frame);
  br.sendTransform(stamped_tf);

}

int main(int argc, char **argv){

    std::string node_name = "fromP3dToTF";
    if(argc >= 1)   node_name += argv[1];

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("parent_frame", parent_frame);
    pnh.getParam("child_frame", child_frame);
    pnh.getParam("topic", topic);
    ros::Subscriber odom_sub = nh.subscribe(topic, 1, odom_callback);
    ros::spin();

    return 0;
}