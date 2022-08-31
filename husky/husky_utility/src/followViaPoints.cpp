#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <iostream>

static bool set_or_read;

// traj
static std::string points_filedir = "";
std::vector<Eigen::Vector2d> viaPoint;
static int vp_cnt = 0;

// nav
static std::string path_topic;
ros::Publisher pub_path;
nav_msgs::Path path;

// send cmd
static std::string send_cmd = "/cmd_vel";
ros::Publisher pub_cmd;
static int cmd_hz = 10;
Eigen::Vector2d cmd_dir(0, 0);
static double max_vel = 5.0;
static double max_ang = 1.0;
static double dist_threshold = 0.5;
static double ang_mul = 5.0;

// get odom
static std::string odom_topic = "/odom";
ros::Subscriber sub_odom;

void getViaPoint(const geometry_msgs::PoseStampedConstPtr &pose) {
    Eigen::Vector2d p(pose->pose.position.x, pose->pose.position.y);
    viaPoint.emplace_back(p);

    ROS_INFO("get via point");
    geometry_msgs::PoseStamped path_pose;

    path.header.frame_id = pose->header.frame_id;
    path.header.stamp = pose->header.stamp;

    path_pose.header.stamp = pose->header.stamp;
    path_pose.header.seq = pose->header.seq;
    path_pose.header.frame_id = pose->header.frame_id;
    path_pose.pose.position.x = pose->pose.position.x;
    path_pose.pose.position.y = pose->pose.position.y;
    path_pose.pose.position.z = pose->pose.position.z;
    path_pose.pose.orientation.x = pose->pose.orientation.x;
    path_pose.pose.orientation.y = pose->pose.orientation.y;
    path_pose.pose.orientation.z = pose->pose.orientation.z;
    path_pose.pose.orientation.w = pose->pose.orientation.w;

    path.poses.emplace_back(path_pose);

    pub_path.publish(path);
}

void odomCallback(const nav_msgs::OdometryConstPtr &odom) {
    static tf::TransformBroadcaster br;
    tf::Transform tf;
    geometry_msgs::Pose odom_pose = odom->pose.pose;

    // tf::poseMsgToTF(odom_pose, tf);
    // tf::StampedTransform stamped_tf(tf, odom->header.stamp, parent_frame,
    //                                 child_frame);
    // br.sendTransform(stamped_tf);

    // update twist dir
    if (!viaPoint.empty()) {
        double th = 2 * atan2(odom_pose.orientation.z, odom_pose.orientation.w);
        Eigen::Matrix2d R_map_odom;
        R_map_odom << cos(th), -sin(th), sin(th), cos(th);

        Eigen::Vector2d odom_point(odom_pose.position.x, odom_pose.position.y);
        cmd_dir = R_map_odom.transpose() * (viaPoint[vp_cnt] - odom_point);

        if (cmd_dir.norm() < dist_threshold && vp_cnt + 1 < viaPoint.size()) {
            vp_cnt += 1;
        }
    }
}

inline double limitVel(double x, char flag) {
    return flag == 'l' ? std::max(-max_vel, std::min(max_vel, x)) : std::max(-max_ang, std::min(max_ang, x));
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "followViaPoints");
    ros::NodeHandle nh("~");

    // nh.getParam("path_topic", path_topic);
    nh.getParam("points_filedir", points_filedir);
    nh.getParam("odom_topic", odom_topic);

    std::cout << "Odom topic: " << odom_topic << std::endl;

    nh.getParam("set_or_read", set_or_read);

    pub_path = nh.advertise<nav_msgs::Path>(path_topic, 10, true);

    nh.getParam("send_cmd", send_cmd);
    nh.getParam("cmd_hz", cmd_hz);
    nh.getParam("max_vel", max_vel);
    nh.getParam("max_ang", max_ang);
    nh.getParam("dist_threshold", dist_threshold);
    nh.getParam("ang_mul", ang_mul);

    if (!set_or_read) {
        // init via point from external txt
        std::ifstream infile(points_filedir);
        double x, y;

        while (infile >> x >> y) {
            viaPoint.emplace_back(Eigen::Vector2d(x, y));
        }

        infile.close();
    }

    pub_cmd = nh.advertise<geometry_msgs::Twist>(send_cmd, 10, true);

    // geometry_msgs::PoseStamped path_pose;
    // path.header.frame_id = parent_frame;
    // path.header.stamp = ros::Time::now();
    // path_pose.header.stamp = path.header.stamp;
    // path_pose.header.seq = 0;
    // path_pose.header.frame_id = parent_frame;
    // path_pose.pose.position.x = 0;
    // path_pose.pose.position.y = 0;
    // path_pose.pose.position.z = 0;
    // path_pose.pose.orientation.x = 0;
    // path_pose.pose.orientation.y = 0;
    // path_pose.pose.orientation.z = 0;
    // path_pose.pose.orientation.w = 1;
    // path.poses.emplace_back(path_pose);

    ros::Subscriber vpSub = nh.subscribe("/move_base_simple/goal", 5, getViaPoint);

    sub_odom = nh.subscribe(odom_topic, 10, odomCallback);

    ros::Rate r(cmd_hz);
    while (ros::ok()) {
        geometry_msgs::Twist v;
        double th = atan2(cmd_dir(1), cmd_dir(0));

        if (th > 10.0 / 180 * M_PI)
            v.linear.x = 0;
        else
            v.linear.x = limitVel(cmd_dir(0), 'l');
        v.linear.y = 0;
        v.linear.z = 0;
        v.angular.x = 0;
        v.angular.y = 0;
        v.angular.z = limitVel(ang_mul * th, 'a');

        if (!viaPoint.empty())
            pub_cmd.publish(v);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
