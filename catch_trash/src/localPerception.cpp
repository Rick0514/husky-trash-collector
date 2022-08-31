#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

using namespace std;

// 0. planer inform LP to work!!
// 1. if no trash in view, rotate itself
// 2. got trash ! tune itself slowly till trash is in the middle of view
// 3. go straight slowly till trash sink
// 4. grab the ball
// 5. inform planer serivce: "/ball_grabed"

bool vis = true;

class LocalPerception
{
private:
    bool _enable{false};

    const string _getToWorkTopic{"/lp_work"};

    ros::NodeHandle& _nh;
    ros::Publisher _cmdPub;
    ros::Publisher _targetPub;
    ros::Publisher _doorPub; 
    ros::ServiceServer _workServer;    
    image_transport::ImageTransport _imgTsp;
    image_transport::Subscriber _imgSub;

    cv::Mat _img;
    cv::Mat _showImg;
    cv::Point2f _pos;

    const int _centerGap = 8;


public:

    LocalPerception(ros::NodeHandle& nh) : _nh(nh), _imgTsp(nh)
    {
        _targetPub = _nh.advertise<geometry_msgs::PoseStamped>("/target", 1);
        _imgSub = _imgTsp.subscribe("/camera/image_raw", 1, &LocalPerception::imageCallback, this);
        _cmdPub = _nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
        _workServer = _nh.advertiseService(_getToWorkTopic, &LocalPerception::getToWork, this);
        _doorPub = _nh.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 1);
       
        ROS_DEBUG_NAMED("lp", "LP is init!!");
    }

    bool getToWork(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
    {
        ROS_DEBUG_NAMED("lp", "LP is getting to work!!");
        _enable = true;
        return true;
    }

    bool findTrash()
    {
        cv::Mat dst, mask_red, mask_blue;
        vector<cv::Mat> bgr;
        cv::split(_img, bgr);
        cv::threshold(bgr.at(2), mask_red, 230, 255, cv::THRESH_BINARY);
        cv::threshold(bgr.at(0), mask_blue, 50, 255, cv::THRESH_BINARY_INV);

        dst = mask_red & mask_blue;

        if(vis){
            cv::imshow("ball", dst);
            cv::waitKey(1);
        }

        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if(contours.size() == 0)    return false;

        vector<float> radiusVec;
        vector<cv::Point2f> centerVec;
        for(int i=0; i<contours.size(); i++)
        {   
            float r;
            cv::Point2f center;
            cv::minEnclosingCircle(contours[i], center, r);
            // ROS_WARN("circle coord(%f, %f)", center.x, center.y);
            radiusVec.emplace_back(std::move(r));
            centerVec.emplace_back(std::move(center));
            cv::circle(_showImg, center, r, cv::Scalar(0, 255, 0), 1);
        }

        // get max one
        auto maxIdx = std::distance(radiusVec.begin(), max_element(radiusVec.begin(), radiusVec.end()));
        if(radiusVec.at(maxIdx) < 5){
            return false;
        }

        _pos = centerVec.at(maxIdx);
        

        return true;
    }

    void moveForward(float x)
    {
        geometry_msgs::Twist tw;
        // complete it
        tw.linear.x = x;
        tw.linear.y = 0;
        tw.linear.z = 0;
        tw.angular.x = 0;
        tw.angular.y = 0;
        tw.angular.z = 0;
        _cmdPub.publish(tw);
    }

    void moveAround(float z)
    {
        geometry_msgs::Twist tw;
        // complete it
        tw.linear.x = 0;
        tw.linear.y = 0;
        tw.linear.z = 0;
        tw.angular.x = 0;
        tw.angular.y = 0;
        tw.angular.z = z;
        _cmdPub.publish(tw);
    }

    void rotateYourself()
    {
        moveAround(0.5);
    }

    bool tuningPosition()
    {
        if(_pos.x < 320 - _centerGap)
        {
            // turn left
            moveAround(0.1);
            return true;
        }else if(_pos.x > 320 + _centerGap)
        {
            // turn right
            moveAround(-0.1);
            return true;
        }else if(_pos.y < 420)
        {
            // go forward
            moveForward(0.5);
            return true;
        }

        return false;
    }

    void grabTheTrash()
    {
        ROS_DEBUG_NAMED("lp", "start to grab trash!!");
        // 1. go forward
        // 2. put down the door
        // complete it
        int pub_times = 10;
        for (size_t i = 0; i < pub_times; i++){
            moveForward(0.2);
            ros::Duration(0.2).sleep();
        }
        // grab
        std_msgs::Float64 door_msg;
        door_msg.data = -1.57;
        _doorPub.publish(door_msg);
        
        // stop
        // set tw to 0
        moveForward(0);

    }

    void informPlaner()
    {
        geometry_msgs::PoseStamped p;
        p.pose.position.x = 26.7;
        p.pose.position.y = 18.0;
        p.pose.position.z = 0;
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        _targetPub.publish(p);
        // _targetPub.publish(p);

        _enable = false;
        ROS_DEBUG_NAMED("lp", "LP finished its work!!");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        if(!_enable)    return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            _img = cv_ptr->image.clone();
            _showImg = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if(!findTrash()){
            rotateYourself();
            return;
        }

        if(!tuningPosition()){
            moveForward(0);
            ROS_DEBUG_NAMED("lp", "pos is good!! ready to grab");
            grabTheTrash();
            informPlaner();
            cv::destroyAllWindows();       
            return; 
        }

        if(vis){
            cv::imshow("circle", _showImg);
            cv::waitKey(1);
        }
    }

};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_perception_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("vis", vis);

    LocalPerception LP(nh);

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}

