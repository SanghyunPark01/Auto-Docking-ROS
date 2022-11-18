#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include "mode.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rsutil.h>

cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 618.53013, 0., 324.7346, 0., 616.45867, 246.24308, 0., 0., 1.);
cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.123868, -0.281684, -0.002987, 0.000575, 0.);
double u0 = 324.7346;
double v0 = 246.24308;

class NodeServer
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Subscriber _sub_flag;
    bool _bFlag = false;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> back_rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image> back_depth_sub;
    message_filters::Subscriber<sensor_msgs::Image> front_rgb_sub;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

public:
    NodeServer()
    {
        _sub_flag = _nh.subscribe("/check_DockingSystem_Flag", 1, &NodeServer::Docking_Flag, this);
        _pub = _nh.advertise<geometry_msgs::Twist>("/docking/cmd_vel", 1000);

        back_rgb_sub.subscribe(_nh, "/camera/color/image_raw", 1);
        back_depth_sub.subscribe(_nh, "/camera/depth/image_rect_raw", 1);
        front_rgb_sub.subscribe(_nh, "/camera/color/image_raw", 1);
        sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1), back_rgb_sub, back_depth_sub, front_rgb_sub));

        sync->registerCallback(boost::bind(&NodeServer::callback, this, _1, _2, _3));
    }
    void Docking_Flag(const std_msgs::Int32 n)
    {
        if (n.data == 0)
        {
            _bFlag = false;
        }
        else if (n.data == 1)
        {
            _bFlag = true;
        }
    }
    void callback(const sensor_msgs::ImageConstPtr &back_rgb_msg, const sensor_msgs::ImageConstPtr &back_depth_msg, const sensor_msgs::ImageConstPtr &front_rgb_msg);
};

void NodeServer::callback(const sensor_msgs::ImageConstPtr &back_rgb_msg, const sensor_msgs::ImageConstPtr &back_depth_msg, const sensor_msgs::ImageConstPtr &front_rgb_msg)
{
    // converting Img -> CV::Mat class
    _bFlag = true; //for test
    
    if (_bFlag == true)
    {
        cv_bridge::CvImagePtr cv_ptr1, cv_ptr2, cv_ptr3;
        try
        {
            cv_ptr1 = cv_bridge::toCvCopy(back_rgb_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e1)
        {
            ROS_ERROR("cv_bridge exception: %s", e1.what());
            return;
        }
        try
        {
            cv_ptr2 = cv_bridge::toCvCopy(back_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e2)
        {
            ROS_ERROR("cv_bridge exception: %s", e2.what());
            return;
        }
        try
        {
            cv_ptr3 = cv_bridge::toCvCopy(front_rgb_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e1)
        {
            ROS_ERROR("cv_bridge exception: %s", e1.what());
            return;
        }
        cv::Mat back_rgb_image1 = cv_ptr1->image;
        cv::Mat back_depth_image = cv_ptr2->image;
        cv::Mat front_rgb_image1 = cv_ptr3->image;

        cv::Mat front_rgb_image, back_rgb_image;
        int mode = 0; // 0 : No marker //1 : detect marker front camera //2 : detect marker back camera

        cv::undistort(front_rgb_image1, front_rgb_image, cameraMatrix, distCoeffs);
        cv::undistort(back_rgb_image1, back_rgb_image, cameraMatrix, distCoeffs);

        mode::Mode main(front_rgb_image, back_rgb_image);
        main.SelectMode();
        mode = main.mode();

        geometry_msgs::Twist cmd_vel;
        
        
        std::cout << "mode : "<< mode << std::endl;

        if(mode == 0 && main.lastMarkerPOS()==true){
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -0.6;
        }else if(mode == 0 && main.lastMarkerPOS()==false){
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.6;
        }
        else if (mode == 0)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.6;
        }
        else if (mode == 1)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.6;
        }
        else if (mode == 2)
        {
            main.Mode2(back_depth_image);
            cmd_vel = main.cmd_vel();
        }
        else
        {
            ROS_ERROR("Mode error : check mode converting");
            return;
        }
        _pub.publish(cmd_vel);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_dock_ros");

    NodeServer subpub;

    ros::spin();

    return 0;
}
