#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>

namespace ControlSystem{
    class DockingController{
    private:
        cv::Vec2f _dCurrLError = 0;
        double _dCurrAError = 0;
        cv::Vec2f _dPastLError = 0;
        double _dPastAError = 0;
        cv::Mat _cvCImg;
        cv::Mat _cvDImg;
        std::vector<cv::Point2f> _vp2Corner;
        std::vector<cv::Point3f> _vp3Corner;
        cv::Point2f _dp2Center;
        cv::Point3f _dp3Center;
        double _dYaw;
        double _dYaw_DEG;
        geometry_msgs::Twist _cmd_vel;
        double _closedepth(void);
        void _dCalculateYaw(double markerdepth);
        void _YawFusion(double opencv_yaw, double my_yaw);
    public:
        DockingController(cv::Mat BackColorImg, cv::Mat BackDepthImg, std::vector<cv::Point2f> corner);
        void PDcontrol(cv::Vec2f dPastLError, double dPastAError);
        cv::Vec2f CurrentLinearError(void);
        double CurrentAngularError(void);
        std::vector<cv::Point2f> Corner(void);
        cv::Point2f Center(void);
        geometry_msgs::Twist cmd_vel(void);
        void CalculateError(void);
    };
}


#endif