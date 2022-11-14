#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controller.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>

namespace controller{
    geometry_msgs::Twist main_controller(cv::Mat BackColorImg, cv::Mat BackDepthImg,std::vector<cv::Point2f>corners, cv::Vec2f& dPastLError, double& dPastAError);
}



#endif