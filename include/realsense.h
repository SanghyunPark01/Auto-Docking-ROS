#ifndef REALSENSE_H
#define REALSENSE_H
#include "opencv2/highgui/highgui.hpp"

namespace realsense{
    cv::Point3f Deproject_2D_to_3D(double x, double y, float depth);
    double Get_Depth_Value(double x, double y, cv::Mat depthImg);
}

#endif