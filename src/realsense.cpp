#include "realsense.h"

#include "opencv2/highgui/highgui.hpp"
namespace realsense{
    cv::Point3f Deproject_2D_to_3D(double x, double y, float depth){
    double fx, fy, ppx, ppy;
    fx = 618.530133;
    fy = 616.458672;
    ppx = 324.734598;
    ppy = 246.243079;

    double x_s = (x - ppx) / fx;
    double y_s = (y - ppy) / fy;

    cv::Point3f point;
    point.x = depth * x_s;
    point.y = depth * y_s;
    point.z = depth;

    return point;
    }
    double Get_Depth_Value(double x, double y, cv::Mat depthImg){
        double depth = 0.001*depthImg.at<u_int16_t>(y,x);
        return depth;
    }

}