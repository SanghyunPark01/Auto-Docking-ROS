#ifndef DETECTOR_H
#define DETECTOR_H
#include <opencv2/highgui/highgui.hpp>

namespace Detector{
    bool FrontDetect_OneMarkers(std::vector<cv::Point2f>& vP2_Fcorners, cv::Mat cImg);
    bool BackDetect_OneMarkers(std::vector<cv::Point2f>& vP2_Bcorners, cv::Mat cImg, bool& lastmarker);
}



#endif