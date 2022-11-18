#ifndef MODE_H
#define MODE_H

#include "mode.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>

namespace mode{
    class Mode{
    private:
        int _nMode = 0;
        cv::Mat _BackColorImg,_FrontColorImg;
        geometry_msgs::Twist _cmd_vel;
        cv::Mat _BackDepthImg;
        std::vector<cv::Point2f> _vP2_Bcorners;
        std::vector<cv::Point2f> _vP2_Fcorners;
        bool _bLastMarkerPOS = false; //ture : right, false : left
        double _dDistance;
    public:
        Mode(cv::Mat front_rgb_image, cv::Mat back_rgb_image);
        int mode(void);
        geometry_msgs::Twist cmd_vel(void);
        void SelectMode(void);
        void Mode2(cv::Mat back_depth_image);
        bool lastMarkerPOS(void);
        double Mdistance(void);
    };
}



#endif