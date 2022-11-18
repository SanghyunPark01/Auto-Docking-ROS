#include "mode.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>
#include "controller.h"
#include "detector.h"
#include "vector"

#define LIMIT_LV 1
#define LIMIT_AV 0.6

cv::Vec2f dPastLError = 0;
double dPastAError = 0;
bool _bLastMarkerPOS = false; //ture : right, false : left

namespace mode
{   
    Mode::Mode(cv::Mat front_rgb_image, cv::Mat back_rgb_image){
        _BackColorImg = back_rgb_image;
        _FrontColorImg = front_rgb_image;
    }
    int Mode::mode(void){
        return _nMode;
    }
    geometry_msgs::Twist Mode::cmd_vel(void){
        return _cmd_vel;
    }
    bool Mode::lastMarkerPOS(void){
        return _bLastMarkerPOS;
    }
    double Mode::Mdistance(void){
        return _dDistance;
    }
    void Mode::SelectMode(void){
        int mode;

        //마커 인식 -> id 확인 -> (앞카메라 뒷카메라 둘다 인식 x - mode=0)(앞카메라 맞는아이디 - mode=1)(뒷카메라 맞는아이디 - mode=2)

        cv::imshow("brgb",_BackColorImg);
        cv::waitKey(1);
        //cv::imshow("frgb",_FrontColorImg);
        std::vector<cv::Point2f> vP2_Bcorners;
        std::vector<cv::Point2f> vP2_Fcorners;
        
        bool FrontExist = false;
        bool BackExist = false;
        FrontExist = Detector::FrontDetect_OneMarkers(vP2_Fcorners, _FrontColorImg);
        BackExist  = Detector::BackDetect_OneMarkers(vP2_Bcorners, _BackColorImg, _bLastMarkerPOS);
        //std::cout <<vP2_Bcorners<<std::endl;

        if(!FrontExist && !BackExist){
            mode = 0;
        }else if(FrontExist && !BackExist){
            mode = 1;
        }else if(!FrontExist && BackExist){
            mode = 2;
        }else{
            mode = 2;
            std::cout << "[Warning] Markers are detected on both Front and Back cameras! (from mode.cpp line 49)" << std::endl;
        }

        _vP2_Bcorners = vP2_Bcorners;
        _vP2_Fcorners = vP2_Fcorners;

        _nMode = mode;
    }
    void Mode::Mode2(cv::Mat BackDepthImg){
        _BackDepthImg = BackDepthImg;
        geometry_msgs::Twist cmd_vel;
        //cv::imshow("depth",_BackDepthImg);
        //cv::waitKey(1);

        cmd_vel = controller::main_controller(_BackColorImg, _BackDepthImg,_vP2_Bcorners ,dPastLError, dPastAError, _dDistance);

        //saturation
        if(cmd_vel.angular.z > LIMIT_AV)cmd_vel.angular.z = LIMIT_AV;
        if(cmd_vel.angular.z < -LIMIT_AV)cmd_vel.angular.z = -LIMIT_AV;
        if(cmd_vel.linear.x > LIMIT_LV)cmd_vel.linear.x = LIMIT_LV;
        if(cmd_vel.linear.x < -LIMIT_LV)cmd_vel.linear.x = -LIMIT_LV;
        if(cmd_vel.linear.y > LIMIT_LV)cmd_vel.linear.y = LIMIT_LV;
        if(cmd_vel.linear.y < -LIMIT_LV)cmd_vel.linear.y = -LIMIT_LV;

        _cmd_vel = cmd_vel;
    }
    
}