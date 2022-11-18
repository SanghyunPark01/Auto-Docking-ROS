#include "ControlSystem.h"
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <realsense.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

cv::Mat cameraMatrix1 = (cv::Mat1d(3, 3) << 618.53013, 0., 324.7346, 0., 616.45867, 246.24308, 0., 0., 1.);
cv::Mat distCoeffs1 = (cv::Mat1d(1, 5) << 0.123868, -0.281684, -0.002987, 0.000575, 0.);

#define REFER_X 0.1 //forward
#define REFER_Y 0.0 //Side
#define REFER_A 0.0 //Yaw

#define LX_Kp 0.2;
#define LX_Kd 0.1;

#define LY_Kp 1;
#define LY_Kd 0.4;

#define A_Kp 0.05;
#define A_Kd 0.03;

namespace ControlSystem{
    DockingController::DockingController(cv::Mat BackColorImg, cv::Mat BackDepthImg, std::vector<cv::Point2f> corner){
        _cvCImg = BackColorImg;
        _cvDImg = BackDepthImg;
        _vp2Corner = corner;
        cv::Point2f center;
        for (const auto& corner_point : corner)
        {
            center += corner_point;
        }
        _dp2Center = center/4;
    }
    cv::Vec2f DockingController::CurrentLinearError(void){
        return _dCurrLError;
    }
    double DockingController::CurrentAngularError(void){
        return _dCurrAError;
    }
    std::vector<cv::Point2f> DockingController::Corner(void){
        return _vp2Corner;
    }
    cv::Point2f DockingController::Center(void){
        return _dp2Center;
    }
    geometry_msgs::Twist DockingController::cmd_vel(void){
        return _cmd_vel;
    }
    void DockingController::CalculateError(void){
        double dMarkerDepth;

        dMarkerDepth = realsense::Get_Depth_Value(_dp2Center.x, _dp2Center.y, _cvDImg);
        _dCalculateYaw(dMarkerDepth);
        //std::cout << "Yaw : " << _dYaw_DEG << std::endl;
        //std::cout << _dp3Center << std::endl; // Back camera : x, y, z => robot : -y, -z , -x
        //std::cout << _dp3Center.z<< std::endl;
        _dCurrLError(0) = REFER_X - (_dp3Center.z);
        _dCurrLError(1) = REFER_Y - (_dp3Center.x);
        _dCurrAError = REFER_A - _dYaw_DEG;
        
    }
    void DockingController::_dCalculateYaw(double dMarkerDepth){

        std::vector<cv::Vec3d> rvecs, tvecs;
        std::vector<std::vector<cv::Point2f>> corners;
        corners.push_back(_vp2Corner);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.0525, cameraMatrix1, distCoeffs1, rvecs, tvecs);

        if(dMarkerDepth < 0.2)dMarkerDepth = tvecs[0][2];
        else{
            
            _vp3Corner.push_back(realsense::Deproject_2D_to_3D(_vp2Corner[0].x,_vp2Corner[0].y,realsense::Get_Depth_Value(_vp2Corner[0].x,_vp2Corner[0].y,_cvDImg)));
            _vp3Corner.push_back(realsense::Deproject_2D_to_3D(_vp2Corner[1].x,_vp2Corner[1].y,realsense::Get_Depth_Value(_vp2Corner[1].x,_vp2Corner[1].y,_cvDImg)));
            _vp3Corner.push_back(realsense::Deproject_2D_to_3D(_vp2Corner[2].x,_vp2Corner[2].y,realsense::Get_Depth_Value(_vp2Corner[2].x,_vp2Corner[2].y,_cvDImg)));
            _vp3Corner.push_back(realsense::Deproject_2D_to_3D(_vp2Corner[3].x,_vp2Corner[3].y,realsense::Get_Depth_Value(_vp2Corner[3].x,_vp2Corner[3].y,_cvDImg)));
        }
        _dp3Center = realsense::Deproject_2D_to_3D(_dp2Center.x,_dp2Center.y,dMarkerDepth);
        
        
        //------------------------------
        //If you erase the if statement below, an segmentation error sometimes occurs.
        if(_vp3Corner.empty() && dMarkerDepth >= 0.2){
            dMarkerDepth = 0;
        }
        //------------------------------

        double dRealSenseYaw = 0;
        double dOpencvYaw;
        cv::Mat Rm;
        cv::Rodrigues(rvecs[0],Rm);
        
        double r31 = Rm.at<double>(2,0);
        dOpencvYaw = asin(-r31);
        if(dMarkerDepth >=0.2){
            cv::Vec3f corner_ld, corner_lu, corner_ru, corner_rd;
            cv::Vec3f tan_ld, tan_lu, tan_rd, tan_ru;


            //0: lu, 1: ru, 2: rd, 3: ld
            corner_lu(0) = _vp3Corner[0].x;
            corner_lu(1) = _vp3Corner[0].y;
            corner_lu(2) = _vp3Corner[0].z;

            corner_ru(0) = _vp3Corner[1].x;
            corner_ru(1) = _vp3Corner[1].y;
            corner_ru(2) = _vp3Corner[1].z;
        
            corner_rd(0) = _vp3Corner[2].x;
            corner_rd(1) = _vp3Corner[2].y;
            corner_rd(2) = _vp3Corner[2].z;

            corner_ld(0) = _vp3Corner[3].x;
            corner_ld(1) = _vp3Corner[3].y;
            corner_ld(2) = _vp3Corner[3].z;

            tan_ld = (corner_lu - corner_ld).cross((corner_rd - corner_ld));
            tan_lu = (corner_ru - corner_lu).cross((corner_ld - corner_lu));
            tan_rd = (corner_ld - corner_rd).cross((corner_ru - corner_rd));
            tan_ru = (corner_rd - corner_ru).cross((corner_lu - corner_ru));

            cv::Vec3f avgT = (tan_ld+tan_lu+tan_ru+tan_rd)/4;

            cv::Point2f projection_point;
            projection_point.x = avgT(2);
            projection_point.y = avgT(0);

            
            dRealSenseYaw = atan(projection_point.y/projection_point.x);
        }
        _YawFusion(dOpencvYaw, dRealSenseYaw);   
        _dYaw_DEG = _dYaw*(180/3.14);

        // std::cout << "opencv yaw : " << dOpencvYaw*(180/3.14) << std::endl;
        // std::cout << "My Yaw : " << dRealSenseYaw*(180/3.14) <<std::endl;
        //std::cout << "Fusion : " << _dYaw_DEG << std::endl;
    }
    void DockingController::_YawFusion(double opencv_yaw, double my_yaw){
        if(_dp3Center.z < 0.2){
            _dYaw = opencv_yaw;
        }else{
            if(opencv_yaw >=0){
                _dYaw = abs(my_yaw);
            }else{
                _dYaw = -abs(my_yaw);
            }
        }
        if(isnan(my_yaw))_dYaw = opencv_yaw;
    }
    void DockingController::PDcontrol(cv::Vec2f dPastLError, double dPastAError){
        _dPastLError = dPastLError;
        _dPastAError = dPastAError;
        double dt = 0.1;
        //LError[0] : Forward Error, LError[1] : Side Error, AError : Yaw Error
        geometry_msgs::Twist PDvel;
        double dAKp = A_Kp;
        double dAKd = A_Kd;
        double dLxKp = LX_Kp;
        double dLxKd = LX_Kd;
        double dLyKp = LY_Kp;
        double dLyKd = LY_Kd;

        PDvel.angular.z = dAKp*_dCurrAError + dAKd*(_dCurrAError-_dPastAError);

        //PDvel.linear.x = dLxKp*_dCurrLError(0) + dLxKd*(_dCurrLError(0)-_dPastLError(0))/dt;
        if(abs(_dCurrLError(0))>0.03){
            if(_dCurrLError(0)>0)PDvel.linear.x = 0.3;
            else if(_dCurrLError(0)<0)PDvel.linear.x = -0.3;
        }else{PDvel.linear.x = 0;}

        PDvel.linear.y = dLyKp*_dCurrLError(1) + dLyKd*(_dCurrLError(1)-_dPastLError(1))/dt;

        if(abs(_dCurrLError(0)) < 0.55 && abs(_dCurrLError(1)) > 0.1){
            PDvel.linear.x = 0;
            PDvel.angular.z = 0;
        }

        _cmd_vel = PDvel;
    }
}
