#include "controller.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/highgui/highgui.hpp>
#include "ControlSystem.h"

#define ALLOWABLE_ERROR_X 0.06 //Robot forward(m)
#define ALLOWABLE_ERROR_Y 0.02 //Robot Side
#define ALLOWABLE_ERROR_A 5 //Robot Angle(deg)

//main controller : calculate Yaw, Distance and PD Control
geometry_msgs::Twist controller::main_controller(cv::Mat BackColorImg, cv::Mat BackDepthImg, std::vector<cv::Point2f>corner, cv::Vec2f& dPastLError, double& dPastAError, double& Mdistance){
    //double dCurrLError = 0;
    //double dCurrAError = 0;

    geometry_msgs::Twist cmd_vel;
    cv::Vec2f dPLE = dPastLError;
    double dPAE = dPastAError;

    ControlSystem::DockingController Controller(BackColorImg, BackDepthImg, corner);

    Controller.CalculateError();

    std::cout << "Angular_error : " << Controller.CurrentAngularError() << std::endl;
    std::cout << "X_error : " << Controller.CurrentLinearError()(0) << std::endl;
    std::cout << "Y_error : " << Controller.CurrentLinearError()(1) << std::endl;

    if(abs(Controller.CurrentLinearError()(0)) < ALLOWABLE_ERROR_X && abs(Controller.CurrentLinearError()(1)) < ALLOWABLE_ERROR_Y && abs(Controller.CurrentAngularError()) < ALLOWABLE_ERROR_A){
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0;
        return cmd_vel;
    }

    Controller.PDcontrol(dPLE, dPAE);

    Mdistance = Controller.Mdistance();
    cmd_vel = Controller.cmd_vel();

    if(abs(Controller.CurrentLinearError()(0)) < ALLOWABLE_ERROR_X) cmd_vel.linear.x = 0;
    if(abs(Controller.CurrentLinearError()(1)) < ALLOWABLE_ERROR_Y) cmd_vel.linear.y = 0;
    if(abs(Controller.CurrentAngularError())   < ALLOWABLE_ERROR_A) cmd_vel.angular.z = 0;

    //std::cout << dPastAError <<std::endl << dPastLError <<std::endl;
    //update Past error
    dPastLError = Controller.CurrentLinearError();
    dPastAError = Controller.CurrentAngularError();

    return cmd_vel;
}

