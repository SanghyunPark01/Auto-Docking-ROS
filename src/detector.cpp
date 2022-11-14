#include "detector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>


#define MARKER_TARGET 23
double dU0 = 324.7346;
double dV0 = 246.24308;
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

class marker{
private:
    std::vector<cv::Point2f> _corners;
    cv::Point2f _center;
public:
    marker(std::vector<cv::Point2f> corners){
        _corners = corners;
        for (const auto& corner : corners)
        {
            _center += corner;
        }
        _center = _center/4;
    }
    cv::Point2f center(void){
        return _center;
    }
    std::vector<cv::Point2f> corners(void){
        return _corners;
    }
};

namespace Detector{
    bool FrontDetect_OneMarkers(std::vector<cv::Point2f>& vP2_Fcorners, cv::Mat cImg){
        std::vector<int> fids;
        std::vector<int> good_ids;
        
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<std::vector<cv::Point2f>> good_corners;
        
        cv::aruco::detectMarkers(cImg, dictionary, corners,fids);
        if(fids.size()<=0) return false;
        for(int i = 0 ; i< fids.size();i++){
            if(fids[i] == MARKER_TARGET){
                good_ids.push_back(fids[i]);
            }
        }
        if(good_ids.size()<=0 || good_ids.size()>1) return false;
        return true;
    }
    bool BackDetect_OneMarkers(std::vector<cv::Point2f>& vP2_Bcorners, cv::Mat cImg){
        
        std::vector<int> b_ids;
        std::vector<int> good_ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<std::vector<cv::Point2f>> good_corners;
        std::vector<cv::Point2f> one_good_corners;
        cv::aruco::detectMarkers(cImg, dictionary, corners,b_ids);
        if(b_ids.size()<=0) return false;
        for(int i = 0 ; i< b_ids.size();i++){
            if(b_ids[i] == MARKER_TARGET){
                good_ids.push_back(b_ids[i]);
                good_corners.push_back(corners[i]);
            }
        }
        if(good_ids.size() <= 0)return false;
        std::vector<marker> marker_info;
        if(good_corners.size() == 1){
            vP2_Bcorners = good_corners[0];
            //std::cout<<good_corners[0][0].x<<std::endl;
            return true;
        }
        for(int i = 0; i < good_ids.size(); i++){
            marker temp(corners[i]);
            marker_info.push_back(temp);
        }
        marker one_good_marker = marker_info[0];
        for(int i = 0; i < good_ids.size(); i++){
            one_good_marker = abs(one_good_marker.center().x-dU0) < abs(marker_info[i].center().x-dU0) ? one_good_marker : marker_info[i];
        }
        vP2_Bcorners = one_good_marker.corners();

        return true;
    }

}