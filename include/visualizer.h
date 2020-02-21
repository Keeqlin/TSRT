#ifndef VISULIZER_H
#define VISULIZER_H

#include "cvEigenConverter.h"

#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include <boost/algorithm/string.hpp>


namespace VISUALIZER{

#define RED (0,0,255)
#define BLUE (255,0,0)
#define GREEN (0,255,0)
#define YELLOW (0,255,255)
#define WHITE (255,255,255)


//  right-hand coordinate systm
//  (+x,+y,+z): right, down, front
//  (x-axis, y-axis, z-axis): pitch, yaw, roll 
class GBRxyzPt{
public:
    explicit GBRxyzPt(){xyz = Eigen::Vector3f(0,0,0); GBR = cv::Vec3f(0,0,0);}
    explicit GBRxyzPt(const Eigen::Vector3f _xyz, cv::Vec3b _GBR):xyz(_xyz), GBR(_GBR){}
    Eigen::Vector3f xyz;
    cv::Vec3b GBR;
};

class Obj{
    explicit Obj(const cv::Mat RGB_img, const std::string& _Name);
protected:
    std::vector<GBRxyzPt> pointcloud;
    std::string Name;
};

class Camera_Viewer{
public:
    explicit Camera_Viewer(Eigen::Matrix3f& _K):K(_K){
        Rwc = Eigen::Matrix3f::Identity(); 
        twc = Eigen::Vector3f(0,0,0);
        projected_Img = cv::Mat::zeros(cv::Size(K(0,2)*2,K(1,2)*2),CV_8UC3);
    }
    explicit Camera_Viewer(Eigen::Matrix3f& _K, Eigen::Matrix3f& _Rwc, Eigen::Vector3f& _twc):K(_K), Rwc(_Rwc), twc(_twc){
        projected_Img = cv::Mat::zeros(cv::Size(K(0,2)*2,K(1,2)*2),CV_8UC3);
    }
    void projectToImg(const std::vector<GBRxyzPt>& pointcloud);
 protected:
    bool out_of_Img(int col, int row) const;
    cv::Point2f projection(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const Eigen::Vector3f& pt_world);

 private:
    Eigen::Matrix3f Rwc; // world coordinate system
    Eigen::Vector3f twc; // world coordinate system
    Eigen::Matrix3f K;   // Intrinsics matrix
    // Eigen::VectorXf distCoeffs;
    cv::Mat projected_Img;
};

// class visualizer{

// private:

    
// };

}  // namespace VISUALIZER
#endif // VISULIZER_H