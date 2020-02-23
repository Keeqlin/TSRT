#ifndef VISULIZER_H
#define VISULIZER_H

#include "cvEigenConverter.h"
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include <boost/algorithm/string.hpp>


namespace VISUALIZER{

#define RED cv::Vec3b(0,0,255)
#define BLUE cv::Vec3b(255, 0, 0)
#define GREEN cv::Vec3b(0,255,0)
#define YELLOW cv::Vec3b(0,255,255)
#define WHITE cv::Vec3b(255, 255, 255)

//  right-hand coordinate systm (unit: m)
//  (+x,+y,+z): right, down, front
//  (x-axis, y-axis, z-axis): pitch, yaw, roll 

inline cv::Point2f pixelToCam(const cv::Point& pixel, const cv::Size& size){
    cv::Point2f fpt(pixel.x, pixel.y);
    fpt.x -= static_cast<float>(size.width-1)/2;
    fpt.y -= static_cast<float>(size.height-1)/2;
    return fpt;
}

class GBRxyzPt{
public:
    explicit GBRxyzPt(){xyz = Eigen::Vector3f(0,0,0); GBR = cv::Vec3f(0,0,0);}
    explicit GBRxyzPt(const Eigen::Vector3f _xyz, cv::Vec3b _GBR):xyz(_xyz), GBR(_GBR){}
    Eigen::Vector3f xyz;
    cv::Vec3b GBR;
};

inline std::ostream &operator<<(std::ostream &os, const GBRxyzPt &rhs){
    os << "xyz"<< rhs.xyz<<", GBR"<< rhs.GBR;
    return os;
} /*-------------------IMUDATA-------------------*/

class Pose{
public:
    explicit Pose(){
        Rcw = Rwc = Eigen::Matrix3f::Identity();
        tcw = twc = Eigen::Vector3f(0,0,0);
    }
    explicit Pose(const Eigen::Matrix3f& _Rwc, const Eigen::Vector3f& _twc):Rwc(_Rwc), twc(_twc){
        Rcw = Rwc.inverse();
        tcw = -Rcw*twc;
    }
    inline Eigen::Vector3f PwToPc(const Eigen::Vector3f& pt_world){
        auto pt_camera = Rcw*pt_world + tcw;
        return pt_camera;
    }
    inline Eigen::Vector3f PcToPw(const Eigen::Vector3f &pt_camera){
        auto pt_world = Rwc * pt_camera + twc;
        return pt_world;
    }
    inline bool isorigin(){
        return (Rcw == Eigen::Matrix3f::Identity() && twc == Eigen::Vector3f(0, 0, 0)) ? true : false;
    }
    //camera coordination system to world coordination system
    Eigen::Matrix3f Rwc;
    Eigen::Vector3f twc;
    //wourld coordination system to camera coordination system
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
};

class Camera_Viewer{
public:
    explicit Camera_Viewer(Eigen::Matrix3f& _K):K(_K){
        projected_Img = cv::Mat::zeros(cv::Size(K(0,2)*2,K(1,2)*2),CV_8UC3);
    }
    explicit Camera_Viewer(Eigen::Matrix3f& _K, const Pose& _pose):K(_K), pose(_pose){
        projected_Img = cv::Mat::zeros(cv::Size(K(0,2)*2,K(1,2)*2),CV_8UC3);
    }
    void projectToImg(const std::vector<GBRxyzPt>& pointcloud);
    cv::Mat getImg() const;
 protected:
    bool out_of_Img(int col, int row) const;
    cv::Point2f projection(const Eigen::Vector3f& pt_world);
 private:
     Pose pose;           // world coordinate system
     Eigen::Matrix3f K;   // Intrinsics matrix
     // Eigen::VectorXf distCoeffs;
     cv::Mat projected_Img;
};

// class visualizer{

// private:

    
// };

class Obj{
public:
    Obj(){}
    Obj(const Pose& _pose):pose(_pose){}
    std::vector<GBRxyzPt>& get() { return pointcloud;}
protected :
    void PposeToPw();
    std::vector<GBRxyzPt> pointcloud;
    Pose pose;
    std::string Name;
};

class TSR_Obj: public Obj{
public:
    explicit TSR_Obj(cv::Mat TSR_img, double TSR_height_pose, double TSR_depth_pose, const Pose &TSR_pose = Pose());
};

class LANE_Obj : public Obj{
public:
    explicit LANE_Obj(double LANE_len, const cv::Vec3b &color, const cv::Point &dash_para = cv::Point(), const Pose &LANE_pose = Pose());
};

}  // namespace VISUALIZER
#endif // VISULIZER_H