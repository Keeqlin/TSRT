#include "visualizer.h"

namespace VISUALIZER{


Obj::Obj(const cv::Mat RGB_img, const std::string& _Name):Name(_Name){
    
}


void Camera_Viewer::projectToImg(const std::vector<GBRxyzPt>& pointcloud){
    auto Rcw = Rwc.inverse();
    auto tcw = -Rwc.inverse()*twc;
    // vanish point: very far point
    Eigen::Vector3f vp_world(twc.x(),twc.y(),1000); 
    cv::Point2f vp_img = projection(Rcw,tcw,vp_world);

    auto check_vp = [&vp_img](const Eigen::Vector3f& pt_world,const cv::Point2f& pt_img){
        if(pt_world.y()==0 && pt_img.y<vp_img.y){
            return true;
        }
        return false;
    };

    for(auto& pt: pointcloud){
        cv::Point2f pt_img = projection(Rcw,tcw,pt.xyz);
        if(!out_of_Img(pt_img.x,pt_img.y)&& !check_vp(pt.xyz,pt_img)){
            projected_Img.at<cv::Vec3b>(pt_img.y,pt_img.x) = pt.GBR;
        }   
    }
}

bool Camera_Viewer::out_of_Img(int col, int row) const{
    if(col>=0 && row>=0)
        if(col<projected_Img.cols && row<projected_Img.rows)
            return false;
    return true;
}

cv::Point2f Camera_Viewer::projection(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const Eigen::Vector3f& pt_world){
    Eigen::Vector3f pt_camera = Rcw*pt_world + tcw;
    // normalized plane: z = 1, in front of center of camera
    pt_camera /= pt_camera.z(); 
    auto pt_img = K*pt_camera;
    return cv::Point2f(pt_img.x(),pt_img.y());
}

} //namespace VISUALIZER