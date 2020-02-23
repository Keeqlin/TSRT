#include "visualizer.h"

namespace VISUALIZER{


cv::Mat Camera_Viewer::getImg() const{
    return projected_Img.clone();
}

void Camera_Viewer::projectToImg(const std::vector<GBRxyzPt>& pointcloud){
    // vanish point: very far point
    Eigen::Vector3f vp_world(pose.twc.x(),pose.twc.y(),1000);
    cv::Point2f vp_img = projection(vp_world);

    auto check_vp = [&vp_img](const Eigen::Vector3f& pt_world,const cv::Point2f& pt_img){
        return (pt_world.y()==0 && pt_img.y<vp_img.y)?true:false;
    };

    for(auto& pt: pointcloud){
        cv::Point2f pt_img = projection(pt.xyz);
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

cv::Point2f Camera_Viewer::projection(const Eigen::Vector3f &pt_world){
    Eigen::Vector3f pt_camera = pose.PwToPc(pt_world);
    // normalized plane: z = 1, in front of center of camera
    pt_camera /= pt_camera.z();
    auto pt_img = K*pt_camera;
    return cv::Point2f(pt_img.x(),pt_img.y());
}

void Obj::PposeToPw(){
    if (!pose.isorigin()){
        for (auto &Pt : pointcloud)
            Pt.xyz = pose.PcToPw(Pt.xyz);
    }
}

TSR_Obj::TSR_Obj(cv::Mat TSR_img, double TSR_height_pose, double TSR_depth_pose, const Pose &TSR_pose) : Obj(TSR_pose){
    pointcloud.reserve(TSR_img.cols*TSR_img.rows+5*TSR_depth_pose*100); // 1 pixel = 1 cm
    //obj creation based on current pose
    for(int y=0; y<TSR_img.rows; y++)
        for(int x=0; x<TSR_img.cols; x++){
            auto pt = pixelToCam(cv::Point(x,y),TSR_img.size());
            GBRxyzPt Pt(Eigen::Vector3f(static_cast<float>(pt.x)/100, static_cast<float>(pt.y)/100+TSR_height_pose, TSR_depth_pose), TSR_img.at<cv::Vec3b>(y, x));
            pointcloud.push_back(Pt);
        }
    int TSR_bar_width = 5; //cm
    TSR_height_pose += static_cast<float>(TSR_img.rows)/200;
    for(int x = -TSR_bar_width / 2; x <= TSR_bar_width / 2; x++) 
        for(int y = TSR_height_pose * 100; y < 0; y++){
            GBRxyzPt Pt(Eigen::Vector3f(static_cast<float>(x)/100, static_cast<float>(y)/100, TSR_depth_pose), WHITE);
            pointcloud.push_back(Pt);
        }
    //convert pts to world coordinate system
    PposeToPw();
}

LANE_Obj::LANE_Obj(double LANE_len, const cv::Vec3b &color, const cv::Point &dash_para, const Pose &LANE_pose):Obj(LANE_pose){
    pointcloud.reserve(LANE_len*100);
    //obj creation based on current pose
    for(int i = 0; i<LANE_len*100; i++){
        GBRxyzPt Pt(Eigen::Vector3f(0,0,static_cast<float>(i)/100),color);
        pointcloud.push_back(Pt);
    }
    //convert pts to world coordinate system
    PposeToPw();
}

} //namespace VISUALIZER