#include "visualizer.h"

namespace VISUALIZER{


cv::Mat Camera_Viewer::getImg() const{
    return projected_Img.clone();
}

void Camera_Viewer::setPose(const Pose& _pose){
    pose = _pose;
    projected_Img = cv::Mat::zeros(projected_Img.size(),CV_8UC3);
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

TS_Rect::TS_Rect(cv::Mat _TS_img, double TS_height_pose, double TS_depth_pose, const Pose &TS_pose) : Obj(TS_pose){
    TS_img = _TS_img.clone();
    pointcloud.reserve(TS_img.cols*TS_img.rows+5*TS_depth_pose*100); // 1 pixel = 1 cm
    //obj creation based on current pose
    for(int y=0; y<TS_img.rows; y++)
        for(int x=0; x<TS_img.cols; x++){
            auto pt = pixelToCam(cv::Point(x,y),TS_img.size());
            GBRxyzPt Pt(Eigen::Vector3f(static_cast<float>(pt.x)/100, static_cast<float>(pt.y)/100+TS_height_pose, TS_depth_pose), TS_img.at<cv::Vec3b>(y, x));
            if(y==0 && x==0)
				Conrner_3D[0] = Pt.xyz;
			if(y==0 && x==TS_img.cols-1)
				Conrner_3D[1] = Pt.xyz;
			if(y==TS_img.rows-1 && x==TS_img.cols-1)
				Conrner_3D[2] = Pt.xyz;
			if(y==TS_img.rows-1 && x==0)
				Conrner_3D[3] = Pt.xyz;

            pointcloud.push_back(std::move(Pt));
        }
    int TS_bar_width = 5; //cm
    TS_height_pose += static_cast<float>(TS_img.rows)/200;
    for(int x = -TS_bar_width / 2; x <= TS_bar_width / 2; x++) 
        for(int y = TS_height_pose * 100; y < 0; y++){
            GBRxyzPt Pt(Eigen::Vector3f(static_cast<float>(x)/100, static_cast<float>(y)/100, TS_depth_pose), WHITE);
            pointcloud.push_back(std::move(Pt));
        }
    //convert pts to world coordinate system
    PposeToPw();
}

void TS_Rect::cal_rect_ref(const cv::Size& Mapping_size){
    double ratio = (Mapping_size.width > Mapping_size.height) ? static_cast<double>(Mapping_size.height) / static_cast<double>(TS_img.rows) : static_cast<double>(Mapping_size.width) / static_cast<double>(TS_img.cols);
	// std::cout << "ratio: " << ratio << std::endl;
	cv::Point2f Mapping_center = cv::Point2f((Mapping_size.width-1)/2, (Mapping_size.height-1)/2);
    rect_ref.ptArr.get()[0] = (Mapping_center + ratio * pixelToCam(cv::Point(0,0),TS_img.size()));
    rect_ref.ptArr.get()[1] = (Mapping_center + ratio * pixelToCam(cv::Point(TS_img.cols-1,0),TS_img.size()));
    rect_ref.ptArr.get()[2] = (Mapping_center + ratio * pixelToCam(cv::Point(TS_img.cols-1,TS_img.rows-1),TS_img.size()));
    rect_ref.ptArr.get()[3] = (Mapping_center + ratio * pixelToCam(cv::Point(0,TS_img.rows-1),TS_img.size()));
}

void TS_Rect::cal_rect_proj(std::function<cv::Point2f(const Eigen::Vector3f&)> projection){
    for(int i=0; i<4; i++)
        rect_proj.ptArr.get()[i] = projection(Conrner_3D[i]);
}

LANE::LANE(double LANE_len, const cv::Vec3b &color, const cv::Point &dash_para, const Pose &LANE_pose):Obj(LANE_pose){
    pointcloud.reserve(LANE_len*100);
    //obj creation based on current pose
    for(int i = 0; i<LANE_len*100; i++){
        GBRxyzPt Pt(Eigen::Vector3f(0,0,static_cast<float>(i)/100),color);
        pointcloud.push_back(std::move(Pt));
    }
    //convert pts to world coordinate system
    PposeToPw();
}


LANE::LANE(const std::vector<GBRxyzPt>& pts, const cv::Point &dash_para, const Pose &LANE_pose):Obj(LANE_pose){
    pointcloud = std::move(pts);
    //convert pts to world coordinate system
    PposeToPw();
}


} //namespace VISUALIZER