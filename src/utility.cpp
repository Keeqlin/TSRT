#include "utility.h"


Eigen::Matrix3d Camera_Motion::K;
        
std::vector<std::string> getline_and_prasingstr(std::fstream& fs, const std::string& delim){
    std::vector<std::string> vstr;
    std::string line;
    getline(fs,line);
    boost::split(vstr, line, boost::is_any_of(delim),boost::token_compress_on);
    return vstr;
}

//split video into sub track
void crop_video(const std::string& raw_video, const std::string& croped_video, int start_sec, int lasting_sec){
    cv::VideoCapture cap(raw_video.c_str()); 
    if(!cap.isOpened())
        WARNING_MSG_EXIT("Error opening video: "+raw_video);

    int FPS = cap.get(CV_CAP_PROP_FPS);
    cv::Size size(cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    
    //set output video
    cv::VideoWriter video(croped_video,CV_FOURCC('m','p','4','v'),FPS, size);
    cv::Mat img;

    cap.set(CV_CAP_PROP_POS_FRAMES,FPS*start_sec-1);
    for(int i = lasting_sec*FPS; i>=0; i--){
        cap.read(img);
        video << img;
        cv::imshow("croped video", img);
        cv::waitKey(FPS);
    }
    cap.release();
    video.release();
    
    std::cout<<"Generating "<<croped_video<<" with "<<FPS*lasting_sec<<" frames, FPS("<<FPS<<"), Size("<<size<<")..."<<std::endl<<std::endl;
}


cv::Point2d vpt[10];
int labeled_num_pt = 0;
cv::Mat annoted_Img;
cv::Mat draw_img;
std::string winName = "Label pt";
bool labeled_flag = false;

void on_mouse(int EVENT, int x, int y, int flags, void* ustc)
{   
    if(labeled_num_pt == 0)
        draw_img = annoted_Img.clone(); 

    auto draw_pt = [&](){
        draw_img = annoted_Img.clone();
        for(int i =0; i<labeled_num_pt; i++)
            cv::circle(draw_img,vpt[i],3,RED,-1);
        cv::imshow(winName, draw_img);
    };
    //label points
	if(EVENT == CV_EVENT_LBUTTONDOWN){
		vpt[labeled_num_pt] = cv::Point2d(x,y);
        labeled_num_pt++;    
        draw_pt();
	}

	if(EVENT == CV_EVENT_RBUTTONUP){
        if(labeled_flag)
            labeled_flag = false;

        if(labeled_num_pt > 0)
            labeled_num_pt--;
        draw_pt();
	}
		
    if(labeled_flag){
        for(int i=0;i<labeled_num_pt-1;i++)
            cv::line(draw_img,vpt[i],vpt[i+1],BLUE,2);
        cv::line(draw_img,vpt[labeled_num_pt-1],vpt[0],BLUE,2);
    }

    cv::imshow(winName, draw_img);
    if( cv::waitKey(0) == 'f'){
        if(!labeled_flag)
            labeled_flag = true;
    }

    if(!labeled_flag){
        char key = cv::waitKey(0);    
        switch (key){
        case 'a':
            vpt[labeled_num_pt-1].x--;
            draw_pt();
            break;
        case 'd':
            vpt[labeled_num_pt-1].x++;
            draw_pt();
            break;
        case 'w':
            vpt[labeled_num_pt-1].y--;
            draw_pt();
            break;
        case 's':
            vpt[labeled_num_pt-1].y++;
            draw_pt();
            break;
        default:
            break;
        }
    }
}


void Label_GW_TSR_pt(const std::string& video_path){
    std::string gt_path = video_path.substr(0,video_path.find(".mp4"))+"_GT.txt";
    std::fstream fs(gt_path.c_str(), std::ios::out|std::ios::trunc);
    if(!fs)
        WARNING_MSG_EXIT("Fail to open Ground Truth txt");

    cv::VideoCapture cap(video_path); 
    if(!cap.isOpened())
        WARNING_MSG_EXIT("Error opening video..");
	
    std::cout<<video_path<<" has "<<cap.get(CV_CAP_PROP_FRAME_COUNT)<<std::endl;
	cv::Mat Img;
	int frameIdx = 0;
	// extern bool labeled_flag;
	// extern int labeled_num_pt;
	// extern cv::Point2d vpt[10];
	while(cap.read(Img)){
		// extern std::string winName;
		// extern cv::Mat annoted_Img;
		annoted_Img = Img.clone();
		cv::imshow(winName, annoted_Img);
		std::stringstream ss;
        while(true){
			cv::setMouseCallback(winName, on_mouse, NULL);
			if(cv::waitKey(0) == 'q'){
				std::cout<<"save ";
                ss<<frameIdx;
                fs<<frameIdx;
				for(int i =0;i<labeled_num_pt;i++){
                    ss<<','<<vpt[i];
                    fs<<','<<vpt[i];
                }
                ss<<std::endl;
                fs<<std::endl;
                std::cout<<ss.str();
            
				labeled_flag = false;
				labeled_num_pt = 0;
				break;
			}
		}
		frameIdx++;
	}

    fs.close();

}

void TEST_HOMO(const std::string& video_path){

    std::string gt_path = video_path.substr(0,video_path.find(".mp4"))+"_GT.txt";
    std::fstream fs(gt_path.c_str(), std::ios::in);
    if(!fs)
        WARNING_MSG_EXIT("Fail to open Ground Truth txt");

    cv::VideoCapture cap(video_path); 
    if(!cap.isOpened())
        WARNING_MSG_EXIT("Error opening video..");

    cv::Mat Img;
    cv::Mat OriImg; 
    cv::Mat Homography;
    int length = 300;
    cv::Point Homo_pt[4]={cv::Point(0,0),cv::Point(length,0),cv::Point(length,length),cv::Point(0,length)};
    int frameIdx = 0;
    int pt_num = 0;
    int FPS = cap.get(CV_CAP_PROP_FPS);
    auto draw_line = [&](cv::Point* arr){
        for(int i =0; i<pt_num; i++){
            cv::circle(Img,arr[i],3,RED,-1);
            if(i!=pt_num-1)
                cv::line(Img,arr[i],arr[i+1],BLUE,2);
            if(i==pt_num-1)
                cv::line(Img,arr[pt_num-1],arr[0],BLUE,2);
        }
    };
    while(cap.read(Img)){
        OriImg = Img.clone();
        auto vstr = getline_and_prasingstr(fs, " ,[]");
        if(vstr.size()>1){
            pt_num = (vstr.size()-1)/2;
            cv::Point labeled_pts[pt_num];
            for(int i= 1; i<=pt_num; i++){
                labeled_pts[i-1] = cv::Point(std::stoi(vstr[i*2-1]),std::stoi(vstr[i*2]));
            }
            draw_line(labeled_pts);

            // cv::Mat h = cv::findHomography(arr2vec(labeled_pts,4),arr2vec(Homo_pt,4));
            // cv::warpPerspective(OriImg,Homography,h,cv::Size(length,length));
            // cv::Point2f Homo_center = cv::Point2f(length/2,length/2);
            // cv::circle(Homography,Homo_center,2,GREEN,-1);            
            // cv::imshow("HOMO_trans",Homography);   

            // cv::Mat inverse;
            // cv::Point2f inv_Homo_center = h.inv()*Homo_center;
            // cv::warpPerspective(Homography,inverse,h.inv(),Img.size());
            // cv::imshow("Inv",inverse);
            // cv::circle(Img,inv_Homo_center,2,GREEN,-1);

            cv::Point2f bbox_center = cv::Point2f(0,0);
            for(int i =0; i<pt_num; i++){
                bbox_center.x += labeled_pts[i].x;
                bbox_center.y += labeled_pts[i].y;
            }
            bbox_center /= 4;
            cv::circle(Img,bbox_center,2,RED,-1);

        }


        cv::imshow("HOMO",Img);
        cv::waitKey(0);
        // cv::waitKey(FPS);
    }
}

std::vector<cv::Point> arr2vec(cv::Point* arr, int num){
    std::vector<cv::Point> vecpt;
    for(int i =0; i<num; i++){
        vecpt.push_back(arr[i]);
    }
    return vecpt;
}


cv::Point2f operator*(cv::Mat M, const cv::Point2f& p)
{ 
    cv::Mat_<double> src(3/*rows*/,1 /* cols */); 

    src(0,0)=p.x; 
    src(1,0)=p.y; 
    src(2,0)=1.0; 

    cv::Mat_<double> dst = M*src; //USE MATRIX ALGEBRA 
    return cv::Point2f(dst(0,0),dst(1,0)); 
} 




cv::Mat Matrix3dtoCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);
    return cvMat.clone();
}


Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat& cvMat3)
{
    Eigen::Matrix<double,3,3> M;
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            M(i,j) = cvMat3.at<double>(i,j);
    return M;
}


Eigen::Vector3d Pixe2DtoPt3D(double pixel_x, double pixel_y, cv::Size img_size, double height, double depth){
    
    pixel_x = pixel_x - img_size.width/2;
    pixel_y = pixel_y - img_size.height/2;
    // height -= img_size.height/(2*100);
    pixel_x /= 100; //cm->m
    pixel_y /= 100; //cm->m
    return Eigen::Vector3d(pixel_x,pixel_y+height,depth); // m
}

double degTorad(double deg){
    if(deg<0)
        deg+=360;
    return deg*M_PI/180;
}

double radTodeg(double rad){
    return rad/M_PI*180;
}

cv::Point pinhole_inv(int x, int y,cv::Size size){
    return cv::Point(size.width-1-x,size.height-1-y);
}

bool out_of_Img(const cv::Point& pt, cv::Size size){
    if(pt.y<size.height && pt.x<size.width && pt.x>=0 && pt.y>= 0)
        return false;
    return true;
}


void pose_recording(std::fstream& os, cv::Mat& R, cv::Mat& T){
    double yaw,pitch,roll;
    Eigen::Vector3d eigen_T(T.at<double>(0,0),T.at<double>(0,1),T.at<double>(0,2));
    getAnglesformR(R, pitch, yaw, roll);

    eigen_T = -toMatrix3d(R).inverse()*eigen_T;
    getAnglesformR(R.inv(), pitch, yaw, roll);
    os<<yaw<<","<<pitch<<","<<roll<<","
      <<eigen_T.x()<<","<<eigen_T.y()<<","<<eigen_T.z()<<std::endl;
}



void getAnglesformR(cv::Mat R, double &angleX, double &angleY, double &angleZ)
{
    cv::Mat1d rvec;
    cv::Rodrigues(R, rvec);
    rvec = rvec*180/CV_PI;
    angleX = rvec(0);
    angleY = rvec(1);
    angleZ = rvec(2);

    auto T = [](double& deg){
        deg += 180;
        if(deg>360)
            deg -= 360;
        if(deg>359 && deg<360)
            deg = 360 - deg;
    };
//     T(angleX);
//     T(angleY);
//     T(angleZ);
}

double trajectory_tan(double heading_dis, cv::Point2d origin, double width, double length, const std::string& Turn_Dir){
    // for tan(x) = y
    // y: +-6
    // x: +-1.405647 rad
    double tan_thetalim = 1.405647;
    double tan_ylim = 6;
    double y_scale = length/(2*tan_ylim);
    double x_scale = width/(2*tan_thetalim);
    cv::Point2d tan_origin(-tan_thetalim,-tan_ylim);
    double var_x;
    

    if(origin.y > heading_dis){
        var_x = 0;
    }
    else if((heading_dis-origin.y)>length){
        var_x = width;
    }else{
        double scaled_y = (heading_dis-origin.y)/y_scale;
        double tan_theta = atan(scaled_y-tan_ylim);
        var_x = (tan_theta+tan_thetalim)*x_scale;
    }

    if(Turn_Dir == "R")
        return origin.x + var_x;
    if(Turn_Dir == "L")
        return origin.x - var_x;
}

void MapToCamera(std::vector<Eigen::Vector3d> pts, const Camera_Motion& state,cv::Mat stimulated_img, const cv::Vec3b& color,const cv::Point& interval){
    Eigen::Vector3d vp(state.T(0),state.T(1),1000); // very far distance
    Eigen::Vector3d vp_camera = state.R.inverse()*(vp-state.T); 
    // Eigen::Vector3d vp_camera = state.R.inverse()*(vp)-state.T; 

	vp_camera /= vp_camera.z();
	Eigen::Vector3d mapped_vp = state.K*(vp_camera);
    // cv::Point cv_mapped_vp(pinhole_inv(mapped_vp.x(),mapped_vp.y(),stimulated_img.size()));
    cv::Point cv_mapped_vp(mapped_vp.x(),mapped_vp.y());

    for(auto& pt_world:pts){
        if( interval.x != 0 && pt_world.y() == 0 && (static_cast<int>(pt_world.z()*100)%interval.x > interval.y))
            continue;
        Eigen::Vector3d pt_camera = state.R.inverse()*(pt_world-state.T); 
		pt_camera /= pt_camera.z();
		Eigen::Vector3d mapped_Pt = state.K*(pt_camera);
        cv::Point cv_mapped_Pt(mapped_Pt.x(),mapped_Pt.y());
        //check the vanish point for ground object
        bool vp_flag = ((pt_world.y() == 0) && (cv_mapped_vp.y>cv_mapped_Pt.y)) ?true:false;
		if(!out_of_Img(cv_mapped_Pt,stimulated_img.size() ) && !vp_flag)
		    stimulated_img.at<cv::Vec3b>(cv_mapped_Pt) = color;
	}
}



double dis_Vector3d(const Eigen::Vector3d& lhs,const Eigen::Vector3d& rhs){
    double res = std::pow(std::abs(rhs.x()-lhs.x()),2) + std::pow(std::abs(rhs.y()-lhs.y()),2) + std::pow(std::abs(rhs.z()-lhs.z()),2);
    return std::sqrt(res);
}




// void camera_pose_pnp(std::vector<cv::Point3f>& marker3d, std::vector<cv::Point2f>& mapped_marker2d, cv::Mat K){
// 	cv::Mat Rvec;
//     cv::Mat_<float> Tvec;
//     cv::Mat raux, taux;
//     cv::Mat_<float> rotMat(3, 3);
// 	cv::solvePnP( marker3d, mapped_marker2d , K, cv::noArray(), raux, taux);
// 	raux.convertTo(Rvec, CV_32F);    //旋转向量
//     taux.convertTo(Tvec, CV_32F);   //平移向量
//     cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵

//     //格式转换
//     auto R_n = toMatrix3d(rotMat);
//     Eigen::Vector3d T_n(Tvec.at<float>(0,0),Tvec.at<float>(0,1),Tvec.at<float>(0,2));
//     Eigen::Vector3d P_oc =  -R_n.inverse()*T_n;
// 	std::cout<<"P_camera = "<<P_oc.transpose()<<std::endl;
// }