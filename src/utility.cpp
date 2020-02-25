#include "utility.h"


        
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









void pose_recording(std::fstream& os, cv::Mat& R, cv::Mat& T){
    // double yaw,pitch,roll;
    Eigen::Vector3f eigen_T(T.at<float>(0,0),T.at<float>(0,1),T.at<float>(0,2));
    // getAnglesformR(R, pitch, yaw, roll);
    cv::Matx33f R33((float*)R.ptr());
    eigen_T = -cvToEigenMat(R33).inverse()*eigen_T;
    Eigen::Vector3f eulerAng = cvToEigenMat(R33).inverse().eulerAngles(2, 1, 0);
    eulerAng = eulerAng / (CV_PI/180);
    float roll, yaw, pitch;
    roll = eulerAng.z();
    yaw = eulerAng.y();
    pitch = eulerAng.x();
    // getAnglesformR(R.inv(), pitch, yaw, roll);
    os<<yaw<<","<<pitch<<","<<roll<<","
      <<eigen_T.x()<<","<<eigen_T.y()<<","<<eigen_T.z()<<std::endl;
}


void pnp(std::vector<cv::Point3f>& Pt_3D, std::vector<cv::Point2f>& projected_pt, cv::Mat K){
    static std::fstream pnp_stream;
    if(!pnp_stream){
        std::string name = "./../pnp_pose";
		pnp_stream.open(std::string(name+".txt").c_str(), std::ios::out | std::ios::trunc);
    }
    
    cv::Mat Rvec;
	cv::Mat_<float> Tvec;
	cv::Mat raux, taux;
	cv::Mat_<float> rotMat(3, 3);
	cv::solvePnP( Pt_3D, projected_pt, K, cv::noArray(), raux, taux);
	raux.convertTo(Rvec, CV_32F);    //旋转向量
	taux.convertTo(Tvec, CV_32F);   //平移向量
	cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
	
    auto Rcw = rotMat.inv();
    auto tcw = -Rcw*Tvec;
    // std::cout<<"tcw: "<<tcw.t()<<"m"<<std::endl;
    // std::cout<<"pyr_cw: "<<-Rvec.t()*DegperRad<<std::endl<<std::endl;
    pnp_stream<<tcw.t()<<','<<-Rvec.t()*DegperRad<<std::endl;
}


void Homography(std::vector<cv::Point2f>& ref_vertex,std::vector<cv::Point2f>& proj_vertex,  cv::Mat K){
    // use for recording pose estimation of decomposedH
    static std::vector<std::fstream> decomposedH_stream;
    if(decomposedH_stream.size() == 0){
        for(int i=0; i<4; i++){
		std::string name = "./../decompose_H_pose";
		std::fstream os(std::string(name+std::to_string(i)+".txt").c_str(), std::ios::out | std::ios::trunc);
		decomposedH_stream.push_back(std::move(os));
	    }
    }
    
    std::vector<cv::Mat> r,t,n;
	cv::Mat H = cv::findHomography(ref_vertex,proj_vertex);
	cv::decomposeHomographyMat(H,K,r,t,n);
    		
	// // auto ratio_K = K.clone();
	// // ratio_K.at<double>(1,1) *= y_factor;
	// cv::decomposeHomographyMat(H,ratio_K,r,t,n);

    for(int i=0;i<n.size(); i++){
        cv::Mat Rcw = r[i].inv();
        cv::Mat tcw = -Rcw*t[i];
        cv::Mat Rvec;
        cv::Rodrigues(Rcw,Rvec);
        // std::cout<<"Sol_"<<i<<std::endl;
        // std::cout<<"\tHomo_tcw:"<<tcw.t()<<std::endl;
        // std::cout<<"\tHomo_pyr_cw: "<<Rvec.t()*DegperRad<<std::endl<<std::endl;
        decomposedH_stream[i]<<tcw.t()<<','<<Rvec.t()*DegperRad<<std::endl;
    }
}


