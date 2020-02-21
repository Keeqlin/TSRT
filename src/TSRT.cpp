#include "utility.h"
#include "cvEigenConverter.h"
#include <limits>

// using namespace std;
using namespace cv;


extern const std::string BTSD_root_path;
extern const std::string BTSD_sample_img_path;



std::vector<Mat> read_BTSD_Seq(const std::string& path, int index_start, const std::string& zero_num, const std::string& img_format, const std::string& prefix = "");
std::vector<TSR> read_BTSD_GT(const std::string& ground_truth_path, unsigned int camera_index);

int main(int argc, char** argv){
	// //load BTSD dataset
	// std::string path = "Seq03/01/";
	// int index_start = 22000;
	// read_BTSD_Seq(BTSD_root_path+path,index_start,"6","png","image.");

	// split raw video
	// crop_video("./../recordvepp.mp4", "./../TSR1.mp4", 0, 3);
	// crop_video("./../recordvepp.mp4", "./../TSR2.mp4", 7, 4);
	// crop_video("./../recordvepp.mp4", "./../TSR3.mp4", 26, 5);
	// crop_video("./../recordvepp.mp4", "./../TSR4.mp4", 51, 9);
	// crop_video("./../recordvepp.mp4", "./../TSR3.mp4", 576, 4);

	// Label TSR pt
	// Label_GW_TSR_pt("./../TSR1.mp4");
	// Label_GW_TSR_pt("./../TSR2.mp4");
	// Label_GW_TSR_pt("./../TSR4.mp4");


	//Homography Test
	// TEST_HOMO("./../TSR1.mp4");
	// TEST_HOMO("./../TSR4.mp4");


	// // preview clip
	// cv::VideoCapture cap("./../TSR2.mp4"); 
    // if(!cap.isOpened())
    //     WARNING_MSG_EXIT("Error opening video..");
	
	// cv::Mat Img;
	// while(cap.read(Img)){
	// 	cv::imshow("preview", Img);
	// 	cv::waitKey(cap.get(CV_CAP_PROP_FPS));
	// }

	auto Y = Matx33d{1.34,2.123,3.45,4.0,5.123,6.234,7.213,8.62,9.36};
	auto X = cvToEigenMat(Y);
	std::cout<<"Y: \n"<<Y<<std::endl;
	std::cout<<"X: \n"<<X<<std::endl;
	auto Z = EigenTocvMat(X);
	std::cout<<"Z: \n"<<Z<<std::endl;
	auto pt3 = cvPtToEigenVec(cv::Point3f(3234.234,45.123,234.4354));
	std::cout<<"pt3: \n"<<pt3<<std::endl;

	auto XXX = EigenVecTocvPt(Eigen::Vector2d(1.456,2.213));
	std::cout<<"XXX: \n"<<XXX<<std::endl;
	


	// // use for recording pose estimation of decomposedH
	// std::vector<std::fstream> pose_stream;
	// for(int i=0; i<4; i++){
	// 	std::string name = "decompose_H_pose";
	// 	std::fstream os(std::string(name+std::to_string(i)+".txt").c_str(), std::ios::out | std::ios::trunc);
	// 	pose_stream.push_back(std::move(os));
	// }

	
	// // SLAM 14 courses: pp.100 
	// // camera configuration (first pirority)
	double cx = 300; 
	double cy = 250;
	double fx = 518;
	double fy = 518;

	Eigen::Matrix3d Intrinsics = Eigen::Matrix3d::Zero(); //pixel
	Intrinsics(0,0) = fx, 
	Intrinsics(0,2) = cx; 
	Intrinsics(1,1) = fy, 
	Intrinsics(1,2) = cy; 
	Intrinsics(2,2) = 1;
	// std::cout<<Intrinsics<<std::endl;
	// Camera_Motion::setK(Intrinsics); //set Intrinisics matrix
	// cv::Mat K =  Matrix3dtoCvMat(Intrinsics);
	// cv::Mat stimulated_img = cv::Mat::zeros(cv::Size(cx*2,cy*2),CV_8UC3);


	// // Modeling
	// cv::Mat Sign = cv::imread("../F13.png",cv::IMREAD_COLOR);
	// cv::Mat scaled_Sign;
	// cv::resize(Sign,scaled_Sign,cv::Size(Sign.cols*4,Sign.rows*4));
	// std::cout<<"scaled_Sign.size(): "<<scaled_Sign.size()<<std::endl; //cm
	// std::vector<cv::Point> ref_scaled_Sign_pt,scaled_Sign_pt;
	// double ratio = (stimulated_img.cols>stimulated_img.rows)?static_cast<double>(stimulated_img.rows)/static_cast<double>(scaled_Sign.rows):static_cast<double>(stimulated_img.cols)/static_cast<double>(scaled_Sign.cols);
	// std::cout<<"ratio: "<<ratio<<std::endl;
	// cv::Point stimulated_img_center = cv::Point(cx,cy);
	// cv::Point scaled_Sign_center = cv::Point(scaled_Sign.cols/2,scaled_Sign.rows/2);
	// scaled_Sign_pt.push_back(cv::Point(0,0));
	// scaled_Sign_pt.push_back(cv::Point(scaled_Sign.cols-1,0));
	// scaled_Sign_pt.push_back(cv::Point(scaled_Sign.cols-1,scaled_Sign.rows-1));
	// scaled_Sign_pt.push_back(cv::Point(0,scaled_Sign.rows-1));
	// for(auto pt: scaled_Sign_pt){
	// 	ref_scaled_Sign_pt.push_back(stimulated_img_center+ ratio*(pt - scaled_Sign_center));
	// 	std::cout<<"ref_scaled_Sign_pt: "<<ref_scaled_Sign_pt.back()<<std::endl;
	// }
	// std::cout<<std::endl;
	
	// // double fix_length = scaled_Sign.cols;
	// // ref_scaled_Sign_pt.push_back(cv::Point(0,0));
	// // ref_scaled_Sign_pt.push_back(cv::Point(fix_length-1,0));
	// // ref_scaled_Sign_pt.push_back(cv::Point(fix_length-1,fix_length-1));
	// // ref_scaled_Sign_pt.push_back(cv::Point(0,fix_length-1));
	// cv::Point center(0,0);
	// for(auto& pt: ref_scaled_Sign_pt)
	// 	center+=pt;
	// center/=4;
	// // ref_scaled_Sign_pt.push_back(center);
	// double y_factor = 1; 
	// // y_factor = static_cast<double>(scaled_Sign.cols)/static_cast<double>(scaled_Sign.rows); 


	// double Road_width = 3.5; //m
	// double TSR_depth = 80; // m
	// double TSR_height = -3; // m
	// double camera_height = -1; // m
	// cv::Point3d ini_camera(-Road_width*3.5,camera_height,0); //m
	// std::vector<RECT> mapped_Sign; //store mapped pt of TSR

	// //object creation
	// std::vector<Eigen::Vector3d> Horizontal_line;
	// int len_Horizontal_line = 100; // m 
	// Horizontal_line.reserve(len_Horizontal_line*2*100+1);
	// for(int i = -len_Horizontal_line*100; i<= len_Horizontal_line*100; i+=1) 
	// 	Horizontal_line.push_back(Eigen::Vector3d(static_cast<double>(i)/100,0,3000));
		
	// std::vector<Eigen::Vector3d> TSR_Bar;
	// int width_TSR_Bar = 5; // cm 
	// TSR_Bar.reserve(width_TSR_Bar*2*100+1);
	// for(int i = -width_TSR_Bar/2; i<= width_TSR_Bar/2; i++)
	// 	for(int j = TSR_height*100; j<=0; j++ )
	// 	TSR_Bar.push_back(Eigen::Vector3d(static_cast<double>(i)/100,static_cast<double>(j)/100,TSR_depth));


	// std::vector<std::vector<Eigen::Vector3d>> Lanes(5); //form right to left
	// int len_Lane_Boundary = 100; // m 
	// for(auto& Lane: Lanes)
	// 	Lane.reserve(len_Lane_Boundary*100);
	// for(int i = 0; i< len_Lane_Boundary*100; i+=1){
	// 	for(int lane_index = 0; lane_index< Lanes.size(); lane_index++)
	// 		Lanes[lane_index].push_back(Eigen::Vector3d(-Road_width*lane_index,0,static_cast<double>(i)/100));
	// }
		

	// // std::vector<Eigen::Vector3d> mapPath;
	// // double tan_len = 20; // m 
	// // mapPath.reserve(tan_len*100);
	// // for(int i = 0; i< TSR_depth*100; i+=1){ // i:cm
	// // 	double x; 
	// // 	if(i<=4000)
	// // 		x = trajectory_tan(static_cast<double>(i)/100,cv::Point2d(ini_camera.x,20),std::abs(Road_width),tan_len);
	// // 	else
	// // 		x = trajectory_tan(static_cast<double>(i)/100,cv::Point2d(mapPath[4000].x(),45),std::abs(Road_width),tan_len);
	// // 	mapPath.push_back(Eigen::Vector3d(x,0,static_cast<double>(i)/100));
	// // }

	// std::vector<Eigen::Vector3d> mapPath;
	// double tan_len = 30; // m 
	// mapPath.reserve(tan_len*100);
	// for(int i = 0; i< (TSR_depth-15)*100; i+=1){ // i:cm
	// 	double x; 
	// 		x = trajectory_tan(static_cast<double>(i)/100,cv::Point2d(ini_camera.x,20),std::abs(Road_width),tan_len);
	// 	mapPath.push_back(Eigen::Vector3d(x,0,static_cast<double>(i)/100));
	// }


	// std::vector<Camera_Motion> camera_pose;
	// double speed = 120; //km/hr
	// int FPS = 30;
	// double speed_perframe = (speed*1000/3600)/FPS;


	// // int step = (TSR_depth-12)/speed_perframe;
	// // std::cout<< (TSR_depth-12)<<"m for "<<speed<<" km/hr, takes "<<step<<" frames ("<<step/30<<" sec)"<<std::endl;
	// // std::vector<double> tmp{0,-1,-2,-3,-4,-5,-4,-3,-2,-1,0,0,0,0,0};
	// // std::vector<double> car_yaw;
	// // for(int i = 0; i<4; i++)
	// // 	car_yaw.insert(car_yaw.end(),tmp.begin(),tmp.end());

	// // for(int i = 0; i<step; i++){
	// // 	Camera_Motion state(Eigen::AngleAxisd(degTorad(car_yaw[i]),Eigen::Vector3d(0,1,0)).matrix(),Eigen::Vector3d(ini_camera.x,ini_camera.y,i));
	// // 	camera_pose.push_back(state);
	// // }

	// auto ref_pt = mapPath[0];
	// camera_pose.push_back(Camera_Motion(Eigen::Matrix3d::Identity(),mapPath[0]+Eigen::Vector3d(0,camera_height,0)));
	// int count = 0;
	// for(auto& path_pt: mapPath){
	// 	if(path_pt.z() > TSR_depth-8)
	// 		break;
	// 	if(dis_Vector3d(path_pt,ref_pt)>speed_perframe){
	// 		camera_pose.push_back(Camera_Motion(Eigen::Matrix3d::Identity(),path_pt+Eigen::Vector3d(0,camera_height,0)));
	// 		ref_pt = path_pt;
	// 	}
	// 	count++;
	// }
	// for(int i = 0; i<camera_pose.size()-1;i++){
	// 	auto cur = camera_pose[i].T;
	// 	auto next = camera_pose[i+1].T;
	// 	auto rad = std::atan2(next.x()-cur.x(),next.z()-cur.z());
	// 	auto R = Eigen::AngleAxisd(rad,Eigen::Vector3d(0,1,0)).matrix();
	// 	camera_pose[i].R = R;
	// }
	// camera_pose[camera_pose.size()-1].R = camera_pose[camera_pose.size()-2].R; 
	// std::cout<<"Frames: "<<camera_pose.size()<<std::endl;


	// for(auto& state:camera_pose){
	// 	RECT rect; 
	// 	stimulated_img = cv::Mat::zeros(cv::Size(stimulated_img.cols,stimulated_img.rows),CV_8UC3);
		
	// 	//project sign
	// 	Eigen::Vector3d pcam1,pcam2,pcam3;
	// 	for(int y = 0; y < scaled_Sign.rows; y++)
	// 		for(int x = 0; x < scaled_Sign.cols; x++){
	// 			Eigen::Vector3d pt_world = Pixe2DtoPt3D(x,y,scaled_Sign.size(),TSR_height,TSR_depth);
	// 			Eigen::Vector3d pt_camera = state.R.inverse()*(pt_world-state.T); //p_world to p_camera

	// 			if(y==0 && x==0)
	// 				pcam1 = pt_camera;
	// 			if(y==scaled_Sign.rows-1 && x==scaled_Sign.cols-1)
	// 				pcam2 = pt_camera;
	// 			if(y==scaled_Sign.rows-1 && x==0)
	// 				pcam3 = pt_camera;


	// 			pt_camera /= pt_camera.z(); // normalized coordination: z = 1, in front of center of camera
	// 			Eigen::Vector3d mapped_Pt = Intrinsics*(pt_camera);
	// 			cv::Point cv_mapped_Pt(mapped_Pt.x() ,mapped_Pt.y());
	// 			auto scaled_Intrinsics = Intrinsics;
	// 			scaled_Intrinsics(1,1) *= y_factor;
	// 			Eigen::Vector3d scaled_mapped_Pt = scaled_Intrinsics*(pt_camera);
	// 			cv::Point scaled_cv_mapped_Pt(scaled_mapped_Pt.x() ,scaled_mapped_Pt.y());

	// 			if(y==0 && x==0)
	// 				rect.ptArr.get()[0] = scaled_cv_mapped_Pt;
	// 			if(y==0 && x==scaled_Sign.cols-1)
	// 				rect.ptArr.get()[1] = scaled_cv_mapped_Pt;
	// 			if(y==scaled_Sign.rows-1 && x==scaled_Sign.cols-1)
	// 				rect.ptArr.get()[2] = scaled_cv_mapped_Pt;
	// 			if(y==scaled_Sign.rows-1 && x==0)
	// 				rect.ptArr.get()[3] = scaled_cv_mapped_Pt;

	// 			if(!out_of_Img(cv_mapped_Pt,stimulated_img.size()))
	// 				stimulated_img.at<cv::Vec3b>(cv_mapped_Pt) = scaled_Sign.at<cv::Vec3b>(y,x);
	// 	}
	// 	rect.cal_center();
	// 	mapped_Sign.push_back(rect);
	// 	// auto v = pcam1 - pcam2;
	// 	// auto u = pcam1 - pcam3;
	// 	// auto nor_vec = v.cross(u);
	// 	// std::cout<<"comput n: "<<nor_vec.transpose()/std::sqrt( std::pow(nor_vec.x(),2)+std::pow(nor_vec.y(),2)+std::pow(nor_vec.z(),2))<<std::endl;

	// 	//Projecting 
	// 	MapToCamera(TSR_Bar, state, stimulated_img, cv::Vec3b(255,255,255));
	// 	MapToCamera(Horizontal_line, state, stimulated_img, cv::Vec3b(0,255,0));
	// 	for(auto& Lane: Lanes)
	// 		MapToCamera(Lane, state, stimulated_img, cv::Vec3b(0,255,255));
	// 	MapToCamera(mapPath, state, stimulated_img, cv::Vec3b(0,0,255),cv::Point(200,60));
		
	// 	//Draw center of stimulated_img
	// 	cv::circle(stimulated_img, cv::Point(stimulated_img.cols/2,stimulated_img.rows/2),3,RED,-1);

	// 	std::vector<cv::Mat> r,t,n;
	// 	Mat H = cv::findHomography(ref_scaled_Sign_pt,std::vector<cv::Point>(rect.ptArr.get(),rect.ptArr.get()+(rect.num_pt-1)));
	// 	auto ratio_K = K.clone();
	// 	ratio_K.at<double>(1,1) *= y_factor; 
	// 	cv::decomposeHomographyMat(H,K,r,t,n);
	// 	// cv::decomposeHomographyMat(H,ratio_K,r,t,n);

	// 	for(int i=0; i<r.size(); ++i){
	// 		pose_recording(pose_stream[i],r.at(i),t.at(i));
	// 		std::cout<<"n("<<i<<")"<<n.at(i).t()<<std::endl;
	// 	}
	// 	std::cout<<std::endl<<std::endl;

	// 	cv::Mat reprojection = cv::Mat::zeros(cv::Size(stimulated_img.cols,stimulated_img.rows),CV_8UC3);
	// 	warpPerspective(scaled_Sign, reprojection, H, reprojection.size());
	// 	cv::imshow("Stimulated_img",stimulated_img);
	// 	cv::imshow("reprojection",reprojection);		
	// 	// cv::waitKey(0);
	// 	cv::waitKey(1);
	// }


// 	// pnp verification
// 	std::vector<cv::Point3f> TSR_3d;
// 	for(auto& pt_2D:scaled_Sign_pt){
// 			auto pt3D = Pixe2DtoPt3D(pt_2D.x,pt_2D.y,scaled_Sign.size(),TSR_height,TSR_depth);
// 			TSR_3d.push_back(cv::Point3f(pt3D.x(),pt3D.y(),pt3D.z()));
// 			std::cout<<"TSR_3d: "<<TSR_3d.back()<<std::endl;
// 	}
// 	std::vector<cv::Point3f> m_marker3d;
// 	std::vector<cv::Point2f> ref_scaled_Sign_fpt;
// 	for(auto& pt:TSR_3d)
// 		m_marker3d.push_back(cv::Point3f(pt.x,pt.y,pt.z));
// 	for(auto& pt: ref_scaled_Sign_pt)
// 		ref_scaled_Sign_fpt.push_back(cv::Point2f(pt.x,pt.y));

// 	cv::Mat Rvec;
//     cv::Mat_<float> Tvec;
//     cv::Mat raux, taux;
//     cv::Mat_<float> rotMat(3, 3);
// 	cv::solvePnP( TSR_3d, ref_scaled_Sign_fpt , K, cv::noArray(), raux, taux);
// 	raux.convertTo(Rvec, CV_32F);    //旋转向量
//     taux.convertTo(Tvec, CV_32F);   //平移向量
//     cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵

//     //格式转换
//     auto R_n = toMatrix3d(rotMat);
//     Eigen::Vector3d T_n(Tvec.at<float>(0,0),Tvec.at<float>(0,1),Tvec.at<float>(0,2));
//     Eigen::Vector3d P_oc =  -R_n.inverse()*T_n;
// 	std::cout<<"P_oc = "<<P_oc.transpose()<<std::endl;


// // Test homography
// 	// set transformation from 1st to 2nd camera (assume Test_K is unchanged)
//   	cv::Mat1f rvecDeg = (cv::Mat1f(3, 1) << 0, 3, 0);
//   	cv::Mat1f t = (cv::Mat1f(3, 1) << 5, 5, -50);
//   	std::cout << "-------------------------------------------\n";
//   	std::cout << "Ground truth:\n";
//   	std::cout << "K = \n" << K << std::endl << std::endl;
//   	std::cout << "rvec = \n" << rvecDeg.t() << std::endl << std::endl;
//   	std::cout << "t = \n" << t.t() << std::endl << std::endl;

//   	// project on both cameras
//   	std::vector<cv::Point2f> Q, P, S;
//   	cv::Mat1f oriPose = (cv::Mat1f(3, 1) << 0, 3, -77);
//   	std::cout<<"oriPose: "<<oriPose<<std::endl;
//   	cv::projectPoints(TSR_3d,
//                       cv::Mat1f::zeros(3, 1),
// 					  oriPose,
//                       K,
//                       cv::Mat(),
//                       Q);
// 	std::cout<<"Projected pts(Q):"<<std::endl;
// 	for(auto& pt: Q)
// 		std::cout<<"\t"<<pt<<", ";
//   	std::cout<<std::endl;
//   std::cout<<"projected(w,h): "<<(Q[2]-Q[0])<<std::endl;


//   cv::projectPoints(TSR_3d,
//                     rvecDeg*CV_PI/180,
//                     t,
//                     K,
//                     cv::Mat(),
//                     P);
//   std::cout<<"Projected pts(P):"<<std::endl;
//   for(auto& pt: P)
// 	std::cout<<"\t"<<pt<<", ";
//   std::cout<<std::endl;

//   // find homography
//   cv::Mat H = cv::findHomography(Q, P);
//   std::cout << "-------------------------------------------\n";
//   std::cout << "Estimated H = \n" << H << std::endl << std::endl;

//   // check by reprojection
//   std::vector<cv::Point2f> P_(P.size());
//   cv::perspectiveTransform(Q, P_, H);
//   double sumError = 0;
//   for (size_t i = 0; i < P.size(); i++) {
//     sumError += cv::norm(P[i] - P_[i]);
//   }

//   std::cout << "-------------------------------------------\n";
//   std::cout << "Average reprojection error = "
//       << sumError/P.size() << std::endl << std::endl;


//   // decompose using identity as internal parameters matrix
//   std::vector<cv::Mat> Rs, Ts;
//   cv::decomposeHomographyMat(H,
//                              K,
//                              Rs, Ts,
//                              cv::noArray());

//   std::cout << "-------------------------------------------\n";
//   std::cout << "Estimated decomposition:\n\n";
//   std::cout << "rvec = " << std::endl;
//   for (auto R_ : Rs) {
//     cv::Mat1d rvec;
//     cv::Rodrigues(R_, rvec);
//     std::cout << rvec*180/CV_PI << std::endl << std::endl;
//   }

//   std::cout << std::endl;

//   std::cout << "t = " << std::endl;
//   for (auto t_ : Ts) {
//     std::cout << t_ << std::endl << std::endl;
//   }
	// Homo Test end

























	

	return 0;
}

std::vector<Mat> read_BTSD_Seq(const std::string& path, int index_start, const std::string& zero_num, const std::string& img_format, const std::string& prefix){
	//load ground truth
	std::string gt_path = BTSD_root_path+"Seqs_poses_annotations/sequence3_GT.txt";
	auto vTSR = read_BTSD_GT(gt_path,1);

	Mat Img;
	std::string imageName;
	std::vector<Mat> vecImg;
    std::string padding_format = "%0"+zero_num+"d";
	
	//Read the image 
	while(true){
		char buff[stoi(zero_num)];
		sprintf(buff, padding_format.c_str() , index_start);
		imageName = path+prefix+std::string(buff)+"."+img_format;
		Img = imread(imageName.c_str(),IMREAD_COLOR);
		
		if(Img.empty()) //end of file
			break;
		// else
		// 	vecImg.push_back(Img);
		bool flag = false;

		for(auto& tsr:vTSR){
			if(tsr.frame == index_start){
				cv::rectangle(Img, tsr.bbox, RED,2);
				std::cout<<tsr<<std::endl;
				flag = true;
			}
		}

		cv::Mat dst;
		cv::resize(Img,dst,cv::Size(Img.cols/2, Img.rows/2));
		imshow(path.c_str(),dst);

		waitKey(50);
		index_start++;
	}

	std::cout<<"Finish loading...\n"
			 <<"Number of Image: "<<vecImg.size()<<std::endl;
	return vecImg;
}

std::vector<TSR> read_BTSD_GT(const std::string& ground_truth_path, unsigned int camera_index){
	const int preInfo_line = 3;
	const int TSR_line  = 4;
	std::vector<std::string> vstr;
	std::vector<TSR> vTSR;

	std::fstream read_stream(ground_truth_path.c_str(),std::ios::in);
	if(!read_stream)
		WARNING_MSG_EXIT("Fail to load BTSD_GT: "+ground_truth_path);
	
	std::string line;
	getline(read_stream,line);
	int num_pole = stoi(line);
	int np = 1; //cur number of pole

	while(np <= num_pole){
		//load preInfo 
		for(int i = 0; i<preInfo_line; i++)
			getline(read_stream,line);

		//load number of centers used for getting the 3D position of the poles
		vstr = getline_and_prasingstr(read_stream);
		int num_of_construct_pole = stoi(vstr[0]);
		for(int i =0; i<num_of_construct_pole;i++)
			getline(read_stream,line);

		//load number of annotated bounding boxes for the traffic signs on this pole
		vstr = getline_and_prasingstr(read_stream);
		int num_abbox = stoi(vstr[0]);		

		for(int i=0; i<num_abbox; i++){
			// one bounding box - 2D coordinates (upleft , downright)
			vstr = getline_and_prasingstr(read_stream);
			Rect bbox(Point(static_cast<int>(stod(vstr[1])),static_cast<int>(stod(vstr[0]))),
					  Point(static_cast<int>(stod(vstr[3])),static_cast<int>(stod(vstr[2]))));

			// camera, frame, order number on the pole for this traffic sign
			vstr = getline_and_prasingstr(read_stream);
			uint8_t camera_ith = stoi(vstr[0]);
			unsigned int frame = stoi(vstr[1]);

			// type of the traffic sign
			vstr = getline_and_prasingstr(read_stream);
			std::string TSR_type = vstr[0];

			// 3D center coordinates for this annotated traffic sign
			vstr = getline_and_prasingstr(read_stream);
			Point3d pos(stod(vstr[0]),stod(vstr[1]),stod(vstr[2]));
			
			if(camera_ith == camera_index){
				TSR* pTSR = new TSR(TSR_type,bbox,frame,pos);
				vTSR.push_back(*pTSR);
			}
		}
		// empty line
		getline(read_stream,line);
		np++;
	}

	return vTSR;
}