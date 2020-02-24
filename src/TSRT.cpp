#include "utility.h"
#include "cvEigenConverter.h"
#include <limits>
#include "visualizer.h"

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



	// use for recording pose estimation of decomposedH
	std::vector<std::fstream> pose_stream;
	for(int i=0; i<4; i++){
		std::string name = "decompose_H_pose";
		std::fstream os(std::string(name+std::to_string(i)+".txt").c_str(), std::ios::out | std::ios::trunc);
		pose_stream.push_back(std::move(os));
	}

	
	// // SLAM 14 courses: pp.100 
	// // camera configuration (first pirority)
	double cx = 300; 
	double cy = 250;
	double fx = 518;
	double fy = 518;

	Eigen::Matrix3f Intrinsics = Eigen::Matrix3f::Zero(); //pixel
	Intrinsics(0,0) = fx, 
	Intrinsics(0,2) = cx; 
	Intrinsics(1,1) = fy, 
	Intrinsics(1,2) = cy; 
	Intrinsics(2,2) = 1;

	// cv::Size Img_size(cx * 2, cy * 2);
	// cv::Mat stimulated_img = cv::Mat::zeros(cv::Size(cx * 2, cy * 2), CV_8UC3);

	
	// // Modeling
	cv::Mat Sign = cv::imread("../F13.png", cv::IMREAD_COLOR);
	cv::Mat Scaled_Sign;
	cv::resize(Sign, Scaled_Sign, cv::Size(Sign.cols * 4, Sign.rows * 4)); //1 pixel = 1 cm 
	std::cout << "Scaled_Sign.size(): " << Scaled_Sign.size() <<" cmxcm"<<std::endl; //cm
	
	double Road_width = 3.5; //m
	double TSR_depth = 80; // m
	double TSR_height = -4; // m
	double camera_height = -1; // m
	
	VISUALIZER::Camera_Viewer viewer(Intrinsics, VISUALIZER::Pose(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-Road_width*1.5, camera_height, 0)));
	std::vector<VISUALIZER::Obj*> vecObj_ptr;
	
	vecObj_ptr.push_back(new VISUALIZER::TS_Rect(Scaled_Sign, TSR_height, TSR_depth)); //add TS_Rect
	for(int i =0; i<4; i++){
		VISUALIZER::Pose LANE_pose(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-Road_width*i, 0, 0));
		VISUALIZER::Obj *Obj_ptr = new VISUALIZER::LANE(100, YELLOW, cv::Point(), LANE_pose);
		vecObj_ptr.push_back(Obj_ptr);
	} // end of LANE creation and push_back

	double path_length = 100; // m
	std::vector<VISUALIZER::GBRxyzPt> path;
	path.reserve(path_length*100);
	for(int i=0; i<path_length*100; i++){//cm
		double x;
		if(i>=200){
			x = SHAPE::tan_path(i,Road_width*100,30*100);
		} 
		VISUALIZER::GBRxyzPt Pt(Eigen::Vector3f(x/100,0,static_cast<float>(i)/100),RED);
		path.push_back(std::move(Pt));
	} // end of math generation
	vecObj_ptr.push_back(new VISUALIZER::LANE(path,cv::Point(100,40),  VISUALIZER::Pose(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-Road_width*1.5, 0, 0))));

	std::vector<VISUALIZER::Pose> camera_pose;
	double speed = 120; // km per hr
	int FPS = 30;
	double speed_perframe = (speed*1000/3600)/FPS;
	auto ref_xyz = vecObj_ptr.back()->get()[0].xyz; //set referenced origin
	auto offset = Eigen::Vector3f(0,camera_height,0);
	camera_pose.push_back(VISUALIZER::Pose(Eigen::Matrix3f::Identity(),ref_xyz+offset));
	int count = 0;
	for(auto& path_pt: vecObj_ptr.back()->get()){
		if(path_pt.xyz.z() > TSR_depth-8)
			break;
		if(distance(path_pt.xyz,ref_xyz)>speed_perframe){
			camera_pose.push_back(VISUALIZER::Pose(Eigen::Matrix3f::Identity(),path_pt.xyz+offset));
			ref_xyz = path_pt.xyz;
		}
		count++;
	}
	// for(int i = 0; i<camera_pose.size()-1;i++){
	// 	auto cur = camera_pose[i].T;
	// 	auto next = camera_pose[i+1].T;
	// 	auto rad = std::atan2(next.x()-cur.x(),next.z()-cur.z());
	// 	auto R = Eigen::AngleAxisd(rad,Eigen::Vector3d(0,1,0)).matrix();
	// 	camera_pose[i].R = R;
	// }
	// camera_pose[camera_pose.size()-1].R = camera_pose[camera_pose.size()-2].R;
	std::cout<<"Given path:"<<distance(camera_pose.front().twc,camera_pose.back().twc)<<"m with speed:"<<speed<<"km/hr, takes "<<camera_pose.size()<<" frames ("<<static_cast<float>(camera_pose.size())/FPS<<" sec)"<<std::endl;


	//main loop
	auto ptr_TS_Rect = dynamic_cast<VISUALIZER::TS_Rect*>(vecObj_ptr.front());
	ptr_TS_Rect->cal_rect_ref(viewer.getImg().size());
	std::vector<cv::Point2f> ref_vertex(ptr_TS_Rect->rect_ref.ptArr.get(),ptr_TS_Rect->rect_ref.ptArr.get()+4);
	for(auto& pose: camera_pose){
		viewer.setPose(pose);

		for(auto& Obj_ptr: vecObj_ptr) //project all 3D objects
			viewer.projectToImg(Obj_ptr->get());
		cv::Mat cur_view = viewer.getImg();
		ptr_TS_Rect->cal_rect_proj(std::bind(&VISUALIZER::Camera_Viewer::projection,&viewer,std::placeholders::_1));
		for(int i=0; i<4; i++)
			cv::circle(cur_view, ptr_TS_Rect->rect_proj.ptArr.get()[i],2,GREEN,-1);

		std::vector<cv::Point2f> proj_vertex(ptr_TS_Rect->rect_proj.ptArr.get(),ptr_TS_Rect->rect_proj.ptArr.get()+4);

		std::vector<cv::Mat> r,t,n;
		cv::Mat H = cv::findHomography(ref_vertex,proj_vertex);
		cv::decomposeHomographyMat(H,EigenTocvMat(Intrinsics),r,t,n);
		
		
		// // auto ratio_K = K.clone();
		// // ratio_K.at<double>(1,1) *= y_factor;
		// cv::decomposeHomographyMat(H,ratio_K,r,t,n);

		for(int i=0; i<r.size(); ++i){
			pose_recording(pose_stream[i],r.at(i),t.at(i));
			std::cout<<"n("<<i<<")"<<n.at(i).t()<<std::endl;
		}


		cv::imshow("TEST",cur_view);
		cv::waitKey(1);
	}
	




		

			// for(auto& state:camera_pose){
			// 	RECT rect;
			// 	stimulated_img = cv::Mat::zeros(cv::Size(stimulated_img.cols,stimulated_img.rows),CV_8UC3);

			// 	//project sign
			// 	Eigen::Vector3d pcam1,pcam2,pcam3;
			// 	for(int y = 0; y < Scaled_Sign.rows; y++)
			// 		for(int x = 0; x < Scaled_Sign.cols; x++){
			// 			Eigen::Vector3d pt_world = Pixe2DtoPt3D(x,y,Scaled_Sign.size(),TSR_height,TSR_depth);
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