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

	
	// // Modeling
	cv::Mat Sign = cv::imread("../F13.png", cv::IMREAD_COLOR);
	cv::Mat Scaled_Sign;
	cv::resize(Sign, Scaled_Sign, cv::Size(Sign.cols * 4, Sign.rows * 4)); //1 pixel = 1 cm 
	std::cout << "Scaled_Sign.size(): " << Scaled_Sign.size() <<" cmxcm"<<std::endl; //cm
	
	double Road_width = 3.5; //m
	double TSR_depth = 80; // m
	double TSR_height = -4; // m
	double camera_height = -1; // m
	

	Eigen::Matrix3f camera_pyr_config = Eigen::AngleAxisf(-10*RadperDeg,Eigen::Vector3f(1,0,0)).toRotationMatrix();
	VISUALIZER::Camera_Viewer viewer(Intrinsics, VISUALIZER::Pose(camera_pyr_config, Eigen::Vector3f(-Road_width*1.5, camera_height, 0)));
	std::vector<VISUALIZER::Obj*> vecObj_ptr; 
	
	// Add TS_Rect
	vecObj_ptr.push_back(new VISUALIZER::TS_Rect(Scaled_Sign, TSR_height, TSR_depth));
	auto ptr_TS_Rect = dynamic_cast<VISUALIZER::TS_Rect*>(vecObj_ptr.front());
	ptr_TS_Rect->cal_rect_ref(viewer.getImg().size());
	std::vector<cv::Point2f> ref_vertex(ptr_TS_Rect->rect_ref.ptArr.get(),ptr_TS_Rect->rect_ref.ptArr.get()+4);
	std::vector<cv::Point3f> TS_3d; //use for pnp verification
	for(int i =0; i<4; i++)
		TS_3d.push_back(EigenVecTocvPt(ptr_TS_Rect->Conrner_3D[i]));

	
	double path_length = 100; // m
	for(int i =0; i<4; i++){
		VISUALIZER::Pose LANE_pose(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-Road_width*i, 0, 0));
		VISUALIZER::Obj *Obj_ptr = new VISUALIZER::LANE(path_length, YELLOW, cv::Point(), LANE_pose);
		vecObj_ptr.push_back(Obj_ptr);
	} // end of LANE creation and adding

	std::vector<VISUALIZER::GBRxyzPt> path;
	path.reserve(path_length*mTOcm);
	for(int i=0; i<path_length*mTOcm; i++){//cm
		double x;
		if(i>=200){
			x = SHAPE::tan_path(i,Road_width*mTOcm,30*mTOcm);
		} 
		VISUALIZER::GBRxyzPt Pt(Eigen::Vector3f(x*cmTOm,0,i*cmTOm),RED);
		path.push_back(std::move(Pt));
	} // end of path generation
	vecObj_ptr.push_back(new VISUALIZER::LANE(path,cv::Point(100,40),  VISUALIZER::Pose(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-Road_width*1.5, 0, 0))));

	std::vector<VISUALIZER::Pose> camera_pose;
	double speed = 120; // km per hr
	int FPS = 30;
	double speed_perframe = (speed*1000/3600)/FPS;
	auto ref_xyz = vecObj_ptr.back()->get()[0].xyz; //set referenced origin
	auto camera_height_offset = Eigen::Vector3f(0,camera_height,0);
	camera_pose.push_back(VISUALIZER::Pose(camera_pyr_config,ref_xyz+camera_height_offset));
	int count = 0;
	for(auto& path_pt: vecObj_ptr.back()->get()){
		if(path_pt.xyz.z() > TSR_depth-8)
			break;
		if(distance(path_pt.xyz,ref_xyz)>speed_perframe){
			camera_pose.push_back(VISUALIZER::Pose(camera_pyr_config,path_pt.xyz+camera_height_offset));
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
	std::cout<<"Given path:"<<distance(camera_pose.front().twc,camera_pose.back().twc)<<"m with speed:"<<speed<<"km/hr, it takes "<<camera_pose.size()<<" frames ("<<static_cast<float>(camera_pose.size())/FPS<<" sec)"<<std::endl;


	//main loop
	auto K = EigenTocvMat(Intrinsics);
	pnp(TS_3d,ref_vertex,K);
	for(auto& pose: camera_pose){
		viewer.setPose(pose);

		//project all 3D objects
		for(auto& Obj_ptr: vecObj_ptr) 
			viewer.projectToImg(Obj_ptr->get());
		cv::Mat cur_view = viewer.getImg();
		ptr_TS_Rect->cal_rect_proj(std::bind(&VISUALIZER::Camera_Viewer::projection,&viewer,std::placeholders::_1));
		for(int i=0; i<4; i++)
			cv::circle(cur_view, ptr_TS_Rect->rect_proj.ptArr.get()[i],2,GREEN,-1);

		std::vector<cv::Point2f> proj_vertex(ptr_TS_Rect->rect_proj.ptArr.get(),ptr_TS_Rect->rect_proj.ptArr.get()+4);
		pnp(TS_3d,proj_vertex,K); //pnp verification
		Homography(ref_vertex,proj_vertex,K);

		cv::imshow("viewer",cur_view);
		cv::waitKey(1);
	}
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

