#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <chrono>
#include <memory>
#include "boost/any.hpp"
#include <boost/algorithm/string.hpp>


#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "staple_tracker.hpp"



#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cvEigenConverter.h"

#define CODE_POS "...@"<<__func__<<" in "<<__FILE__ <<":"<< __LINE__<<"\n"
#define LOG_MSG(msg) {std::cerr<<"[Log]: "<<msg<<CODE_POS;}
#define ERROR_MSG(msg) {std::cerr<<"[Error]: "<<msg<<CODE_POS;}
#define WARNING_MSG(msg) {std::cerr<<"[WARNING]: "<<msg<<CODE_POS;}                                                    
#define ERROR_MSG_EXIT(msg) {std::cerr<<"[Error]: "<<msg<<CODE_POS; exit(1);}
#define WARNING_MSG_EXIT(msg) {std::cerr<<"[WARNING]: "<<msg<<CODE_POS; exit(1);}                                                    
#define FORBIDDEN_FUNC ERROR_MSG_EXIT("Using forbidden function")
#define OFFLINE_ONLY ERROR_MSG_EXIT("For OFFLINE mode only")
#define ONVEHICLE_ONLY ERROR_MSG_EXIT("For ONVEHICLE mode only")


const std::string BTSD_root_path = "/home/leon/GallopWave/TSR_Dataset/BelgiumTSD/";
const std::string BTSD_sample_img_path = BTSD_root_path+"DefinedTS/img/";

#define RED cv::Vec3b(0,0,255)
#define BLUE cv::Vec3b(255, 0, 0)
#define GREEN cv::Vec3b(0,255,0)
#define YELLOW cv::Vec3b(0,255,255)
#define WHITE cv::Vec3b(255, 255, 255)

const float DegperRad = 180.0/M_PI;
const float RadperDeg = M_PI/180.0;
const float cmTOm = 1.0/100;
const float mTOcm = 100;

class TSR{
public:
    TSR(std::string& _type, cv::Rect& _bbox, unsigned int _frame, cv::Point3d& _pos):
        type(_type), bbox(_bbox), frame(_frame), pos(_pos){
            std::string img_name = BTSD_sample_img_path+type+".png";
            sample_img = cv::imread(img_name.c_str(), cv::IMREAD_COLOR);
        }
    friend std::ostream& operator<<(std::ostream& os, const TSR& rhs);
	std::string seqName;
	std::string type;
	cv::Rect bbox;
	unsigned int frame;
	cv::Mat sample_img;
    cv::Point3d pos;
};

inline std::ostream& operator<<(std::ostream& os, const TSR& rhs){
    os<<rhs.frame<<','<<rhs.type<<','<<rhs.bbox<<','<<rhs.pos;
    return os;
}

class BTSD_GT{
	std::string seqName;
	int number_pole;
};

std::vector<std::string> getline_and_prasingstr(std::fstream& fs, const std::string& delim = " ;");
void crop_video(const std::string& raw_video, const std::string& croped_video, int start_sec, int lasting_sec);
void on_mouse(int EVENT, int x, int y, int flags, void* ustc);
void Label_GW_TSR_pt(const std::string& video_path);
void Tracking_Test(const std::string& video_path);
void Corner_Test(const std::string& video_path);
cv::Rect scaling_Rect(cv::Rect rect, double factor);


void labeled_video(const std::string& video_path);
void pnp(std::vector<cv::Point3f>& Pt_3D, std::vector<cv::Point2f>& projected_pt, cv::Mat K);
void Homography(std::vector<cv::Point2f>& ref_vertex,std::vector<cv::Point2f>& proj_vertex, cv::Mat K);
double calculate_time(const std::chrono::steady_clock::time_point& _start);

#endif 














