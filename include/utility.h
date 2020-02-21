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

#include <Eigen/Core>
#include <Eigen/Geometry>

#define BLUE cv::Scalar(255,0,0)
#define RED cv::Scalar(0,0,255)
#define GREEN cv::Scalar(0,255,0)


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
    // if(!rhs.sample_img.empty()){
    //     cv::imshow("Sample Img", rhs.sample_img);
    //     cv::waitKey(100);
    // }
    return os;
}/*-------------------IMUDATA-------------------*/

class BTSD_GT{
	std::string seqName;
	int number_pole;

};

template<int NUM_PT>
class npt_shape{
    public:
        npt_shape(){
            num_pt = NUM_PT+1;
            ptArr = std::shared_ptr<cv::Point>(new cv::Point[NUM_PT+1](), std::default_delete<cv::Point[]>());
        }
        void cal_center(){
            for(int i=0; i<num_pt-1; i++)
                ptArr.get()[num_pt-1] += ptArr.get()[i];
            ptArr.get()[num_pt-1] /= (num_pt-1);
        }
    // private:
        //start with center pt then go through from the most upper-left pt in clock-wise direction
        std::shared_ptr<cv::Point> ptArr;
        int num_pt;
};

using RECT = npt_shape<4>;


class Camera_Motion{
    public:
        Camera_Motion(){
            R = Eigen::Matrix3d::Identity();
            T = Eigen::Vector3d::Zero();
        }
        explicit Camera_Motion(const Eigen::Matrix3d& _R, const Eigen::Vector3d& _T):R(_R),T(_T){}
        static void setK(Eigen::Matrix3d _K){K=_K;}
        static Eigen::Matrix3d getK(){return K;}
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        static Eigen::Matrix3d K;
};


std::vector<std::string> getline_and_prasingstr(std::fstream& fs, const std::string& delim = " ;");
void crop_video(const std::string& raw_video, const std::string& croped_video, int start_sec, int lasting_sec);
void on_mouse(int EVENT, int x, int y, int flags, void* ustc);
void Label_GW_TSR_pt(const std::string& video_path);
void TEST_HOMO(const std::string& video_path);
std::vector<cv::Point> arr2vec(cv::Point* arr, int num);
cv::Point2f operator*(cv::Mat M, const cv::Point2f& p);
cv::Mat Matrix3dtoCvMat(const Eigen::Matrix3d &m);
Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat& cvMat3);


// Image center (assume cm )is origin of world coordinate
Eigen::Vector3d Pixe2DtoPt3D(double pixel_x, double pixel_y, cv::Size img_size, double height, double depth);
double degTorad(double deg);
double radTodeg(double rad);
cv::Point pinhole_inv(int x, int y,cv::Size size);
bool out_of_Img(const cv::Point& pt, cv::Size size);
void pose_recording(std::fstream& os, cv::Mat& R, cv::Mat& T);
void getAnglesformR(cv::Mat R, double &angleX, double &angleY, double &angleZ);
double trajectory_tan(double heading_dis, cv::Point2d origin, double width, double length, const std::string& Turn_Dir = "R");
void MapToCamera(std::vector<Eigen::Vector3d> pts, const Camera_Motion& state,cv::Mat stimulated_img, const cv::Vec3b& color,const cv::Point& interval = cv::Point(0,0));
double dis_Vector3d(const Eigen::Vector3d& lhs,const Eigen::Vector3d& rhs);




#endif 














