#ifndef SHAPE_H
#define SHAPE_H
#include "utility.h"

namespace SHAPE{
inline double tan_path(double heading_dis, double width, double length, const std::string& Turn_Dir = "R"){
    // for tan(x) = y
    // y: +-6
    // x: +-1.405647 rad
    double theta_lim = 1.405647;
    double y_lim = 6;
    double y_scale = length/(2*y_lim);
    double x_scale = width/(2*theta_lim);
    cv::Point2d tan_origin(-theta_lim,-y_lim);
    double var_x;
    
    if(heading_dis <= 0){
        var_x = 0;
    }
    else if(heading_dis > length){
        var_x = width;
    }else{
        double scaled_y = heading_dis/y_scale;
        double tan_theta = atan(scaled_y-y_lim);
        var_x = (tan_theta+theta_lim)*x_scale;
    }
    return var_x;
}


template<int vertex>
class npt_shape{
    public:
        npt_shape(){
            num_pt = vertex+1; //add extra pt which is average of all vertexes
            ptArr = std::shared_ptr<cv::Point2f>(new cv::Point2f[num_pt](), std::default_delete<cv::Point2f[]>());
        }
        void cal_center(){
            for(int i=0; i<vertex; i++)
                ptArr.get()[vertex] += ptArr.get()[i];
            ptArr.get()[vertex] = ptArr.get()[vertex]/4;
        }
        //start with the most upper-left pt in clock-wise direction
        std::shared_ptr<cv::Point2f> ptArr;
        int num_pt;
};
using RECT = npt_shape<4>;

} // namespace SHAPE




#endif // SHAPE_H