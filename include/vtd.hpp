#ifndef VTD_H
#define VTD_H

#include "opencv2/core/core.hpp"
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <visualizer.h>
#include <fstream>
#include "utility.h"

Eigen::MatrixXf Polynomial_regression(const int order, const std::vector<float>& vec_x, const std::vector<float>& vec_y){
    const int data_num = vec_y.size();
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(data_num,1);
    for(int row = 0; row<data_num; row++)
        b(row,0) = vec_y[row];
    
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(data_num,order+1);
    for(int row = 0; row<data_num; row++){
        float x = vec_x[row];
        for(int col = 0; col<order+1; col++){
            A(row,col) = pow(x,col);
        }
    }

    // // SVD
    // auto svd_sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    // std::cout<<"Estimated coeff(SVD):\n\t";
	// std::cout<<svd_sol.transpose()<<std::endl;

    // normal equation
    return (A.transpose() * A).ldlt().solve(A.transpose() * b);
}



class vtd_roadmark_Reader{
public:
    ~vtd_roadmark_Reader(){read_stream.close();}
    void init();
    bool read_roadmark();
    std::vector<VISUALIZER::LANE>& get_roadmark();
protected:
    bool seperate_data();
private:
    std::fstream read_stream;
    std::vector<VISUALIZER::LANE> vec_LANE;
    std::string line;
    double simTime;
    uint simFrame;
};





#endif // VTD_H
