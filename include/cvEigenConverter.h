#ifndef CVEIGENCONVERTER_H
#define CVEIGENCONVERTER_H

#include "opencv2/core/core.hpp"
#include <Eigen/Core>



template <typename T>
inline double distance(const Eigen::Matrix<T,3,1>& lhs,const Eigen::Matrix<T,3,1>& rhs = Eigen::Matrix<T,3,1>(0,0,0)){
    double res = std::pow(std::abs(rhs(0,0)-lhs(0,0)),2) + std::pow(std::abs(rhs(1,0)-lhs(1,0)),2) + std::pow(std::abs(rhs(2,0)-lhs(2,0)),2);
    return std::sqrt(res);
}


// inline Eigen::Matrix<float,3,3> toMatrix3f(const cv::Mat& cvMat3){
//     Eigen::Matrix<float,3,3> M;
//     for(int i=0;i<3;i++)
//         for(int j=0; j<3; j++)
//             M(i,j) = cvMat3.at<float>(i,j);
//     return M;
// }

template<typename T, int cols, int rows>
Eigen::Matrix<T, cols, rows> cvToEigenMat(const cv::Matx<T,cols,rows>& cvMat){
    Eigen::Matrix<T,cols,rows> M;
    cv::Mat tmp(cvMat);
    for(int i=0;i<rows;i++){
        T* data = tmp.ptr<T>(i);
        for(int j=0; j<cols; j++){
            M(i,j) = data[j];
        }
    }
    return M;
}

template<typename T, int cols, int rows>
cv::Mat EigenTocvMat(const Eigen::Matrix<T, cols, rows>& Matrix){
    cv::Mat tmp = cv::Mat_<T>::zeros(cols, rows);
    for(int i=0;i<rows;i++){
        T* data = tmp.ptr<T>(i);
        for(int j=0; j<cols; j++){
            data[j] = Matrix(i,j);
        }
    }
    return tmp.clone();
}

template<typename T>
Eigen::Matrix<T,2,1> cvPtToEigenVec(const cv::Point_<T>& pt){
    return Eigen::Matrix<T,2,1>(pt.x,pt.y);
}

template<typename T>
Eigen::Matrix<T,3,1> cvPtToEigenVec(const cv::Point3_<T>& pt){
    return Eigen::Matrix<T,3,1>(pt.x,pt.y,pt.z);
}

template<typename T>
cv::Point_<T> EigenVecTocvPt(const Eigen::Matrix<T,2,1>& pt){
    return cv::Point_<T>(pt(0,0),pt(1,0));
}

template<typename T>
cv::Point3_<T> EigenVecTocvPt(const Eigen::Matrix<T,3,1>& pt){
    return cv::Point3_<T>(pt(0,0),pt(1,0),pt(2,0));
}


#endif // CVEIGENCONVERTER_H
