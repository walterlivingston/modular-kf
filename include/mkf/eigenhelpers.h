#pragma once

#include <Eigen/Dense>

typedef Eigen::MatrixX<double> matX;
typedef Eigen::VectorX<double> vecX;
typedef Eigen::Matrix<double,3,3> mat3;
typedef Eigen::Vector<double,3> vec3;
typedef Eigen::Vector<double,4> vecQ;

mat3 skew(vec3 v){
    mat3 ret;
    ret << 0, -v(2), v(1),
           v(2), 0, -v(0),
           -v(1), v(0), 0;
    return ret;
}