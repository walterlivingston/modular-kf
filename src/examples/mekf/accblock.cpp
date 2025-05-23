#include "examples/mekf/accblock.h"
#include "quaternions.h"

using namespace mekf;

AccMeasurementBlock::AccMeasurementBlock(vecX& meas_sigmas)
    : mkf::MeasurementBlockBase(meas_sigmas){

}

AccMeasurementBlock::~AccMeasurementBlock(){

}

matX AccMeasurementBlock::updateObservationMatrix(const vecX& x, const vecX& y){
    vecX X = this->_aux.getNominalState();
    matX toRfromB = q2DCM(X({0,1,2,3}));
    matX toBfromR = toRfromB.transpose();
    vecX g;
    g << 0, 0, -9.81;

    matX O = Eigen::MatrixXd::Zero(3,3);
    matX I = Eigen::MatrixXd::Identity(3,3);
    matX H(9,6);
    H << I, O, I,
         O, skew(toBfromR*g), O;
    this->_H = H;
    return H;
}

matX AccMeasurementBlock::calcMeasurementCovariance(){
    vecX variances = this->_meas_sigmas.array()*this->_meas_sigmas.array();
    matX R = variances.diagonal();
    return R;
}

vecX AccMeasurementBlock::calcMeasurementEstimate(const vecX& x, const vecX& y){
    vecX X = this->_aux.getNominalState();
    matX toRfromB = q2DCM(X({0,1,2,3}));
    matX toBfromR = toRfromB.transpose();
    vecX g;
    g << 0, 0, -9.81;

    vecX yhat = toBfromR*g;
    return yhat;
}

mkf::EstWithNominal AccMeasurementBlock::applyError(vecX& x, vecX& X){
    mkf::EstWithNominal ewn;
    ewn.X(X.size());
    ewn.x = Eigen::VectorXd::Zero(x.size());

    vec3 alpha = 0.5*x({3,4,5});
    vecQ qa;
    qa << 1,alpha;
    qa = qNormalize(qa);
    vecX X_ = X;
    ewn.X({0,1,2,3}) = qMult(X_({0,1,2,3}), qa);
    ewn.X({4,5,6}) = X_({4,5,6});

    ewn.x({0,1,2}) = x({0,1,2});
    ewn.x({6,7,8}) = x({6,7,8});

    return ewn;
}