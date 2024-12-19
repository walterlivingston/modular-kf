#include "examples/mekf/magblock.h"
#include "quaternions.h"

using namespace mekf;

MagMeasurementBlock::MagMeasurementBlock(vecX& meas_sigmas)
    : mkf::MeasurementBlockBase(meas_sigmas){

}

MagMeasurementBlock::~MagMeasurementBlock(){

}

matX MagMeasurementBlock::updateObservationMatrix(vecX& x, vecX& y){
    vecX X = this->_aux.getNominalState();
    matX toRfromB = q2DCM(X({0,1,2,3}));
    matX toBfromR = toRfromB.transpose();
    vecX h = toRfromB*y;
    vecX b = {sqrt(h(0)*h(0) + h(1)*h(1)), 0, h(2)};

    matX O = Eigen::MatrixXd::Zero(3,3);
    matX I = Eigen::MatrixXd::Identity(3,3);
    matX H(9,6);
    H << I, O, I,
         O, skew(toBfromR*b), O;
    this->_H = H;
}

matX MagMeasurementBlock::calcMeasurementCovariance(){
    vecX variances = this->_meas_sigmas.array()*this->_meas_sigmas.array();
    matX R = variances.diagonal();
    return R;
}

matX MagMeasurementBlock::calcInnovationCovariance(vecX& x, matX& P){
    matX R = this->calcMeasurementCovariance();
    matX S = (*_H)*P*(*_H).transpose() + R;
    return S;
}

vecX MagMeasurementBlock::calcMeasurementEstimate(vecX& x, vecX& y){
    vecX X = this->_aux.getNominalState();
    matX toRfromB = q2DCM(X({0,1,2,3}));
    matX toBfromR = toRfromB.transpose();
    vecX h = toRfromB*y;
    vecX b = {sqrt(h(0)*h(0) + h(1)*h(1)), 0, h(2)};

    vecX yhat = toBfromR*b;
    return yhat;
}

mkf::EstWithNominal MagMeasurementBlock::applyError(vecX& x, vecX& X){
    mkf::EstWithNominal ewn;
    ewn.X(X.size());
    ewn.x = Eigen::VectorXd::Zero(x.size());

    vec3 alpha = 0.5*x({3,4,5});
    vecQ qa = {1,alpha};
    qa = qNormalize(qa);
    vecX X_ = X;
    ewn.X({0,1,2,3}) = qMult(X_({0,1,2,3}), qa);
    ewn.X({4,5,6}) = X_({4,5,6});

    ewn.x({0,1,2}) = x({0,1,2});
    ewn.x({6,7,8}) = x({6,7,8});

    return ewn;
}