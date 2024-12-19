#include "examples/mekf/accblock.h"
#include "quaternions.h"

using namespace mekf;

AccMeasurementBlock::AccMeasurementBlock(vecX& meas_sigmas)
    : mkf::MeasurementBlockBase(meas_sigmas){

}

AccMeasurementBlock::~AccMeasurementBlock(){

}

matX AccMeasurementBlock::updateObservationMatrix(vecX& x, vecX& y){
    vecX X = this->_aux.getNominalState();
    matX toRfromB = q2DCM(X({0,1,2,3}));
    matX toBfromR = toRfromB.transpose();
    vecX g = {0, 0, -9.81};

    matX O = Eigen::MatrixXd::Zero(3,3);
    matX I = Eigen::MatrixXd::Identity(3,3);
    matX H(9,6);
    H << I, O, I,
         O, skew(toBfromR*g), O;
    this->_H = H;
}

matX AccMeasurementBlock::calcMeasurementCovariance(){
    vecX variances = this->_meas_sigmas.array()*this->_meas_sigmas.array();
    matX R = variances.diagonal();
    return R;
}

matX AccMeasurementBlock::calcInnovationCovariance(vecX& x, matX& P){
    matX R = this->calcMeasurementCovariance();
    matX S = (*_H)*P*(*_H).transpose() + R;
    return S;
}

vecX AccMeasurementBlock::calcMeasurementEstimate(vecX& x, vecX& y){
    vecX X = this->_aux.getNominalState();
    matX toRfromB = q2DCM(X({0,1,2,3}));
    matX toBfromR = toRfromB.transpose();
    vecX g = {0, 0, -9.81};

    vecX yhat = toBfromR*g;
    return yhat;
}