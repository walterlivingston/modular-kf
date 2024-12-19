#include "examples/mekf/stateblock.h"

using namespace mekf;

StateBlock::StateBlock(double& num_states, vecX& state_sigmas)
    : mkf::StateBlockBase(num_states, state_sigmas){

}

StateBlock::~StateBlock(){

}

matX StateBlock::updateStateTransitionMatrix(vecX& x){
    matX O = Eigen::MatrixXd::Zero(3,3);
    matX I = Eigen::MatrixXd::Identity(3,3);
    matX F;
    F << O, O, O,
         O, -skew(x({0,1,2})), -I,
         O, O, O;

    this->_F = F;
    return (*_F);
}

matX StateBlock::calcProcessCovarianceMatrix(double& dt){
    vecX variances = this->_state_sigmas.array()*this->_state_sigmas.array()*sqrt(dt);
    matX Q = variances.diagonal();

    matX O = Eigen::MatrixXd::Zero(3,3);
    matX I = Eigen::MatrixXd::Identity(3,3);
    matX Bw;
    Bw << I,  O, I, 
          O, -I, O,
          O,  O, I;

    matX Phi = (Eigen::MatrixXd::Identity(this->_num_states, this->_num_states) + (*_F)*dt);
    matX Qd = Phi*Bw*Q*Bw.transpose()*Phi.transpose()*dt;
    return Qd;
}

vecX StateBlock::calcState(vecX& x, double& dt){
    vecX ret_x;
    ret_x << x({0,1,2}), Eigen::VectorXd::Zero(3,1), x({6,7,8});
    return ret_x;
}