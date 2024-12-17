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
    return this->_F;
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

    matX Phi = (Eigen::MatrixXd::Identity(this->_num_states, this->_num_states) + this->_F*dt);
    matX Qd = Phi*Bw*Q*Bw.transpose()*Phi.transpose()*dt;
    return Qd;
}

vecX updateState(vecX& x, double& dt){
    vecX ret_x;
    ret_x << x({0,1,2}), Eigen::VectorXd::Zero(3,1), x({6,7,8});
    return ret_x;
}

mkf::EstWithCov StateBlock::propagate(vecX& x, matX& P, double& dt, bool relinearize){
    mkf::EstWithCov ewc;
    matX Phi;
    if(relinearize){
        this->updateStateTransitionMatrix(x);
        ewc.x = this->updateState(x, dt);
        Phi = (Eigen::MatrixXd::Identity(this->_num_states, this->_num_states) + this->_F*dt);
    }else{
        Phi = (Eigen::MatrixXd::Identity(this->_num_states, this->_num_states) + this->_F*dt);
        ewc.x = Phi*x;
    }

    matX Qd = this->calcProcessCovarianceMatrix(dt);
    ewc.P = Phi*P*Phi.transpose() + Qd;

    return ewc;
}