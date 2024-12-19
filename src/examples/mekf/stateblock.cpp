#include "examples/mekf/stateblock.h"
#include "quaternions.h"

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

mkf::EstWithNominal StateBlock::applyError(vecX& x, vecX& X, double& dt){
    mkf::EstWithNominal ewn;
    ewn.X(X.size());
    ewn.x = x;

    vecQ qw = {0, x({0,1,2})};
    vecX X_ = X;
    if(qNorm(X_({0,1,2,3})) < 1e-10){
        vecQ qI = {1, 0, 0, 0};
        X_({0,1,2,3}) = qI;
    }

    ewn.X({0,1,2,3}) = X_({0,1,2,3}) + 0.5*qMult(X_({0,1,2,3}), qw)*dt;
    ewn.X({4,5,6}) = X_({4,5,6});
}