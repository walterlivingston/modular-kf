#include "mkf/kf.h"

using namespace mkf;

KalmanFilter::KalmanFilter(StateBlockBase& state_block, MeasurementBlockBase& meas_block, std::optional<kfOptions&> opts)
    : _state_block(state_block), _meas_block(meas_block){
        double dt;
        if(opts){
            this->_x = opts->xi;
            this->_X = opts->Xi;
            dt = opts->dt;
            this->mode = opts->mode;
        }

        this->_P = this->_state_block.calcProcessCovarianceMatrix(dt);
        
        switch(this->mode){
            case KF_LINEAR:
                this->_state_block.updateStateTransitionMatrix(this->_x);
                this->_meas_block.updateObservationMatrix(this->_x, this->_z);
                break;
            case KF_EXTENDED:
                break;
            case KF_ERROR:
                this->_state_block.applyError(this->_x, this->_X, dt);
                this->_meas_block.applyError(this->_x, this->_X);

                AuxData aux;
                aux.setNominalState(this->_X);
                this->_state_block.processAuxData(aux);
                this->_meas_block.processAuxData(aux);
                break;
        }
    }

KalmanFilter::~KalmanFilter(){

}

void KalmanFilter::process(double& dt, std::optional<StateBlockBase&> state_block){
    if(state_block){
        this->_state_block = *state_block;
    }

    matX I = Eigen::MatrixXd::Identity(this->_state_block.getNumStates(), this->_state_block.getNumStates());
    matX F = this->_state_block.getStateTransitionMatrix(this->_x, this->mode == (KF_EXTENDED || KF_ERROR));
    matX Phi = I + F*dt;
    matX Qd = this->_state_block.calcProcessCovarianceMatrix(dt);

    if (this->mode == (KF_EXTENDED || KF_ERROR)){
        this->_x = this->_state_block.calcState(this->_x, dt);
    }else{
        this->_x = Phi*this->_x;
    }
    this->_P = Phi*(this->_P)*Phi.transpose();

    if(this->mode == KF_ERROR){
        EstWithNominal ewn = this->_state_block.applyError(this->_x, this->_X, dt);
        this->_X = ewn.x;
        this->_X = ewn.X;

        AuxData aux;
        aux.setNominalState(this->_X);
        this->_state_block.processAuxData(aux);
    }
}

void KalmanFilter::update(vecX& y, std::optional<MeasurementBlockBase&> meas_block){
    if(meas_block){
        this->_meas_block = *meas_block;
    }

    matX H = this->_meas_block.getObservationMatrix(this->_x, y, this->mode == (KF_EXTENDED || KF_ERROR));
    matX R = this->_meas_block.calcMeasurementCovariance();
    matX S = this->_meas_block.calcInnovationCovariance(this->_x, this->_P);
    matX I = Eigen::MatrixXd::Identity(this->_state_block.getNumStates(), this->_state_block.getNumStates());

    matX L = this->_P*H.transpose()*S.inverse();
    vecX yhat;
    if (this->mode == (KF_EXTENDED || KF_ERROR)){
        yhat = this->_meas_block.calcMeasurementEstimate(this->_x, y);
    }else{
        yhat = H*this->_x;
    }
    this->_z = y - yhat;
    this->_x = this->_x + L*this->_z;
    this->_P = (I - L*H)*this->_P*(I - L*H).transpose() + L*R*L.transpose();

    if(this->mode == KF_ERROR){
        EstWithNominal ewn = this->_meas_block.applyError(this->_x, this->_X);
        this->_X = ewn.x;
        this->_X = ewn.X;

        AuxData aux;
        aux.setNominalState(this->_X);
        aux.setMeasurements(y);
        this->_meas_block.processAuxData(aux);
    }
}