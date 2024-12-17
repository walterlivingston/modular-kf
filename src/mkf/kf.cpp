#include "mkf/kf.h"

using namespace mkf;

KalmanFilter::KalmanFilter(StateBlock& state_block, MeasurementBlock& meas_block, std::optional<kfOptions&> opts)
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

void KalmanFilter::process(double& dt, std::optional<StateBlock&> state_block){
    if(state_block){
        this->_state_block = *state_block;
    }

    EstWithCov ewc = this->_state_block.propagate(this->_x, dt, this->mode == (KF_EXTENDED || KF_ERROR));
    this->_P = ewc.P;
    this->_x = ewc.x;

    if(this->mode == KF_ERROR){
        EstWithNominal ewn = this->_state_block.applyError(this->_x, this->_X, dt);
        this->_X = ewn.x;
        this->_X = ewn.X;

        AuxData aux;
        aux.setNominalState(this->_X);
        this->_state_block.processAuxData(aux);
    }
}

void KalmanFilter::update(vecX& y, std::optional<MeasurementBlock&> meas_block){
    if(meas_block){
        this->_meas_block = *meas_block;
    }

    EstInnWithCov eiwc = this->_meas_block.update(this->_x, y, this->mode == (KF_EXTENDED || KF_ERROR));
    this->_P = eiwc.P;
    this->_x = eiwc.x;
    this->_z = eiwc.z;

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