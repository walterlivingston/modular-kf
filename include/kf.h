#pragma once

#include <string>

#include "eigenhelpers.h"
#include "stateblock.h"
#include "measblock.h"

namespace mkf{
    class KalmanFilter{
    public:
        KalmanFilter();
        ~KalmanFilter();

        void process(double& dt, mkf::StateBlock state_block);
        void update(vecX& y, mkf::MeasurementBlock meas_block);

        inline mkf::StateBlock getStateBlock() { return this->_state_block; };
        inline void setStateBlock(mkf::StateBlock& state_block){ this->_state_block = state_block; };
        inline mkf::MeasurementBlock getMeasurementBlock() { return this->_meas_block; };
        inline void setMeasurementBlock(mkf::MeasurementBlock& meas_block) { this->_meas_block = meas_block; };
        inline vecX getStates() { return this->_x; };
        inline vecX getNominalStates() { return this->_X; };
        inline vecX getInnovation() { return this->_z; };
        inline matX getStateCovarianceMatrix() { return this->_P; };
        inline EstWithCov getStateWithCovariance() { return {this->_x, this->_P}; };

    private:
        mkf::StateBlock _state_block;
        mkf::MeasurementBlock _meas_block;
        vecX _x;
        vecX _X;
        matX _P;
        vecX _z;
        std::string mode;
    };
}