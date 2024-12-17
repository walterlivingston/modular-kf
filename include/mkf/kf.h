#pragma once

#include <string>
#include <optional>

#include "eigenhelpers.h"
#include "stateblock.h"
#include "measblock.h"

namespace mkf{
    enum KFTYPE{
        KF_LINEAR = 0,
        KF_EXTENDED = 1,
        KF_ERROR = 2
    };

    struct kfOptions{
        vecX xi;
        vecX Xi;
        double dt;
        KFTYPE mode;
    };

    class KalmanFilter{
    public:
        KalmanFilter(mkf::StateBlock& state_block, mkf::MeasurementBlock& meas_block, std::optional<kfOptions&> opts);
        ~KalmanFilter();

        void process(double& dt, std::optional<mkf::StateBlock&> state_block);
        void update(vecX& y, std::optional<mkf::MeasurementBlock&> meas_block);

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
        vecX _x = vecX(0);
        vecX _X = vecX(0);
        matX _P;
        vecX _z = vecX(0);
        KFTYPE mode = KF_LINEAR;
    };
}