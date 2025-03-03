#pragma once

#include <string>
#include <optional>

#include "mkf/eigenhelpers.h"
#include "mkf/stateblock.h"
#include "mkf/measblock.h"
#include "mkf/covblock.h"

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
        KalmanFilter(mkf::StateBlockBase& state_block, mkf::MeasurementBlockBase& meas_block, std::optional<CovarianceBlockBase> cov_block = std::nullopt, std::optional<kfOptions> opts = std::nullopt);
        ~KalmanFilter();

        void process(double& dt, std::optional<mkf::StateBlockBase> state_block = std::nullopt);
        void update(vecX& y, std::optional<mkf::MeasurementBlockBase> meas_block = std::nullopt);

        inline mkf::StateBlockBase getStateBlock() { return this->_state_block; };
        inline void setStateBlock(mkf::StateBlockBase& state_block){ this->_state_block = state_block; };
        inline mkf::MeasurementBlockBase getMeasurementBlock() { return this->_meas_block; };
        inline void setMeasurementBlock(mkf::MeasurementBlockBase& meas_block) { this->_meas_block = meas_block; };
        inline mkf::CovarianceBlockBase getCovarianceBlock() { return this->_cov_block; };
        inline void setCovarianceBlock(mkf::CovarianceBlockBase& cov_block) { this->_cov_block = cov_block; };
        inline vecX getStates() { return this->_x; };
        inline vecX getNominalStates() { return this->_X; };
        inline vecX getInnovation() { return this->_z; };
        inline matX getStateCovarianceMatrix() { return this->_P; };
        inline EstWithCov getStateWithCovariance() { return {this->_x, this->_P}; };

    private:
        mkf::StateBlockBase _state_block;
        mkf::MeasurementBlockBase _meas_block;
        mkf::CovarianceBlockBase _cov_block;
        vecX _x = vecX(0);
        vecX _X = vecX(0);
        matX _P;
        vecX _z = vecX(0);
        KFTYPE mode = KF_LINEAR;
    };
}