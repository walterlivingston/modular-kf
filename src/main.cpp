#include <iostream>

#include "mkf/kalmanfilter.hpp"
#include "mkf/covarianceblock.hpp"
#include "mkf/examples/mekf6/mekf_stateblock.hpp"
#include "mkf/examples/mekf6/mekf_measblock.hpp"

int main() {
    Eigen::Matrix<double, 6, 1> state_sigmas;
    state_sigmas << 1, 1, 1, 1, 1, 1;

    Eigen::Matrix<double, 3, 1> meas_sigmas;
    meas_sigmas << 1, 1, 1;

    mkf::AuxData aux(6, 3, 6);

    std::shared_ptr<mkf::StateBlock> stateBlock = std::make_shared<mkf::MEKFStateBlock>(6, state_sigmas);
    stateBlock->processAuxData(aux);
    std::shared_ptr<mkf::MEKFMeasBlockAcc> accMeasBlock = std::make_shared<mkf::MEKFMeasBlockAcc>(state_sigmas);
    std::shared_ptr<mkf::MEKFMeasBlockMag> magMeasBlock = std::make_shared<mkf::MEKFMeasBlockMag>(state_sigmas);
    std::shared_ptr<mkf::CovarianceBlock> covBlock = std::make_shared<mkf::CovarianceBlock>();

    mkf::KalmanFilter::Options opts;
    mkf::KalmanFilter kf(stateBlock, magMeasBlock, covBlock, opts);

}
