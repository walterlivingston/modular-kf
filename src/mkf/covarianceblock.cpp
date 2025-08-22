#include "mkf/covarianceblock.hpp"

#include "mkf/kalmanfilter.hpp"
#include "mkf/stateblock.hpp"
#include "mkf/measurementblock.hpp"

namespace mkf {

CovarianceBlock::mat CovarianceBlock::calcProcessCovarianceMatrix(
    const KalmanFilter &filter, double dt, const StateBlockPtr &customStateBlock)
{
    StateBlockPtr sBlock = customStateBlock ? customStateBlock : filter.state_block;
    if (!sBlock) throw std::runtime_error("State block is null in calcProcessCovarianceMatrix");
    return sBlock->calcProcessCovarianceMatrix(dt);
}

CovarianceBlock::mat CovarianceBlock::calcMeasurementCovarianceMatrix(
    const KalmanFilter &filter, const MeasurementBlockPtr &customMeasBlock)
{
    MeasurementBlockPtr mBlock = customMeasBlock ? customMeasBlock : filter.measurement_block;
    if (!mBlock) throw std::runtime_error("Measurement block is null in calcMeasurementCovarianceMatrix");
    return mBlock->calcMeasurementCovarianceMatrix();
}

std::pair<CovarianceBlock::mat,bool> CovarianceBlock::calcInnovationCovarianceMatrix(
    const KalmanFilter &filter)
{
    const mat &H = filter.measurement_block->H();
    const mat &P = filter.P;
    const mat R = filter.measurement_block->calcMeasurementCovarianceMatrix();
    const vec &z = filter.z;

    mat S = H * P * H.transpose() + R;
    S_ = S;

    r_ = std::sqrt(z.transpose() * S.inverse() * z);

    bool reject = false;
    return {S, reject};
}

} // namespace mkf
