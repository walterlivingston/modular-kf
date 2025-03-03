#include "mkf/covblock.h"
#include "mkf/kf.h"

using namespace mkf;

CovarianceBlockBase::CovarianceBlockBase() {};
CovarianceBlockBase::~CovarianceBlockBase() {};

matX CovarianceBlockBase::calcProcessCovarianceMatrix(KalmanFilter* filter, double dt, std::optional<StateBlockBase> customStateBlock){
    StateBlockBase block = filter->getStateBlock();
    if (customStateBlock){
        auto block = *customStateBlock;
    }
    return block.calcProcessCovarianceMatrix(dt);
}

matX CovarianceBlockBase::calcMeasurementCovarianceMatrix(KalmanFilter* filter, std::optional<MeasurementBlockBase> customMeasBlock){
    MeasurementBlockBase block = filter->getMeasurementBlock();
    if (customMeasBlock){
        auto block = *customMeasBlock;
    }
    return block.calcMeasurementCovariance();
}

matX CovarianceBlockBase::calcInnovationCovarianceMatrix(KalmanFilter* filter){
    auto stateBlock = filter->getStateBlock();
    auto measBlock = filter->getMeasurementBlock();
    auto covBlock = filter->getCovarianceBlock();

    matX H = measBlock.getObservationMatrix();
    matX P = filter->getStateCovarianceMatrix();
    matX R = covBlock.calcMeasurementCovarianceMatrix(filter);
    matX S = H*P*H.transpose() + R;

    return S;
}