#pragma once

#include "mkf/measblock.h"

namespace mekf{
    class AccMeasurementBlock : mkf::MeasurementBlockBase{
    public:
        AccMeasurementBlock(vecX& meas_sigmas);
        ~AccMeasurementBlock();

        matX updateObservationMatrix(vecX& x, vecX& y) override;
        matX calcMeasurementCovariance() override;
        matX calcInnovationCovariance(vecX& x, matX& P) override;
        vecX calcMeasurementEstimate(vecX& x, vecX& y) override;
    };
}