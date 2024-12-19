#pragma once

#include "mkf/measblock.h"

namespace mekf{
    class MagMeasurementBlock : mkf::MeasurementBlockBase{
    public:
        MagMeasurementBlock(vecX& meas_sigmas);
        ~MagMeasurementBlock();

        matX updateObservationMatrix(vecX& x, vecX& y) override;
        matX calcMeasurementCovariance() override;
        matX calcInnovationCovariance(vecX& x, matX& P) override;
        vecX calcMeasurementEstimate(vecX& x, vecX& y) override;
    };
}