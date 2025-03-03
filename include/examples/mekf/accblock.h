#pragma once

#include "mkf/measblock.h"

namespace mekf{
    class AccMeasurementBlock : public mkf::MeasurementBlockBase{
    public:
        AccMeasurementBlock(vecX& meas_sigmas);
        ~AccMeasurementBlock();

        matX updateObservationMatrix(const vecX& x, const vecX& y) override;
        matX calcMeasurementCovariance() override;
        vecX calcMeasurementEstimate(const vecX& x, const vecX& y) override;
        mkf::EstWithNominal applyError(vecX& x, vecX& X);
    };
}