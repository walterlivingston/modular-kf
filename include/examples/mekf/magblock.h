#pragma once

#include "mkf/measblock.h"

namespace mekf{
    class MagMeasurementBlock : public mkf::MeasurementBlockBase{
    public:
        MagMeasurementBlock(vecX& meas_sigmas);
        ~MagMeasurementBlock();

        matX updateObservationMatrix(const vecX& x, const vecX& y) override;
        matX calcMeasurementCovariance() override;
        vecX calcMeasurementEstimate(const vecX& x, const vecX& y) override;
        mkf::EstWithNominal applyError(vecX& x, vecX& X);
    };
}