#pragma once

#include <optional>

#include "mkf/stateblock.h"
#include "mkf/measblock.h"
#include "mkf/eigenhelpers.h"

namespace mkf{
    class KalmanFilter;
    class CovarianceBlockBase{
        public:
            CovarianceBlockBase();
            ~CovarianceBlockBase();

            matX calcProcessCovarianceMatrix(KalmanFilter* filter, double dt, std::optional<StateBlockBase> customStateBlock = std::nullopt);
            matX calcMeasurementCovarianceMatrix(mkf::KalmanFilter* filter, std::optional<MeasurementBlockBase> customMeasBlock = std::nullopt);
            matX calcInnovationCovarianceMatrix(mkf::KalmanFilter* filter);
    };
}