#pragma once

#include "eigenhelpers.h"

struct EstWithObs{
    vecX y_hat;
    matX H;
};

namespace mkf{
    class MeasurementBlock{
    public:
        MeasurementBlock(vecX& meas_sigmas);
        ~MeasurementBlock();

        virtual matX updateObservationMatrix(vecX& x, vecX& y);
        virtual matX calcMeasurementCovariance();
        virtual matX calcInnovationCovariance(vecX& x, matX& P);
        virtual EstWithObs update(vecX& x, vecX& y, bool relinearize = false);

        inline vecX getMeasurementSigmas() { return this->_meas_sigmas; };
        inline void setMeasurementSigmas(vecX& meas_sigmas) { this->_meas_sigmas = meas_sigmas; };
        inline matX getObservationMatrix() { return this->_H; };

    private:
        vecX _meas_sigmas;
        matX _H;
    };
}