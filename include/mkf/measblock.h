#pragma once

#include <optional>

#include "eigenhelpers.h"
#include "auxdata.h"

namespace mkf{
    struct EstInnWithCov {
        vecX x;
        vecX z;
        matX P;
        matX S;
    };

    struct EstWithNominal{
        vecX x;
        vecX X;
    };

    class MeasurementBlockBase{
    public:
        MeasurementBlockBase(vecX& meas_sigmas);
        ~MeasurementBlockBase();
        matX getObservationMatrix(vecX& x, vecX& y, bool relinearize = false);

        virtual matX updateObservationMatrix(vecX& x, vecX& y);
        virtual matX calcMeasurementCovariance();
        virtual matX calcInnovationCovariance(vecX& x, matX& P);
        virtual vecX calcMeasurementEstimate(vecX& x, vecX& y);

        inline vecX getMeasurementSigmas() { return this->_meas_sigmas; };
        inline void setMeasurementSigmas(vecX& meas_sigmas) { this->_meas_sigmas = meas_sigmas; };

        inline EstWithNominal applyError(vecX& x, vecX& X){
            std::cerr << "Error: Measurement block error handling has not been implemented!" << std::endl;
        };
        inline void processAuxData(AuxData& aux){
            std::cerr << "Error: Measurement block aux data handling has not been implemented!" << std::endl;
        };


    protected:
        vecX _meas_sigmas;
        std::optional<matX> _H;
        AuxData _aux;
    };
}