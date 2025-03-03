#pragma once

#include <optional>

#include "mkf/eigenhelpers.h"
#include "mkf/auxdata.h"
#include "mkf/stateblock.h"

namespace mkf{

    class MeasurementBlockBase{
    public:
        MeasurementBlockBase(const vecX& meas_sigmas);
        ~MeasurementBlockBase();
        matX getObservationMatrix(const vecX& x = vecX(), const vecX& y = vecX(), bool relinearize = false);

        virtual matX updateObservationMatrix(const vecX& x, const vecX& y);
        virtual matX calcMeasurementCovariance();
        virtual vecX calcMeasurementEstimate(const vecX& x, const vecX& y);

        inline vecX getMeasurementSigmas() { return this->_meas_sigmas; };
        inline void setMeasurementSigmas(const vecX& meas_sigmas) { this->_meas_sigmas = meas_sigmas; };

        inline EstWithNominal applyError(const vecX& x, const vecX& X){
            std::cerr << "Error: Measurement block error handling has not been implemented!" << std::endl;
            return EstWithNominal();
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