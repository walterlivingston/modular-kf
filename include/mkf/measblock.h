#pragma once

#include "eigenhelpers.h"
#include "auxdata.h"

namespace mkf{
    struct EstAndInnWithCov {
        vecX x;
        vecX z;
        matX P;
        matX S;
    };

    struct EstWithNominal{
        vecX x;
        vecX X;
    };

    class MeasurementBlock{
    public:
        MeasurementBlock(vecX& meas_sigmas);
        ~MeasurementBlock();

        virtual matX updateObservationMatrix(vecX& x, vecX& y);
        virtual matX calcMeasurementCovariance();
        virtual matX calcInnovationCovariance(vecX& x, matX& P);
        virtual EstAndInnWithCov update(vecX& x, vecX& y, bool relinearize = false);

        inline vecX getMeasurementSigmas() { return this->_meas_sigmas; };
        inline void setMeasurementSigmas(vecX& meas_sigmas) { this->_meas_sigmas = meas_sigmas; };
        inline matX getObservationMatrix() { return this->_H; };

        inline EstWithNominal applyError(vecX& x, vecX& X){
            std::cerr << "Error: Measurement block error handling has not been implemented!" << std::endl;
        };
        inline void processAuxData(AuxData& aux){
            std::cerr << "Error: Measurement block aux data handling has not been implemented!" << std::endl;
        };


    private:
        vecX _meas_sigmas;
        matX _H;
        AuxData _aux;
    };
}