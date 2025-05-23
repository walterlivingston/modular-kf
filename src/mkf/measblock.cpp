#include "mkf/measblock.h"

using namespace mkf;

MeasurementBlockBase::MeasurementBlockBase(const vecX& meas_sigmas)
    :_meas_sigmas(meas_sigmas){};

MeasurementBlockBase::~MeasurementBlockBase(){};

matX MeasurementBlockBase::getObservationMatrix(const vecX& x, const vecX& y, bool relinearize){
    if(relinearize){
        this->_H = this->updateObservationMatrix(x, y);
    }
    return (*_H);
}