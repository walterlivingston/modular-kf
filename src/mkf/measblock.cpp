#include "mkf/measblock.h"

using namespace mkf;

matX MeasurementBlockBase::getObservationMatrix(const vecX& x, const vecX& y, bool relinearize = false){
    if(relinearize){
        this->_H = this->updateObservationMatrix(x, y);
    }
    return (*_H);
}