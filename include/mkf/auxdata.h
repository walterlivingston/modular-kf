#pragma once

#include <optional>

#include "eigenhelpers.h"

namespace mkf{
    class AuxData{
    public:
        AuxData();
        ~AuxData();

        inline vecX getNominalState() { return this->_X; };
        inline void setNominalState(vecX& X) { this->_X = X; };
        inline vecX getMeasurements() { return this->_y; };
        inline void setMeasurements(vecX& y) { this->_y = y; };
    private:
        vecX _X = vecX(0);
        vecX _y = vecX(0);
    };
}