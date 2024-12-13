#pragma once

#include "eigenhelpers.h"

namespace mkf{
    class AuxData{
    public:
        AuxData();
        ~AuxData();

        inline vecX getNominalState() { return this->_X; };
        inline vecX getMeasurements() { return this->_y; };
    private:
        vecX _X;
        vecX _y;
    };
}