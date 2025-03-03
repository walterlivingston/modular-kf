#pragma once

#include <iostream>

#include "mkf/eigenhelpers.h"
#include "mkf/auxdata.h"

namespace mkf{
    struct EstWithCov {
        vecX x;
        matX P;
    };

    struct EstWithNominal{
        vecX x;
        vecX X;
    };

    class StateBlockBase{
    public:
        StateBlockBase(int num_states, vecX& state_sigmas);
        ~StateBlockBase();
        matX getStateTransitionMatrix(vecX& x, bool relinearize = false);

        virtual matX updateStateTransitionMatrix(vecX& x);
        virtual matX calcProcessCovarianceMatrix(double& dt);
        virtual vecX calcState(vecX& x, double& dt);

        inline double getNumStates() { return this->_num_states; };
        inline void setNumStates(double& num_states) { this->_num_states = num_states; };
        inline vecX getStateSigmas() { return this->_state_sigmas; };
        inline void setStateSigmas(vecX& state_sigmas) { this->_state_sigmas = state_sigmas; };

        inline EstWithNominal applyError(vecX& x, vecX& X, double& dt){
            std::cerr << "Error: State block error handling has not been implemented!" << std::endl;
            return EstWithNominal();
        };
        inline void processAuxData(AuxData& aux){
            std::cerr << "Error: State block aux data handling has not been implemented!" << std::endl;
        };

    protected:
        int _num_states;
        vecX _state_sigmas;
        std::optional<matX> _F;
        AuxData _aux;
    };
}