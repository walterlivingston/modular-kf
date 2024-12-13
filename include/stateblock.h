#pragma once

#include <Eigen/Dense>

#include "eigenhelpers.h"

struct EstWithCov {
    vecX x;
    matX P;
};

namespace mkf{
    class StateBlock{
    public:
        StateBlock(double& num_states, vecX& state_sigmas);
        ~StateBlock();

        virtual matX updateStateTransitionMatrix(vecX& x);
        virtual matX calcProcessCovarianceMatrix(double& dt);
        virtual vecX updateState(vecX& x, double& dt);
        virtual EstWithCov propagate(vecX& x, double& dt, bool relinearize = false);

        inline double getNumStates() { return this->_num_states; };
        inline void setNumStates(double& num_states) { this->_num_states = num_states; };
        inline vecX getStateSigmas() { return this->_state_sigmas; };
        inline void setStateSigmas(vecX& state_sigmas) { this->_state_sigmas = state_sigmas; };
        inline matX getStateTransitionMatrix() { return this->_F; };

    private:
        double _num_states;
        vecX _state_sigmas;
        matX _F;
    };
}