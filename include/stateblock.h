#pragma once

#include <Eigen/Dense>

#include "eigenhelpers.h"

struct EstWithCov {
    vecX estimate;
    matX covariance;
};

namespace mkf{
    class StateBlock{
    public:
        StateBlock(double& num_states, vecX& state_sigmas);
        ~StateBlock();

        virtual matX updateStateTransitionMatrix(vecX& x);
        virtual matX calcProcessCovarianceMatrix(double& dt);
        virtual matX updateState(vecX& x, double& dt);
        virtual EstWithCov propagate(vecX& x, double& dt, bool relinearize = false);

        inline double getNumStates() { return _num_states; };
        inline void setNumStates(double& num_states) { this->_num_states = num_states; };
        inline vecX getStateSigmas() { return _state_sigmas; };
        inline void setStateSigmas(vecX& state_sigmas) { this->_state_sigmas = state_sigmas; };
        inline vecX getStateTransitionMatrix() { return _F; };

    private:
        double _num_states;
        vecX _state_sigmas;
        matX _F;
    };
}