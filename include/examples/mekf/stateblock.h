#pragma once

#include "mkf/stateblock.h"

namespace mekf{
    class StateBlock : mkf::StateBlockBase{
    public:
        StateBlock(double& num_states, vecX& state_sigmas);
        ~StateBlock();

        matX updateStateTransitionMatrix(vecX& x) override;
        matX calcProcessCovarianceMatrix(double& dt) override;
        vecX updateState(vecX& x, double& dt) override;
        mkf::EstWithCov propagate(vecX& x, matX& P, double& dt, bool relinearize = false) override;
    };
}