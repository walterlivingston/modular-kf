#pragma once

#include "mkf/stateblock.h"

namespace mekf{
    class StateBlock : public mkf::StateBlockBase{
    public:
        StateBlock(int num_states, vecX& state_sigmas);
        ~StateBlock();

        matX updateStateTransitionMatrix(vecX& x) override;
        matX calcProcessCovarianceMatrix(double& dt) override;
        vecX calcState(vecX& x, double& dt) override;
        mkf::EstWithNominal applyError(vecX& x, vecX& X, double& dt);
    };
}