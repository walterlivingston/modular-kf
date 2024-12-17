#include "mkf/stateblock.h"

using namespace mkf;

StateBlockBase::StateBlockBase(double& num_states, vecX& state_sigmas)
    : _num_states(num_states), _state_sigmas(state_sigmas){
        
    }