#include "mkf/stateblock.h"

using namespace mkf;

StateBlockBase::StateBlockBase(int num_states, vecX& state_sigmas)
    : _num_states(num_states), _state_sigmas(state_sigmas){
        
    }

StateBlockBase::~StateBlockBase(){};

matX StateBlockBase::getStateTransitionMatrix(vecX& x, bool relinearize){ 
    if(relinearize){
        this->_F = this->updateStateTransitionMatrix(x);
    }
    return (*_F);
}