// stateblock.hpp
#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <utility>

#include "mkf/auxdata.hpp"

namespace mkf {

class StateBlock {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;

    StateBlock(int num_states = 0, const vec &state_sigmas = vec{})
        : num_states_(num_states),
          state_sigmas_(state_sigmas)
    {
        if (num_states_ > 0 && state_sigmas_.size() == 0) {
            state_sigmas_.setZero(num_states_);
        }
        if (num_states_ > 0) {
            F_ = mat::Constant(num_states_, num_states_,
                                 std::numeric_limits<double>::quiet_NaN());
        }
    }
    virtual ~StateBlock() = default;

    int num_states() const noexcept { return num_states_; }
    const vec& state_sigmas() const noexcept { return state_sigmas_; }
    const mat& F() const noexcept { return F_; }
    const AuxData& aux() const noexcept { return aux_; }

    void set_F(const mat &F) { F_ = F; }
    virtual void processAuxData(const AuxData &aux) { aux_ = aux; }

    virtual vec applyError(const vec &x, const vec &X, double dt) {
        (void)x; (void)X; (void)dt;
        throw std::runtime_error("applyError() not implemented for this state block");
    }

    virtual mat updateStateTransitionMatrix(const vec &x) = 0;
    virtual vec updateState(const vec &x, double dt) = 0;
    virtual mat calcProcessCovarianceMatrix(double dt) = 0;
    virtual std::pair<vec, mat> propagate(const vec &x, double dt, bool relinearize) = 0;

protected:
    int num_states_{0};
    vec state_sigmas_{};
    mat F_{};
    AuxData aux_{};
};

} // namespace mkf
