#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "quaternions.h"

#include "mkf/stateblock.hpp"
#include "mkf/utils.hpp"

namespace mkf {

class MEKFStateBlock : public StateBlock {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;

    MEKFStateBlock(int num_states, const vec &state_sigmas)
        : StateBlock(num_states, state_sigmas)
    {}

    std::pair<vec, mat> propagate(const vec &x_, double dt, bool relinearize = false) override
    {
        mat Phi = mat::Identity(num_states_, num_states_);
        vec x;

        if (relinearize || F_.array().isNaN().all()) {
            F_ = updateStateTransitionMatrix(x_);
            x = updateState(x_, dt);
            Phi += F_ * dt;
        } else {
            Phi += F_ * dt;
            x = Phi * x_;
        }

        return {x, Phi};
    }

    mat updateStateTransitionMatrix(const vec &x_) override
    {
        mat F = mat::Zero(num_states_, num_states_);

        F.block<3,3>(0,0) = -utils::skew(aux_.y.head(3));
        F.block<3,3>(0,3) = -mat::Identity(3,3);

        return F;
    }

    vec updateState(const vec &x_, double /*dt*/) override
    {
        vec x = vec::Zero(num_states_);
        x.head(3).setZero();
        x.tail(3) = x_.segment(3,3);
        return x;
    }

    mat calcProcessCovarianceMatrix(double dt) override
    {
        mat Q = state_sigmas_.array().square().matrix().asDiagonal();

        mat Bw = mat::Zero(num_states_, num_states_);
        Bw.block<3,3>(0,0) = -mat::Identity(3,3);
        Bw.block<3,3>(3,3) = mat::Identity(3,3);

        mat Qd = Bw * Q * Bw.transpose() * dt;
        return Qd;
    }

    vec applyError(const vec &x_, const vec &X_, double dt) override
    {
        vec X = vec::Zero(X_.size());

        vec X_local = X_;
        if (qNorm(X_local.head(4)) < 1e-10) {
            X_local.head(4) << 1,0,0,0;
        }

        Eigen::Vector3d w_;
        if (aux_.y.size() > 3) {
            Eigen::Vector3d w = aux_.y.head(3);
            Eigen::Vector3d w1 = aux_.y.segment(3,3);
            w_ = 0.5 * (w + w1);
        } else {
            w_ = aux_.y.head(3);
        }

        X.head(4) = X_local.head(4) + 0.5 * qMult(X_local.head(4), Eigen::Vector4d(0, w_(0), w_(1), w_(2))) * dt;
        X.head(4) = qNormalize(X.head(4));

        X.tail(X_.size() - 4) = X_local.tail(X_.size() - 4);

        return X;
    }
};

} // namespace mkf
