#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <stdexcept>

#include "mkf/stateblock.hpp"
#include "mkf/measurementblock.hpp"
#include "mkf/covarianceblock.hpp"

namespace mkf {

class KalmanFilter {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;

    std::shared_ptr<StateBlock> state_block;
    std::shared_ptr<MeasurementBlock> measurement_block;
    std::shared_ptr<CovarianceBlock> covariance_block;

    vec x;  // State vector
    vec X;  // Nominal state vector
    mat P;  // State covariance
    vec z;  // Innovation
    std::string mode;

    struct Options {
        Eigen::VectorXd x_i;    // initial error state
        Eigen::VectorXd X_i;    // initial nominal state
        double dt;              // filter period
        std::string mode;       // filter mode (linear, extended, error)

        Options() : dt(1.0), mode("linear") {}
    };

    KalmanFilter(std::shared_ptr<StateBlock> sBlock,
                 std::shared_ptr<MeasurementBlock> mBlock,
                 std::shared_ptr<CovarianceBlock> cBlock,
                 const Options &options = Options())
        : state_block(std::move(sBlock)),
          measurement_block(std::move(mBlock)),
          covariance_block(std::move(cBlock)),
          mode(options.mode)
    {
        int n = state_block->num_states();
        x = options.x_i.size() == 0 ? vec::Zero(n) : options.x_i;
        X = options.X_i.size() == 0 ? vec::Zero(n) : options.X_i;

        if (mode == "error") {
            // Apply error handling to state and measurement blocks
            auto sAux = state_block->aux();
            sAux.X = X;
            state_block->applyError(x, X, options.dt);
            state_block->processAuxData(sAux);

            auto mAux = measurement_block->aux();
            mAux.X = X;
            measurement_block->applyError(x, X);
            measurement_block->processAuxData(mAux);
        }

        P = covariance_block->calcProcessCovarianceMatrix(*this, options.dt);

        if (mode == "linear") {
            state_block->updateStateTransitionMatrix(x);
            // state_block->set_F(state_block->updateStateTransitionMatrix(x));
            // measurement_block->set_H(measurement_block->updateObservationMatrix(x, vec::Zero(1)));
        }
    }

    // Time update
    void process(double dt,
                 std::shared_ptr<StateBlock> customStateBlock = nullptr,
                 std::shared_ptr<CovarianceBlock> customInnBlock = nullptr)
    {
        state_block = customStateBlock ? customStateBlock : state_block;
        covariance_block = customInnBlock ? customInnBlock : covariance_block;

        auto [x_new, Phi] = state_block->propagate(x, dt, mode == "extended" || mode == "error");
        mat Qd = covariance_block->calcProcessCovarianceMatrix(*this, dt, state_block);

        P = Phi * P * Phi.transpose() + Qd;
        x = x_new;

        if (mode == "error") {
            X = state_block->applyError(x, X, dt);
            auto aux = state_block->aux();
            aux.X = X;
            state_block->processAuxData(aux);
        }
    }

    // Measurement update
    void update(const vec &y,
                std::shared_ptr<MeasurementBlock> customMeasBlock = nullptr,
                std::shared_ptr<CovarianceBlock> customInnBlock = nullptr)
    {
        measurement_block = customMeasBlock ? customMeasBlock : measurement_block;
        covariance_block = customInnBlock ? customInnBlock : covariance_block;

        auto aux = measurement_block->aux();
        aux.X = X;
        aux.y = y;
        measurement_block->processAuxData(aux);

        auto [yhat, H] = measurement_block->update(x, y, mode == "extended" || mode == "error");
        auto R = covariance_block->calcMeasurementCovarianceMatrix(*this, measurement_block);
        vec oldz = z;
        z = y - yhat;

        auto [S, reject] = covariance_block->calcInnovationCovarianceMatrix(*this);
        if (!reject) {
            mat L = P * H.transpose() * S.inverse();
            int n = state_block->num_states();
            mat I = mat::Identity(n, n);

            x = x + L * z;
            P = (I - L * H) * P * (I - L * H).transpose() + L * R * L.transpose();

            if (mode == "error") {
                auto [x_new, X_new] = measurement_block->applyError(x, X);
                x = x_new;
                X = X_new;

                aux = measurement_block->aux();
                aux.X = X;
                aux.y = y;
                measurement_block->processAuxData(aux);
            }
        } else {
            z = oldz;
        }
    }
};

} // namespace mkf
