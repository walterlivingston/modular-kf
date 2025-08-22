#pragma once

#include <Eigen/Dense>

#include "quaternions.h"

#include "mkf/measurementblock.hpp"
#include "mkf/utils.hpp"

namespace mkf {

class MEKFMeasBlockMag : public MeasurementBlock {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;

    MEKFMeasBlockMag(const vec &meas_sigmas)
        : MeasurementBlock(meas_sigmas)
    {
        g_ = Eigen::Vector3d(0, 0, 9.81);
    }

    // Update measurement
    std::pair<vec, mat> update(const vec &x_, const vec &y_, bool relinearize = false) override
    {
        vec yhat;
        mat H;

        if (relinearize || H_.array().isNaN().all()) {
            H_ = updateObservationMatrix(x_, y_);
            H = H_;
            vec X = aux_.X;
            Eigen::Matrix3d Cb_R = q2DCM(X.head(4));
            Eigen::Vector3d m = y_.head(3);
            Eigen::Vector3d b;
            b << std::sqrt(std::pow(m(0),2) + std::pow(m(1),2)), 0, m(2);
            yhat = Cb_R.transpose() * b - H * x_;
        } else {
            H = H_;
            yhat = H * x_;
        }

        return {yhat, H};
    }

    // Update observation matrix
    mat updateObservationMatrix(const vec & /*x_*/, const vec & /*y_*/) override
    {
        vec X = aux_.X;
        Eigen::Matrix3d C = q2DCM(X.head(4));
        Eigen::Vector3d m = aux_.y.head(3);
        Eigen::Vector3d b;
        b << std::sqrt(std::pow(m(0),2) + std::pow(m(1),2)), 0, m(2);

        mat H = mat::Zero(3, 3);
        H.block<3,3>(0,0) = utils::skew(C.transpose() * b);

        return H;
    }

    // Measurement covariance
    mat calcMeasurementCovarianceMatrix() override
    {
        return meas_sigmas_.array().square().matrix().asDiagonal();
    }

    // Apply error
    std::pair<vec, vec> applyError(const vec &x_, const vec &X_) override
    {
        Eigen::Vector3d alpha = x_.head(3) * 0.5;
        Eigen::Vector4d qa;
        qa << 1, alpha(0), alpha(1), alpha(2);

        vec X = vec::Zero(X_.size());
        X.head(4) = qMult(X_.head(4), qa);
        X.tail(X_.size() - 4) = X_.tail(X_.size() - 4);

        aux_.x = x_;

        vec x = vec::Zero(x_.size());
        x.head(3).setZero();
        x.tail(3) = x_.segment(3,3);

        return {x, X};
    }

private:
    Eigen::Vector3d g_;
};

class MEKFMeasBlockAcc : public MeasurementBlock {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;

    MEKFMeasBlockAcc(const vec &meas_sigmas)
        : MeasurementBlock(meas_sigmas)
    {
        g_ = Eigen::Vector3d(0, 0, 9.81);
    }

    // Update measurement
    std::pair<vec, mat> update(const vec &x_, const vec &y_, bool relinearize = false) override
    {
        vec yhat;
        mat H;

        if (relinearize || H_.array().isNaN().all()) {
            H_ = updateObservationMatrix(x_, y_);
            H = H_;
            vec X = aux_.X;
            Eigen::Matrix3d Cb_R = q2DCM(X.head(4));
            yhat = Cb_R.transpose() * g_ - H * x_;
        } else {
            H = H_;
            yhat = H * x_;
        }

        return {yhat, H};
    }

    // Update observation matrix
    mat updateObservationMatrix(const vec & /*x_*/, const vec & /*y_*/) override
    {
        vec X = aux_.X;
        Eigen::Matrix3d C = q2DCM(X.head(4));
        Eigen::Vector3d m = aux_.y.head(3);

        mat H = mat::Zero(3, 3);
        H.block<3,3>(0,0) = utils::skew(C.transpose() * g_);

        return H;
    }

    // Measurement covariance
    mat calcMeasurementCovarianceMatrix() override
    {
        return meas_sigmas_.array().square().matrix().asDiagonal();
    }

    // Apply error
    std::pair<vec, vec> applyError(const vec &x_, const vec &X_) override
    {
        Eigen::Vector3d alpha = x_.head(3) * 0.5;
        Eigen::Vector4d qa;
        qa << 1, alpha(0), alpha(1), alpha(2);

        vec X = vec::Zero(X_.size());
        X.head(4) = qMult(X_.head(4), qa);
        X.tail(X_.size() - 4) = X_.tail(X_.size() - 4);

        aux_.x = x_;

        vec x = vec::Zero(x_.size());
        x.head(3).setZero();
        x.tail(3) = x_.segment(3,3);

        return {x, X};
    }

private:
    Eigen::Vector3d g_;
};

} // namespace mkf