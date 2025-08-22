// measurementblock.hpp
#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <utility>

#include "mkf/auxdata.hpp"

namespace mkf {

class MeasurementBlock {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;

    explicit MeasurementBlock(const vec &meas_sigmas = vec{})
        : meas_sigmas_(meas_sigmas)
    {
        if (meas_sigmas_.size() == 0) {
            meas_sigmas_.setZero(0);
        }
        if (meas_sigmas_.size() > 0) {
            H_ = mat::Constant(meas_sigmas_.size(), meas_sigmas_.size(),
                                  std::numeric_limits<double>::quiet_NaN());
        }
    }

    virtual ~MeasurementBlock() = default;

    const vec& meas_sigmas() const noexcept { return meas_sigmas_; }
    const mat& H() const noexcept { return H_; }
    const AuxData& aux() const noexcept { return aux_; }

    void set_H(const mat &H) { H_ = H; }
    virtual void processAuxData(const AuxData &aux) { aux_ = aux; }

    virtual std::pair<vec, vec> applyError(const vec &x, const vec &X) {
        (void)x; (void)X;
        throw std::runtime_error("applyError() not implemented for this measurement block");
    }

    virtual mat updateObservationMatrix(const vec &x, const vec &y) = 0;
    virtual mat calcMeasurementCovarianceMatrix() = 0;
    virtual std::pair<vec, mat> update(const vec &x, const vec &y, bool relinearize) = 0;

protected:
    vec meas_sigmas_{};
    mat H_{};
    AuxData aux_{};
};

} // namespace mkf
