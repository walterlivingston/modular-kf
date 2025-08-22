#pragma once

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <cmath>
#include <stdexcept>

namespace mkf {

class StateBlock;
class MeasurementBlock;
class KalmanFilter;

class CovarianceBlock {
public:
    using vec = Eigen::VectorXd;
    using mat = Eigen::MatrixXd;
    using StateBlockPtr = std::shared_ptr<StateBlock>;
    using MeasurementBlockPtr = std::shared_ptr<MeasurementBlock>;

    CovarianceBlock()
        : S_(mat::Constant(0, 0, std::numeric_limits<double>::quiet_NaN())),
          r_(0.0)
    {}

    std::pair<mat, bool> calcInnovationCovarianceMatrix(const KalmanFilter &filter);

    mat calcProcessCovarianceMatrix(const KalmanFilter &filter,
                                       double dt,
                                       const StateBlockPtr &customStateBlock = nullptr);

    mat calcMeasurementCovarianceMatrix(const KalmanFilter &filter,
                                           const MeasurementBlockPtr &customMeasBlock = nullptr);

    const mat &S() const noexcept { return S_; }
    double r() const noexcept { return r_; }

private:
    mat S_;
    double r_;
};

} // namespace mkf
