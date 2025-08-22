#pragma once

#include <Eigen/Dense>

namespace mkf {

struct AuxData {
    Eigen::MatrixXd X;  // nominal state
    Eigen::VectorXd y;  // measurement
    Eigen::VectorXd x;  // error state

    AuxData(int x_size = 0, int y_size = 0, int X_size = 0)
        : X(Eigen::VectorXd::Zero(X_size)),
          y(Eigen::VectorXd::Zero(y_size)),
          x(Eigen::VectorXd::Zero(x_size))
    {}

    // AuxData() = default;
};

} // namespace mkf
