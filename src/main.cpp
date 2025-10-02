#include <boost/tokenizer.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "mkf/kalmanfilter.hpp"
#include "mkf/covarianceblock.hpp"
#include "mkf/examples/mekf6/mekf_stateblock.hpp"
#include "mkf/examples/mekf6/mekf_measblock.hpp"

std::vector<Eigen::VectorXd> loadCsv(const std::string &filename, bool hasHeader = true) {
    std::vector<Eigen::VectorXd> data;
    std::ifstream file(filename);
    if (!file.is_open()) throw std::runtime_error("Cannot open file: " + filename);

    std::string line;
    if (hasHeader) std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        boost::tokenizer<boost::escaped_list_separator<char>> tok(line);
        std::vector<double> row;
        for (auto &t : tok) {
            std::string s = t;
            s.erase(0, s.find_first_not_of(" \t\r\n"));
            s.erase(s.find_last_not_of(" \t\r\n") + 1);
            if (!s.empty()) row.push_back(std::stod(s));
        }
        if (!row.empty()) {
            Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd>(row.data(), row.size());
            data.push_back(vec);
        }
    }
    return data;
}


int main() {
    Eigen::Matrix<double, 6, 1> state_sigmas;
    state_sigmas << 1, 1, 1, 1, 1, 1;

    Eigen::Matrix<double, 3, 1> meas_sigmas;
    meas_sigmas << 1, 1, 1;

    mkf::AuxData aux(6, 3, 6);

    std::shared_ptr<mkf::StateBlock> stateBlock = std::make_shared<mkf::MEKFStateBlock>(6, state_sigmas);
    stateBlock->processAuxData(aux);
    std::shared_ptr<mkf::MEKFMeasBlockAcc> accMeasBlock = std::make_shared<mkf::MEKFMeasBlockAcc>(state_sigmas);
    std::shared_ptr<mkf::MEKFMeasBlockMag> magMeasBlock = std::make_shared<mkf::MEKFMeasBlockMag>(state_sigmas);
    std::shared_ptr<mkf::CovarianceBlock> covBlock = std::make_shared<mkf::CovarianceBlock>();

    mkf::KalmanFilter::Options opts;
    mkf::KalmanFilter kf(stateBlock, magMeasBlock, covBlock, opts);

    std::cout << "Kalman filter has been initialized with " << kf.state_block->num_states() << " states." << std::endl;

    auto gyrData = loadCsv("/Users/livw/dev/modular-kf/data/AHRS/1154_1188_Denson_Dr-2025-01-01_22-17-52/Gyroscope.csv");
    auto accData = loadCsv("/Users/livw/dev/modular-kf/data/AHRS/1154_1188_Denson_Dr-2025-01-01_22-17-52/Accelerometer.csv");
    auto magData = loadCsv("/Users/livw/dev/modular-kf/data/AHRS/1154_1188_Denson_Dr-2025-01-01_22-17-52/Magnetometer.csv");

    size_t n = std::min(accData.size(), std::min(gyrData.size(), magData.size()));

    double first_time = gyrData[0](1);
    double last_time = 0;

    auto accIdx = 0;
    auto magIdx = 0;
    for (size_t i = 1; i < n; ++i) {
        const Eigen::VectorXd &gyrRow = gyrData[i];
        const Eigen::VectorXd &accRow = accData[accIdx];
        const Eigen::VectorXd &magRow = magData[magIdx];

        auto current_time = gyrRow(1) - first_time;
        auto dt = current_time - last_time;

        auto aux = kf.state_block->aux();
        Eigen::VectorXd gyr(3);
        gyr << gyrRow(2), gyrRow(3), gyrRow(4);
        aux.y = gyr;

        kf.process(dt);

        if (current_time >= (accRow(1) - first_time)) {
            accIdx += 1;

            Eigen::VectorXd acc(3);
            acc << accRow(2), accRow(3), accRow(4);

            kf.update(acc);
        }

        if (current_time >= (magRow(1) - first_time)) {
            magIdx += 1;

            Eigen::VectorXd mag(3);
            mag << magRow(2), magRow(3), magRow(4);

            kf.update(mag);
        }
    }
}
