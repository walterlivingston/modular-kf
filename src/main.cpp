#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <boost/algorithm/string.hpp>  // For boost::split
#include <matplot/matplot.h>

#include "mkf/mkf.h"
#include "examples/mekf/stateblock.h"
#include "examples/mekf/accblock.h"
#include "examples/mekf/magblock.h"

using namespace mekf;

std::string filename = "../data/putnam_ahrs_devel.csv";

std::fstream openFile(std::string filename){
    std::fstream fin;
    fin.open(filename, std::ios::in);
        if(fin.is_open()){
        std::cout << "File opened successfully" << std::endl;
        return fin;
    }else{
        std::cerr << "Error: File unable to be opened" << std::endl;
        return fin;
    }
}

int getColumnIdx(std::vector<std::string> vec, std::string col_name){
    auto it = std::find(vec.begin(), vec.end(), col_name);
    if(it != vec.end()){
        return std::distance(vec.begin(), it);
    }else{
        std::cerr << "Error: col_name not found with in vector!" << std::endl;
        return -1;
    }
}

int main(int argc, char **argv){
    std::vector<std::string> col_titles;
    std::vector<double> truth_yaw, truth_pitch, truth_roll;
    int raw_acc_idx, raw_gyr_idx, raw_mag_idx, truth_yaw_idx, truth_pitch_idx, truth_roll_idx;
    
    vecX state_sigmas;
    state_sigmas << 1, 1, 1, 1, 1, 1;
    vecX acc_sigmas;
    acc_sigmas << 1, 1, 1;
    vecX mag_sigmas;
    mag_sigmas << 1, 1, 1;

    mekf::StateBlock sBlock((int) 6, state_sigmas);
    mekf::AccMeasurementBlock accMBlock(acc_sigmas);
    mekf::MagMeasurementBlock magMBlock(mag_sigmas);

    mkf::KalmanFilter(sBlock, accMBlock);

    std::fstream fin = openFile(filename);

    int idx = 0;
    while(!fin.eof()){
        std::string line;
        getline(fin, line);
        if (!line.empty()){
            if(idx < 1){
                boost::split(col_titles, line, boost::is_any_of(","));
    
                raw_acc_idx = getColumnIdx(col_titles, "/vectornav/raw/imu/uncompaccel/x");
                raw_gyr_idx = getColumnIdx(col_titles, "/vectornav/raw/imu/uncompgyro/x");
                raw_mag_idx = getColumnIdx(col_titles, "/vectornav/raw/imu/uncompmag/x");
    
                truth_yaw_idx = getColumnIdx(col_titles, "/novatel_top/heading2/heading");
                truth_pitch_idx = getColumnIdx(col_titles, "/novatel_bottom/heading2/pitch");
                truth_roll_idx = getColumnIdx(col_titles, "/novatel_top/heading2/pitch");
            }else{
                std::vector<std::string> split_string;
                boost::split(split_string, line, boost::is_any_of(","));
                
                if (!split_string.at(truth_pitch_idx).empty()){
                    truth_pitch.push_back(std::stof(split_string.at(truth_pitch_idx)));
                }
                if (!split_string.at(truth_yaw_idx).empty()){
                    truth_yaw.push_back(std::stof(split_string.at(truth_yaw_idx)));
                    truth_roll.push_back(std::stof(split_string.at(truth_roll_idx)));
                }
                if (!split_string.at(raw_acc_idx).empty()){
                    // Time Update & Accelerometer Measurement Update
                }
                if (!split_string.at(raw_mag_idx).empty()){
                    // Magnetometer Measurement Update
                }
            }
        }
        idx += 1;
    }
    fin.close();
    std::cout << 1 << std::endl;
    matplot::plot(truth_yaw);
    matplot::show();
    return 0;
}