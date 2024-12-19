#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>

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
    std::vector<std::string> row;
    std::vector<std::string> col_titles;
    std::vector<std::vector<double>> data;
    std::vector<double> dataline;
    std::string line, word, temp;

    std::fstream fin = openFile(filename);
    int idx = 0;
    while(!fin.eof()){
        row.clear();

        getline(fin, line);
        std::stringstream s(line);

        while(getline(s, word, ',')){
            // if(idx < 1){
            //     col_titles.push_back(word);
            // }else{
            //     dataline.clear();
            //     if(word.empty()){
            //         dataline.push_back(0.0);
            //     }else{
            //         dataline.push_back(std::stod(word));
            //     }
            // }
        }
        // data.push_back(dataline);
        idx += 1;
    }
    fin.close();

    // auto acc_x_idx = getColumnIdx(col_titles, "/vectornav/raw/imu/uncompaccel/x");

    // std::vector<double> raw_acc_x;
    // std::vector<double> raw_acc_y;
    // std::vector<double> raw_acc_z;
    // std::vector<double> raw_gyr_x;
    // std::vector<double> raw_gyr_y;
    // std::vector<double> raw_gyr_z;
    // std::vector<double> raw_mag_x;
    // std::vector<double> raw_mag_y;
    // std::vector<double> raw_mag_z;
    // for(int i = 1; i < idx; i++){
    //     raw_acc_x.push_back(data[i][acc_x_idx]);
    //     raw_acc_y.push_back(data[i][acc_x_idx+1]);
    //     raw_acc_z.push_back(data[i][acc_x_idx+2]);
    //     raw_gyr_x.push_back(data[i][acc_x_idx+3]);
    //     raw_gyr_y.push_back(data[i][acc_x_idx+4]);
    //     raw_gyr_z.push_back(data[i][acc_x_idx+5]);
    //     raw_mag_x.push_back(data[i][acc_x_idx+6]);
    //     raw_mag_y.push_back(data[i][acc_x_idx+7]);
    //     raw_mag_z.push_back(data[i][acc_x_idx+8]);
    // }
}