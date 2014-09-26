
/***************************************************************
 *
 * Class Utilites implements the utility functions to be
 * used in the simulator, including reading files or other
 * visualizations.
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/17/2014 (MM/DD/YYYY)
 ***************************************************************/



#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "Utilities.h"

#define UTILITIES_DEBUG

using namespace std;
using namespace cv;

namespace lab1 {


    /***********************************************************
     * @brief: Read the map
     *
     *      Read in the predefined map. Each element in the map represents
     *      if the location is occupied or not
     *
     * @param  file_path: full path of the map file
     * @return my_map   : a struct of WorldMap to store the map info
     ***********************************************************/
    void Utilities::ReadMap(string file_path, WorldMap& my_map) {

        // Open the file and check if the file is successfully opened
        ifstream fin(file_path.c_str());
        if (!fin) {
            cerr << "Error: Uable to open file: " << file_path << endl;
            return;
        }

        // Read the specifications
        while (1) {
            string key_word;
            fin >> key_word;

            if (key_word.compare("robot_specifications->global_mapsize_x") == 0) {
                fin >> my_map.map_size_x;
#ifdef UTILITIES_DEBUG
                cout << "Found key word \"global_mapsize_x\"." << endl;
#endif
            } else if (key_word.compare("robot_specifications->global_mapsize_y") == 0) {
                fin >> my_map.map_size_y;
#ifdef UTILITIES_DEBUG
                cout << "Found key word \"global_mapsize_y\"." << endl;
#endif
            } else if (key_word.compare("robot_specifications->resolution") == 0) {
                fin >> my_map.resolution;
#ifdef UTILITIES_DEBUG
                cout << "Found key word \"resolution\"." << endl;
#endif
            } else if (key_word.compare("robot_specifications->autoshifted_x") == 0) {
                fin >> my_map.auto_shifted_x;
#ifdef UTILITIES_DEBUG
                cout << "Found key word \"autoshifted_x\"." << endl;
#endif
            } else if (key_word.compare("robot_specifications->autoshifted_y") == 0) {
                fin >> my_map.auto_shifted_y;
#ifdef UTILITIES_DEBUG
                cout << "Found key word \"autoshifted_y\"." << endl;
#endif
            } else if (key_word.compare("global_map[0]:") == 0) {
                fin >> my_map.map_size_x;
                fin >> my_map.map_size_y;
#ifdef UTILITIES_DEBUG
                cout << "Found key word \"global_map[0]\"." << endl;
#endif
                break;
            }
        }

        // Read in the map
        my_map.env_map.create(my_map.map_size_x, my_map.map_size_y, CV_32F);
        for (int i = 0; i < my_map.map_size_y; ++i) {
            for (int j = 0; j < my_map.map_size_x; ++j) {
                fin >> my_map.env_map.at<float>(i, j);
            }
        }

        // Close the file
        fin.close();

        return;
    }


    /***********************************************************
     * @brief: Read the recorded data from odometry and laser
     *
     *      Read the data from the odometry and range finder
     *      to the corresponding vector of structs
     *
     * @param  file_path  : full path of the data log file
     * @return odom_data  : data from the odometry
     * @return laser_data: data from the range finder
     ***********************************************************/
    void Utilities::ReadDataLog(string file_path, vector<OdometryData>& odom_data, vector<LaserData>& laser_data) {

        // Open the file and check if the file is successfully opened
        ifstream fin(file_path.c_str());
        if (!fin) {
            cerr << "Error: Uable to open file: " << file_path << endl;
            return;
        }

        string data_type;
        fin >> data_type;

        while(fin) {

            if (data_type.compare("L") == 0) {
                // Push a new laser data into the vector
                laser_data.push_back(LaserData());
                unsigned int new_laser_data_idx = laser_data.size()-1;

                fin >> laser_data[new_laser_data_idx].odom_robot.x;
                fin >> laser_data[new_laser_data_idx].odom_robot.y;
                fin >> laser_data[new_laser_data_idx].odom_robot.theta;
                fin >> laser_data[new_laser_data_idx].odom_laser.x;
                fin >> laser_data[new_laser_data_idx].odom_laser.y;
                fin >> laser_data[new_laser_data_idx].odom_laser.theta;

                laser_data[new_laser_data_idx].readings.resize(180);
                for (int i = 0; i < 180; ++i){
                    fin >> laser_data[new_laser_data_idx].readings[i];
                }

                double ts;
                fin >> ts;
                laser_data[new_laser_data_idx].odom_robot.ts = ts;
                laser_data[new_laser_data_idx].odom_laser.ts = ts;

            } else if (data_type.compare("O") == 0) {
                // Push a new odometry data into the vector
                odom_data.push_back(OdometryData());
                unsigned int new_odom_data_idx = odom_data.size()-1;

                fin >> odom_data[new_odom_data_idx].x;
                fin >> odom_data[new_odom_data_idx].y;
                fin >> odom_data[new_odom_data_idx].theta;
                fin >> odom_data[new_odom_data_idx].ts;

            } else {
                cerr << "Error: Unrecognized measurement type: " << data_type << endl;
                return;
            }

            fin >> data_type;
        }

        fin.close();

        return;
    }

}





