/***************************************************************
 *
 * Test the implementations of the classes
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/17/2014 (MM/DD/YYYY)
 ***************************************************************/



#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "RobotSimulator.h"
#include "Utilities.h"

using namespace std;
using namespace cv;
using namespace lab1;


int main (int argc, char *argv[]) {

    // Path of the files
    string map_file_path = "/home/ke/cplusplus_ws/particle_filter/data/map/wean.dat";
    string log_file_path = "/home/ke/cplusplus_ws/particle_filter/data/log/robotdata1.log";

    // Read the map
    WorldMap my_map;
    Utilities::ReadMap(map_file_path, my_map);

    // Show the map as an image
    Mat map_img;
    my_map.env_map.convertTo(map_img, -1, 0.5f, 0.5f);
    namedWindow("Map");
    imshow("Map", map_img);
    waitKey(0);

    // Read the data log
    vector<OdometryData> odom_data(0);
    vector<LaserData> laser_data(0);
    Utilities::ReadDataLog(log_file_path, odom_data, laser_data);

    return 1;
}








