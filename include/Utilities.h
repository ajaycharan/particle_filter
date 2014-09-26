
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


#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <string>
#include <vector>
#include "RobotSimulator.h"

namespace lab1 {

    class Utilities {

    public:
        static void ReadMap(std::string file_path, WorldMap& my_map);
        static void ReadDataLog(std::string file_path, std::vector<OdometryData>& odom_data, std::vector<LaserData>& laser_data);

    };

}





#endif



