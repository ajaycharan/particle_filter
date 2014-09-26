/***************************************************************
 *
 * Implements the robot simulator
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/17/2014 (MM/DD/YYYY)
 ***************************************************************/


#ifndef ROBOTSIMULATOR_H
#define ROBOTSIMULATOR_H

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ParticleFilter.h"

namespace lab1 {

    struct Robot {
        // Pose
        float x;
        float y;
        float theta;
        // Pose of the laser
        float xl;
        float yl;
        float thetal;
    };

    class simulator {
        // Robot in the world
        Robot beatle;
        // Map of the world
        WorldMap wean_map;

    };

}





#endif
