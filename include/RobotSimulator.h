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
    public:
        //  Particle filter
        ParticleFilterConfig pf_config;
        ParticleFilter pf_estimator;

        // Robot in the world
        Robot beatle;

        // Map of the world
        WorldMap wean;

        // Time
        double sim_time;

        // All of the data from the sensors
        std::vector<OdometryData> odom_data;
        std::vector<LaserData> laser_data;

    private:

    public:
        simulator();
        ~simulator() {}

        // Interface of the simulator
        // Send sensor data to the particle filter based on the simulation time
        oneStepForward();

    private:

    };

}





#endif
