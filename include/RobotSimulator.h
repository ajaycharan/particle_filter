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

    struct SimulatorConfig {
        // file path of data
        std::string map_path;
        std::string log_path;

        // Configurations for the particle filter
        ParticleFilterConfig pf_config;

    };

    class Simulator {
    public:
        //  Particle filter
        ParticleFilter pf_estimator;

        // Robot in the world
        Robot beatle;

        // Map of the world
        WorldMap wean;
        cv::Mat wean_visual;
        cv::Mat wean_drawing_copy;

        // Time
        double sim_time;

        // All of the data from the sensors
        std::vector<OdometryData> odom_data;
        std::vector<LaserData> laser_data;


    private:
        // The index of the next odometry or laser data
        unsigned int next_odom_data;
        unsigned int next_laser_data;

    public:
        Simulator() {}
        Simulator(SimulatorConfig& sim_config);
        ~Simulator() {}

        // Interface of the simulator
        // Send sensor data to the particle filter based on the simulation time
        bool oneStepForward();

        // Draw the particles on the image
        // two types of drawing: w/o orientations
        void drawParticles(const std::string& drawing_mode = "loc_dir");

        // Draw laser beams on the image
        void drawLaserBeam();

    private:

    };

}





#endif
