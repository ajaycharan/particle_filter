/***************************************************************
 *
 * Test the implementations of the ParticleFilter class
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

#include "Utilities.h"
#include "ParticleFilter.h"
#include "RobotSimulator.h"

using namespace std;
using namespace cv;
using namespace lab1;


int main (int argc, char *argv[]) {


    /*************************************
     *      Create a simulator
     ************************************/
    namedWindow("Map");
    SimulatorConfig sim_config;
    sim_config.map_path = "/home/ke/cplusplus_ws/particle_filter/data/map/wean.dat";
    sim_config.log_path = "/home/ke/cplusplus_ws/particle_filter/data/log/robotdata1.log";

    // Particle filter configurations
    sim_config.pf_config.max_particle_size = 100;
    sim_config.pf_config.laser_x           = 25.0f;
    sim_config.pf_config.laser_y           = 0.0f;
    sim_config.pf_config.laser_theta       = 0.0f;
    sim_config.pf_config.laser_max_reading = 5000.0f;
    sim_config.pf_config.rot1_stddev       = 0.08f;
    sim_config.pf_config.rot2_stddev       = 0.08f;
    sim_config.pf_config.trans_stddev      = 10.0f;
    sim_config.pf_config.alpha1            = 0.5f;
    sim_config.pf_config.alpha2            = 0.5f;
    sim_config.pf_config.alpha3            = 0.5f;
    sim_config.pf_config.alpha4            = 0.5f;
    sim_config.pf_config.dist_stddev       = 10.0f;
    sim_config.pf_config.z_hit             = 0.8f;
    sim_config.pf_config.z_random          = 0.2f;

    Simulator pf_sim(sim_config);



    return 1;
}








