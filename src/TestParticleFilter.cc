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
#include <iomanip>
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

    namedWindow("Particle Filter");

    /*************************************
     *      Create a simulator
     ************************************/
    SimulatorConfig sim_config;
    sim_config.map_path = "/home/ke/cplusplus_ws/particle_filter/data/map/wean.dat";
    sim_config.log_path = "/home/ke/cplusplus_ws/particle_filter/data/log/robotdata1.log";

    // Particle filter configurations
    sim_config.pf_config.max_particle_size = 1000;
    sim_config.pf_config.laser_x           = 25.0f;
    sim_config.pf_config.laser_y           = 0.0f;
    sim_config.pf_config.laser_theta       = 0.0f;
    sim_config.pf_config.laser_max_reading = 5000.0f;
    sim_config.pf_config.rot1_stddev       = 0.04f;
    sim_config.pf_config.rot2_stddev       = 0.04f;
    sim_config.pf_config.trans_stddev      = 1.0f;
    sim_config.pf_config.alpha1            = 1.0f;
    sim_config.pf_config.alpha2            = 0.0f;
    sim_config.pf_config.alpha3            = 1.0f;
    sim_config.pf_config.alpha4            = 0.0f;
    sim_config.pf_config.dist_stddev       = 10.0f;
    sim_config.pf_config.z_hit             = 0.9f;
    sim_config.pf_config.z_random          = 0.1f;


    /**********************************
     *      Start the simulator
     *********************************/
    Simulator pf_sim(sim_config);
    bool continue_flag = true;
    printf("Sim time: %.4f\n", 0.0f);

    while (continue_flag) {
        // Forward the simulation by one step
        continue_flag = pf_sim.oneStepForward();
        // Show the particles and beams
        imshow("Particle Filter", pf_sim.wean_drawing_copy);
        waitKey(0);
        printf("Sim time: %.4f\n", pf_sim.sim_time);
    }
    printf("\n");

    destroyWindow("Particle Filter");

    return 1;
}








