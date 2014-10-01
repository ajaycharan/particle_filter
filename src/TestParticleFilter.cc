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
    sim_config.pf_config.max_particle_size = 30000;
    sim_config.pf_config.laser_x           = 25.0f;
    sim_config.pf_config.laser_y           = 0.0f;
    sim_config.pf_config.laser_theta       = 0.0f;
    sim_config.pf_config.laser_max_reading = 4000.0f;
    sim_config.pf_config.rot1_stddev       = 0.1f;
    sim_config.pf_config.rot2_stddev       = 0.1f;
    sim_config.pf_config.trans_stddev      = 20.0f;
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
    int data_flag = 1;

    while (data_flag) {
        // Forward the simulation by one step
        data_flag = pf_sim.oneStepForward();
        printf("Sim time: %.4f\n", pf_sim.sim_time);

        // Show the particles and beams

        if (data_flag == 2) {
            imshow("Particle Filter", pf_sim.wean_drawing_copy);
            waitKey(5);
        }
    }
    printf("\n");

    destroyWindow("Particle Filter");

    return 1;
}








