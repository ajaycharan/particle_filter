/***************************************************************
 *
 * Implements the robot simulator
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/17/2014 (MM/DD/YYYY)
 ***************************************************************/

#include "RobotSimulator.h"
#include "Utilities.h"

using namespace std;
using namespace cv;


namespace lab1 {

    Simulator::Simulator(SimulatorConfig& sim_config):pf_estimator(sim_config.pf_config) {

        // Read the map
        Utilities::ReadMap(SimulatorConfig.map_path, wean);
        wean.env_map.convertTo(wean_visual, -1, 0.5f, 0.5f);

        // Read the sensor data
        Utilities::ReadDataLog(SimulatorConfig.data_path, odom_data, laser_data);

        // Set the intial simulation time
        sim_time = 0.0f;

        next_odom_data = 0;
        next_laser_data = 0;

        return;
    }

    /**************************************************************
     * @brief: Interface of the simulator
     *
     *      Everytime this function is called, it moves the
     *      simulation by one time step. The time step is
     *      determined by the time difference between consecutive
     *      data.
     *
     * @param
     * @return : if there is still data available
     **************************************************************/
    bool Simulator::oneStepForward() {

        if (next_odom_data >= odom_data.size() && next_laser_data >= laser_data.size()){
            return false;
        }

        // Check which data should be output at the next moment
        double t_odom = 1e10;
        double t_laser = 1e10;

        if (next_odom_data < odom_data.size)
            t_odom = odom_data[next_odom_data].ts;
        if (next_laser_data < laser_data.size)
            t_laser = laser_data[next_laser_data].ts;

        // Activate the particle filter
        if (t_odom < t_laser) {

            // Output the data from the odometry sensor
            sim_tim = t_odom;
            pf_estimator.estimate(odom_data[next_odom_data]);
            ++next_odom_data;

        } else {

            // Output the data from the laser sensor
            sim_tim = t_laser;
            pf_estimator.estimate(laser_data[next_laser_data]);
            ++next_laser_data;

            if (t_odom == t_laser)
                ++next_odom_data;

        }

        return true;
    }

}
