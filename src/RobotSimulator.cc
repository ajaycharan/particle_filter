/***************************************************************
 *
 * Implements the robot simulator
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/17/2014 (MM/DD/YYYY)
 ***************************************************************/

#include <cmath>
#include "RobotSimulator.h"
#include "Utilities.h"

#define DEGREE2RAD 0.0174533f
#define RSIM_DEBUG

using namespace std;
using namespace cv;


namespace lab1 {

    Simulator::Simulator(SimulatorConfig& sim_config):pf_estimator(sim_config.pf_config) {

        // Read the map
        Utilities::ReadMap(sim_config.map_path, wean);
        //flip(wean.env_map, wean.env_map, 0);

        // Some other settings of the particle filter
        pf_estimator.setMap(wean);
        pf_estimator.generateParticles(-1);

        //wean.env_map.copyTo(wean_visual);
        pf_estimator.wean.env_map.copyTo(wean_visual);
        wean_visual.convertTo(wean_visual, -1, 0.5f, 0.5f);
        cvtColor(wean_visual, wean_visual, CV_GRAY2RGB);
        wean_visual.copyTo(wean_drawing_copy);

        // Read the sensor data
        Utilities::ReadDataLog(sim_config.log_path, odom_data, laser_data);

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
     * @return : 0 --> no more sensor data
     *           1 --> read odometry data
     *           2 --> read laser data
     **************************************************************/
    int Simulator::oneStepForward() {

        if (next_odom_data >= odom_data.size() && next_laser_data >= laser_data.size()){
            return 0;
        }

        // Check which data should be output at the next moment
        double t_odom = 1e10;
        double t_laser = 1e10;

        if (next_odom_data < odom_data.size())
            t_odom = odom_data[next_odom_data].ts;
        if (next_laser_data < laser_data.size())
            t_laser = laser_data[next_laser_data].ts;

        // Activate the particle filter
        if (t_odom < t_laser) {

            // Output the data from the odometry sensor
            sim_time = t_odom;
            pf_estimator.estimate(odom_data[next_odom_data]);
            ++next_odom_data;

#ifdef RSIM_DEBUG
            // Draw the particles
            wean_visual.copyTo(wean_drawing_copy);
            drawParticles("loc");
#endif
            return 1;

        } else {

            // Output the data from the laser sensor
            sim_time = t_laser;
            pf_estimator.estimate(laser_data[next_laser_data]);
            ++next_laser_data;

#ifdef RSIM_DEBUG
            // Draw the particles
            // Draw the laser measurement for a single particle
            wean_visual.copyTo(wean_drawing_copy);
            drawParticles("loc");
            if (pf_estimator.particles_update.size() > 0) {

                unsigned int lucky_pt = 0;
                for (unsigned int i = 1; i <pf_estimator.particles_update.size(); ++i) {
                    lucky_pt = pf_estimator.particles_update[i].w > pf_estimator.particles_update[lucky_pt].w ? i : lucky_pt;
                }

                drawLaserBeam(pf_estimator.particles_update[lucky_pt], laser_data[next_laser_data-1]);
            }
#endif

            if (t_odom == t_laser)
                ++next_odom_data;

            return 2;

        }

    }

    /**************************************************************
     * @brief: draw particles on the map image
     *
     *      The function provides two drawing modes: w/o
     *      direction of the robot
     *
     * @param
     * @return
     **************************************************************/
    void Simulator::drawParticles(const string& drawing_mode) {

        // For all particle, compute their locations and orientations
        // in the image
        vector<Particle>& particles = pf_estimator.particles_update;
        vector<Point> p_loc(particles.size());
        vector<Point> p_dir(particles.size());

        for (unsigned int i = 0; i < particles.size(); ++i) {

            p_loc[i].x = (int)floor(particles[i].x/wean.resolution+0.5f);
            p_loc[i].y = (int)floor(particles[i].y/wean.resolution+0.5f);

            float cost = cos(particles[i].theta);
            float sint = sin(particles[i].theta);
            p_dir[i].x = (int)floor((cost*50.0f + particles[i].x)/wean.resolution+0.5f);
            p_dir[i].y = (int)floor((sint*50.0f + particles[i].y)/wean.resolution+0.5f);

        }

        if (drawing_mode.compare("loc") == 0) {

            for (unsigned int i = 0; i < p_loc.size(); ++i) {
                circle(wean_drawing_copy, p_loc[i], 2, CV_RGB(1.0f, 0.0f, 0.0f), -1);
            }

        } else if (drawing_mode.compare("loc_dir") == 0) {

            for (unsigned int i = 0; i < p_loc.size(); ++i) {
                circle(wean_drawing_copy, p_loc[i], 5, CV_RGB(1.0f, 0.0f, 0.0f));
                line(wean_drawing_copy, p_loc[i], p_dir[i], CV_RGB(1.0f, 0.0f, 0.0f));
                if (i == p_loc.size()/2) {
                    circle(wean_drawing_copy, p_loc[i], 5, CV_RGB(0.0f, 1.0f, 0.0f));
                    line(wean_drawing_copy, p_loc[i], p_dir[i], CV_RGB(0.0f, 1.0f, 0.0f));
                }
            }

        } else {
            cerr << "Unrecognized drawing mode: " << drawing_mode << endl;
        }

        return;
    }

    /**************************************************************
     * @brief: draw the laser beams of a particle
     *
     *      Visual the laser beams orginated from a single
     *      particle for debugging.
     *
     * @param
     * @return
     **************************************************************/
    void Simulator::drawLaserBeam(Particle& pt, LaserData& laser_data) {

        // Body frame of the robot
        float cx = pt.x;
        float cy = pt.y;
        float sint = sin(pt.theta);
        float cost = cos(pt.theta);

        // Mark the origin of the laser beam in the world frame
        // Note that the postion of the origin of the laser beams
        // in the body frame is hardcoded for convinent purpose.
        float lx = cost*25.0f + cx;
        float ly = sint*25.0f + cy;

        int lx_grid = (int)(lx/wean.resolution);
        int ly_grid = (int)(ly/wean.resolution);

        for (unsigned int i = 0; i < laser_data.readings.size(); ++i){

            float curr_beam = laser_data.readings[i];
            float curr_th = ((float)i-90.0f+0.5f)*DEGREE2RAD;
            float beamx_body = curr_beam*cos(curr_th)+25.0f;
            float beamy_body = curr_beam*sin(curr_th);
            float beamx_world = cost*beamx_body - sint*beamy_body + cx;
            float beamy_world = sint*beamx_body + cost*beamy_body + cy;

            int beamx_grid = (int)(beamx_world/wean.resolution);
            int beamy_grid = (int)(beamy_world/wean.resolution);

            line(wean_drawing_copy, Point(lx_grid, ly_grid), Point(beamx_grid, beamy_grid), CV_RGB(0.0f, 0.0f, 1.0f));

        }
        return;
    }

}
