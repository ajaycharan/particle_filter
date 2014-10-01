/***************************************************************
 *
 * Implements the particle filter
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/21/2014 (MM/DD/YYYY)
 ***************************************************************/


#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "ParticleFilter.h"

using namespace std;
using namespace cv;

#define PI  3.14159265358979f
#define DEGREE2RAD 0.0174533f
#define A1  0.31938153f
#define A2 -0.356563782f
#define A3  1.781477937f
#define A4 -1.821255978f
#define A5  1.330274429f
#define RSQRT2PI 0.39894228040143267793994605993438f
#define PF_DEBUG

namespace lab1 {

    /**************************************************************
     * @brief: Constructor of the ParticleFilter class
     *
     *      Create a new instance of the particle filter algorithm
     *      using the given configurations.
     *
     * @param  pf_config: configurations for the particle filter
     * @return Nil
     **************************************************************/
    ParticleFilter::ParticleFilter(ParticleFilterConfig& pf_config) {

        // Set the maximum number of particles
        max_particle_size = pf_config.max_particle_size;

        // Position of the laser in the body frame
        laser_x = pf_config.laser_x;
        laser_y = pf_config.laser_y;
        laser_theta = pf_config.laser_theta;
        laser_max_reading = pf_config.laser_max_reading;

        // Empty the pools of the particles
        particles_old.resize(0);
        particles_predict.resize(0);
        particles_update.resize(0);

        // Initialize the parameters of the motion model
        rot1_stddev  = pf_config.rot1_stddev;
        rot2_stddev  = pf_config.rot2_stddev;
        trans_stddev = pf_config.trans_stddev;
        alpha1       = pf_config.alpha1;
        alpha2       = pf_config.alpha2;
        alpha3       = pf_config.alpha3;
        alpha4       = pf_config.alpha4;

        rot1_stddev_hat  = sqrt(alpha1*rot1_stddev*rot1_stddev + alpha2*trans_stddev*trans_stddev);
        rot2_stddev_hat  = sqrt(alpha1*rot2_stddev*rot2_stddev + alpha2*trans_stddev*trans_stddev);
        trans_stddev_hat = sqrt(alpha3*trans_stddev*trans_stddev + alpha4*rot1_stddev*rot1_stddev + alpha4*rot2_stddev*rot2_stddev);

        // Initialize the parameters of the measurement model
        dist_stddev = pf_config.dist_stddev;
        valid_range = 2.5f * dist_stddev;
        z_hit       = pf_config.z_hit;
        z_random    = pf_config.z_random;

        first_odom_data = true;

        // TODO: Initialize other parameters of the filter

        return;
    }

    /**************************************************************
     * @brief: Set the map of the particle filter
     *
     *
     * @param new_map: a new map
     * @return
     **************************************************************/
    void ParticleFilter::setMap(WorldMap& new_map) {

        // Initialize the map
        wean.map_size_x = new_map.map_size_x;
        wean.map_size_y = new_map.map_size_y;
        wean.resolution = new_map.resolution;
        wean.auto_shifted_x = new_map.auto_shifted_x;
        wean.auto_shifted_y = new_map.auto_shifted_y;
        new_map.env_map.copyTo(wean.env_map);

#ifdef PF_DEBUG
            cout << "Map size: " << wean.map_size_x << " " << wean.map_size_y << endl;
            cout << "Resolution: " << wean.resolution << endl;
#endif
        // Categorize the grids in the map
        mapCategorize();

        return;
    }

    /**************************************************************
     * @brief: Generate initial particles
     *
     *      The intial particles distributed uniformly on the map.
     *      Note that this function can only be called after
     *      function setMap is called.
     *
     * @param
     * @return
     **************************************************************/
    void ParticleFilter::generateInitParticles() {

        // Create particles that uniformly distributed in the map
        unsigned int rand_seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(rand_seed);
        uniform_real_distribution<float> randu_x(0.0f, wean.map_size_x-1);
        uniform_real_distribution<float> randu_y(0.0f, wean.map_size_y-1);
        uniform_real_distribution<float> randu_theta(-PI, PI);

        for (int i = 0; i < max_particle_size; ++i) {
            // Add a new particle
            particles_old.push_back(Particle());

            // Ensure the particle is on the unoccupied or
            // unknown location in the map
            float x = randu_x(generator);
            float y = randu_y(generator);

            while (wean.env_map.at<float>((int)(y/wean.resolution), (int)(x/wean.resolution)) != 0.0f) {
                x = randu_x(generator);
                y = randu_y(generator);
            }

            particles_old[particles_old.size()-1].x     = x;
            particles_old[particles_old.size()-1].y     = y;
            particles_old[particles_old.size()-1].theta = randu_theta(generator);
            particles_old[particles_old.size()-1].w     = 0.0f;
        }

        return;
    }


    /**************************************************************
     * @brief: Compute the cdf of a standard normal distribution
     *
     *
     * @param d: value up to which the pdf is integrated
     * @return : the probability
     **************************************************************/
    double ParticleFilter::normal_cdf(double d)
    {
        double K = 1.0 / (1.0 + 0.2316419 * fabs(d));
        double cnd = RSQRT2PI * exp(- 0.5 * d * d) * (K * (A1 + K * (A2 + K * (A3 + K * (A4 + K * A5)))));

        if (d > 0)
            cnd = 1.0 - cnd;

        return cnd;
    }
    /**************************************************************
     * @brief: Categorize the map
     *
     *      Categorize the map into three categories: occupied,
     *      free and unknown based on the probability of occupancy
     *
     * @param  Nil
     * @return Nil
     **************************************************************/
    void ParticleFilter::mapCategorize() {
        for (unsigned int i = 0; i < wean.map_size_x/wean.resolution; ++i) {
            for (unsigned int j = 0; j < wean.map_size_y/wean.resolution; ++j) {
                // Set the grid to
                // occupied: >= 0.5
                // free    : < 0.5
                // unknown : -1
                if (wean.env_map.at<float>(i, j) >= 0.9f) {
                    wean.env_map.at<float>(i, j) = 0.0f;
                } else if(wean.env_map.at<float>(i, j) < 0.05f && wean.env_map.at<float>(i, j) >= 0.0f) {
                    wean.env_map.at<float>(i, j) = 1.0f;
                } else {
                    wean.env_map.at<float>(i, j) = -1.0f;
                }
            }
        }
        return;
    }

    /**************************************************************
     * @brief: Motion model of the particle filter
     *
     *      The function implements an odometry motion model to
     *      generate new samples using the given input data as
     *      the prediction.
     *
     * @param  odom_data: readings from the odometry
     * @return Nil
     **************************************************************/
    bool ParticleFilter::motionModel(OdometryData& odom_data){

        bool if_resample = true;

        // Clear the original pool of particles
        particles_predict.resize(particles_old.size());

        // Create random numbers of normal distribution
        double dt = odom_data.ts - prev_odom_data.ts;
        unsigned int rand_seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(rand_seed);

        normal_distribution<float> randn_rot1( 0.0f, rot1_stddev_hat*(float)dt);
        normal_distribution<float> randn_trans(0.0f, trans_stddev_hat*(float)dt);
        normal_distribution<float> randn_rot2( 0.0f, rot2_stddev_hat*(float)dt);

        uniform_real_distribution<float> randu_x(0.0f, wean.map_size_x-1);
        uniform_real_distribution<float> randu_y(0.0f, wean.map_size_y-1);
        uniform_real_distribution<float> randu_theta(-PI, PI);

        // Compute the relative rotation and translation
        float x_diff     = odom_data.x - prev_odom_data.x;
        float y_diff     = odom_data.y - prev_odom_data.y;
        float theta_diff = odom_data.theta - prev_odom_data.theta;

        if (fabsf(x_diff) < 1.0f && fabsf(y_diff) < 1.0f && fabsf(theta_diff) < 0.02f){
            if_resample = false;
        }

        // For every particle in the pool, generate a new particle
        // based on the input odometry data
        for (unsigned int i = 0; i < particles_old.size(); ++i) {

            // Count the trial rounds
            int trial_cntr = 1;

            float rot1  = atan2(y_diff, x_diff) - prev_odom_data.theta;
            float trans = sqrt(x_diff*x_diff + y_diff*y_diff);
            float rot2  = theta_diff - rot1;

            // Add some noise to the relative measurements
            float rot1_noisy  = rot1  + randn_rot1(generator);
            float trans_noisy = trans + randn_trans(generator);
            float rot2_noisy  = rot2  + randn_rot2(generator);

            // Compute the new location
            // Ensure the particle is on the unoccupied or
            // unknown location in the map
            float x = particles_old[i].x + trans_noisy*cos(particles_old[i].theta+rot1_noisy);
            float y = particles_old[i].y + trans_noisy*sin(particles_old[i].theta+rot1_noisy);

            while (wean.env_map.at<float>((int)(y/wean.resolution), (int)(x/wean.resolution)) != 0.0f && trial_cntr < 20) {

                rot1_noisy  = rot1  + randn_rot1(generator);
                trans_noisy = trans + randn_trans(generator);
                rot2_noisy  = rot2  + randn_rot2(generator);

                x = particles_old[i].x + trans_noisy*cos(particles_old[i].theta+rot1_noisy);
                y = particles_old[i].y + trans_noisy*sin(particles_old[i].theta+rot1_noisy);

                ++trial_cntr;
            }

            if (trial_cntr >= 20) {
                x = randu_x(generator);
                y = randu_y(generator);

                while (wean.env_map.at<float>((int)(y/wean.resolution), (int)(x/wean.resolution)) != 0.0f) {
                    x = randu_x(generator);
                    y = randu_y(generator);
                }
            }

            particles_predict[i].x = x;
            particles_predict[i].y = y;
            if (trial_cntr < 20){
                particles_predict[i].theta = particles_old[i].theta + rot1_noisy + rot2_noisy;
            } else {
                particles_predict[i].theta = randu_theta(generator);
            }
        }

        //Delete the particles which are out of the map
        unsigned particle_index = 0;
        while (particle_index < particles_predict.size()) {

            if (particles_predict[particle_index].x < 0 || particles_predict[particle_index].x >= wean.map_size_x ||
                particles_predict[particle_index].y < 0 || particles_predict[particle_index].y >= wean.map_size_y) {
                vector<Particle>::iterator iter = particles_predict.begin() + particle_index;
                particles_predict.erase(iter);
            } else {
                ++particle_index;
            }

        }
        // Update the previous data
        prev_odom_data = odom_data;

        return if_resample;
    }

    /**************************************************************
     * @brief: Measurement model of the particle filter
     *
     *      The function implements likelihood fields to model the
     *      readings from the laser. Weight of each particle is
     *      computed in this function
     *
     * @param  odom_data: readings from the odometry
     * @return Nil
     **************************************************************/
    void ParticleFilter::measurementModel(vector<float>& ldata) {

        // Store the sum of weights of the particles
        double weight_sum = 0.0f;

        for (unsigned int particle_index = 0; particle_index < particles_predict.size(); ++particle_index){

            // Set the intial weight of the particle to 1;
            float px = particles_predict[particle_index].x;
            float py = particles_predict[particle_index].y;
            float pt = particles_predict[particle_index].theta;
            float cospt = cos(pt);
            float sinpt = sin(pt);
            particles_predict[particle_index].w = 0.0f;

            // Update the weight using the readings from the laser
            for (unsigned int beam_index = 9; beam_index < ldata.size(); beam_index+=18) {

                if (ldata[beam_index] < laser_max_reading){

                    // Create some sub-beams within the cone of current beam
                    float bt = (float)beam_index - 90.0f;
                    //vector<float> sub_beams(3);
                    //sub_beams[0] = bt + 0.1667f;
                    //sub_beams[1] = bt + 0.5000f;
                    //sub_beams[2] = bt + 0.8333f;
                    vector<float> sub_beams(1);
                    sub_beams[0] = bt + 0.5f;

                    // Find the closest obstacle in the directoin of any sub-beam
                    float obstacle_x, obstacle_y;
                    float shortest_dist = 1e10;

                    for (unsigned int sub_beam_index = 0; sub_beam_index < sub_beams.size(); ++sub_beam_index) {

                        // Mark the nearest obstacle has been met in this sub-directoin
                        bool hit_nearest = false;
                        bool out_range   = false;

                        for (float probe_dist = 10.0f; probe_dist <= laser_max_reading; probe_dist+=10.0f) {

                            float probe_x = px + laser_x*cospt - laser_y*sinpt + probe_dist*cos(pt+sub_beams[sub_beam_index]*DEGREE2RAD);
                            float probe_y = py + laser_x*sinpt + laser_y*cospt + probe_dist*sin(pt+sub_beams[sub_beam_index]*DEGREE2RAD);

                            int probe_grid_x = (int)(probe_x/wean.resolution+0.5f);
                            int probe_grid_y = (int)(probe_y/wean.resolution+0.5f);

                            if (probe_grid_x >= 0 && probe_grid_x < wean.map_size_x/wean.resolution &&
                                probe_grid_y >= 0 && probe_grid_y < wean.map_size_y/wean.resolution) {
                                if (wean.env_map.at<float>(probe_grid_y, probe_grid_x) == 1.0f) {
                                    hit_nearest = true;
                                    if (probe_dist < shortest_dist){
                                        obstacle_x = probe_x;
                                        obstacle_y = probe_y;
                                        shortest_dist = probe_dist;
                                    }
                                    break;
                                }
                            } else {
                                out_range = true;
                                break;
                            }
                        }
                        // Proceed to the next sub-beam
                        if (hit_nearest || out_range) continue;
                    }

                    // Project the laser reading into the map
                    float beam_x = px + laser_x*cospt - laser_y*sinpt + ldata[beam_index]*sin(pt+((float)beam_index+0.5f)*DEGREE2RAD);
                    float beam_y = py + laser_x*sinpt + laser_y*cospt - ldata[beam_index]*cos(pt+((float)beam_index+0.5f)*DEGREE2RAD);

                    // Compute the distance between the measurement of the laser beam
                    // with the neareat obstacle in that direction
                    float dist_obstacle_laser = (obstacle_x-beam_x)*(obstacle_x-beam_x)+(obstacle_y-beam_y)*(obstacle_y-beam_y);

                    // Compute the probability of the current beam
                    double norm_dist = sqrt(dist_obstacle_laser) / dist_stddev;
                    double prob_dist = (double)z_hit*(normal_cdf(norm_dist+0.1f)-normal_cdf(norm_dist-0.1f)) + (double)(z_random/laser_max_reading);
                    particles_predict[particle_index].w += prob_dist;

                }
            }

            // Accumulate the weight for later normalization
            weight_sum += particles_predict[particle_index].w;
        }

        // Normalize the weight of all the particles
        for (unsigned int i = 0; i < particles_predict.size(); ++i) {
            particles_predict[i].w /= weight_sum;
        }

        return;
    }


    /**************************************************************
     * @brief: Resample process of the particle fitler
     *
     *      The function implements a low variance resampler to
     *      reduce the sampling error of the particle filter
     *
     * @param  Nil
     * @return Nil
     **************************************************************/
    void ParticleFilter::lowVarResample() {

        particles_update.clear();

        unsigned int particle_size = particles_predict.size();
        double step_size = 1.0f / (double)particle_size;

        // Generate a random number as the starting point
        unsigned int rand_seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(rand_seed);
        uniform_real_distribution<double> randu_r(0.0f, step_size);
        double r = randu_r(generator);

        double c = particles_predict[0].w;
        unsigned int particle_to_sample = 0;

        for (unsigned int i = 0; i < particle_size; ++i) {

            double U = r + (double)i*step_size;
            while (U > c) {
                ++particle_to_sample;
                if (particle_to_sample >= particle_size) break;
                c += particles_predict[particle_to_sample].w;
            }

            // Add the sample to represent the belief after resampling
            if (particle_to_sample >= particle_size) break;
            particles_update.push_back(particles_predict[particle_to_sample]);

        }

        return;
    }


    /**************************************************************
     * @brief: Interface of the particle filter
     *
     *      This interface takes the data from the odometry only,
     *      and use it as input, which handles the case that sensor
     *      readings from laser is not available.
     *
     * @param odom_data: readings from the odometry sensors
     * @return Nil
     **************************************************************/
    void ParticleFilter::estimate(OdometryData& odom_data){

        if (first_odom_data) {
            first_odom_data = false;
            prev_odom_data = odom_data;
            return;
        }

        // Since there is no measurement data,
        // only process model is performed.
        motionModel(odom_data);

        particles_old.clear();
        particles_old = particles_predict;
        return;
    }


    /**************************************************************
     * @brief: Interface of the particle filter
     *
     *      This interface takes both the odometry and laser data.
     *      Note that since the odometry data has been interpolated
     *      in the log file, the odometry data is always available
     *      when there is laser data.
     *
     * @param laser_data: odometry data and readings from the laser
     * @return Nil
     **************************************************************/
    void ParticleFilter::estimate(LaserData& laser_data) {

        if (first_odom_data) {
            first_odom_data = false;
            prev_odom_data = laser_data.odom_robot;
            return;
        }

        // A full pipeline of the particle is implemented
        bool if_resample = motionModel(laser_data.odom_robot);
        if (if_resample) {
            measurementModel(laser_data.readings);
            lowVarResample();
            particles_old.clear();
            particles_old = particles_update;
        } else {
            particles_old.clear();
            particles_old = particles_predict;
        }

        return;
    }

}
