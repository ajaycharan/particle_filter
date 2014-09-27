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
            particles_old.push_back(Particle());
            particles_old[particles_old.size()-1].x = randu_x(generator);
            particles_old[particles_old.size()-1].y = randu_y(generator);
            particles_old[particles_old.size()-1].theta = randu_theta(generator);
            particles_old[particles_old.size()-1].w = 0.0f;
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
                if (wean.env_map.at<float>(i, j) >= 0.5f) {
                    wean.env_map.at<float>(i, j) = 0.0f;
                } else if(wean.env_map.at<float>(i, j) < 0.5f && wean.env_map.at<float>(i, j) > 0.0f) {
                    wean.env_map.at<float>(i, j) = 1.0f;
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
    void ParticleFilter::motionModel(OdometryData& odom_data){

        // Clear the original pool of particles
        particles_predict.resize(particles_old.size());

        // Create random numbers of normal distribution
        unsigned int rand_seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(rand_seed);
        normal_distribution<float> randn_rot1( 0.0f, alpha1*rot1_stddev + alpha2*trans_stddev);
        normal_distribution<float> randn_trans(0.0f, alpha3*trans_stddev+ alpha4*rot1_stddev + alpha4*rot2_stddev);
        normal_distribution<float> randn_rot2( 0.0f, alpha1*rot2_stddev + alpha2*trans_stddev);

        // For every particle in the pool, generate a new particle
        // based on the input odometry data
        for (unsigned int i = 0; i < particles_old.size(); ++i) {

            // Compute the relative rotation and translation
            float x_diff     = odom_data.x - prev_odom_data.x;
            float y_diff     = odom_data.y - prev_odom_data.y;
            float theta_diff = odom_data.theta - prev_odom_data.theta;
            float rot1  = atan2(y_diff, x_diff);
            float trans = sqrt(x_diff*x_diff + y_diff*y_diff);
            float rot2  = theta_diff - rot1;

            // Add some noise to the relative measurements
            float rot1_noisy  = rot1  + randn_rot1(generator);
            float trans_noisy = trans + randn_trans(generator);
            float rot2_noisy  = rot2  + randn_rot2(generator);

            // Compute the new location
            particles_predict[i].x = particles_old[i].x + cos(particles_old[i].theta+rot1_noisy);
            particles_predict[i].y = particles_old[i].y + sin(particles_old[i].theta+rot1_noisy);
            particles_predict[i].theta = particles_old[i].theta + rot1_noisy + rot2_noisy;
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

        return;
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
        float weight_sum = 0.0f;

        for (unsigned int particle_index = 0; particle_index < particles_predict.size(); ++particle_index){

            // Set the intial weight of the particle to 1;
            float px = particles_predict[particle_index].x;
            float py = particles_predict[particle_index].y;
            float pt = particles_predict[particle_index].theta;
            particles_predict[particle_index].w = 1.0f;

            // Update the weight using the readings from the laser
            for (unsigned int beam_index = 0; beam_index < ldata.size(); ++beam_index) {

                if (ldata[beam_index] < laser_max_reading){

                    // Project the laser reading into the map
                    float obstacle_x = px + laser_x*cos(pt) - laser_y*sin(pt) + ldata[beam_index]*cos(pt+beam_index+0.5f);
                    float obstacle_y = py + laser_x*sin(pt) + laser_y*cos(pt) + ldata[beam_index]*sin(pt+beam_index+0.5f);

                    // Find the closest occupied grid to the "obstacle" position within a certain region
                    float upper_grid = (obstacle_y - valid_range) / (float)wean.resolution;
                    float lower_grid = (obstacle_y + valid_range) / (float)wean.resolution;
                    float left_grid  = (obstacle_x - valid_range) / (float)wean.resolution;
                    float right_grid = (obstacle_x + valid_range) / (float)wean.resolution;

                    int upper = (upper_grid > 0.0f) ? (int)floor(upper_grid) : 0;
                    int lower = (lower_grid < wean.map_size_y/wean.resolution-1.0f) ? (int)ceil(lower_grid) : wean.map_size_y/wean.resolution-1;
                    int left  = (left_grid > 0.0f)  ? (int)floor(left_grid) : 0;
                    int right = (right_grid < wean.map_size_x/wean.resolution-1.0f) ? (int)ceil(right_grid) : wean.map_size_x/wean.resolution-1;

                    int closest_x, closest_y;
                    float shortest_dist_square = 1e10;

                    for (int i = upper; i <= lower; ++i) {
                        for (int j = left; j <= right; ++j) {

                            if (wean.env_map.at<float>(i, j) == 1.0f) {
                                float x_diff = (float)j + 0.5f - obstacle_x;
                                float y_diff = (float)i + 0.5f - obstacle_y;
                                float dist_square = x_diff*x_diff + y_diff*y_diff;
                                if (dist_square < shortest_dist_square) {
                                    closest_x = j;
                                    closest_y = i;
                                    shortest_dist_square = dist_square;
                                }
                            } else {
                                continue;
                            }

                        }
                    }

                    // Compute the probability of the current beam
                    float norm_dist = sqrt(shortest_dist_square)*wean.resolution / dist_stddev;
                    float prob_dist = z_hit*((float)normal_cdf(norm_dist+1)-(float)normal_cdf(norm_dist-1)) + z_random/laser_max_reading;

                    // Accumulate the probability
                    particles_predict[particle_index].w *= prob_dist;

                } else {
                    continue;
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
        float step_size = 1.0f / (float)particle_size;

        // Generate a random number as the starting point
        unsigned int rand_seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(rand_seed);
        uniform_real_distribution<float> randu_r(0.0f, step_size);
        double r = randu_r(generator);

        float c = particles_predict[0].w;
        unsigned int particle_to_sample = 0;

        for (unsigned int i = 0; i < particle_size; ++i) {

            float U = r + (i-1)*step_size;
            while (U > c) {
                ++particle_to_sample;
                c += particles_predict[particle_to_sample].w;
            }

            // Add the sample to represent the belief after resampling
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
        motionModel(laser_data.odom_robot);
        //measurementModel(laser_data.readings);
        //lowVarResample();

        particles_old.clear();
        particles_old = particles_predict;
        return;
    }

}

























