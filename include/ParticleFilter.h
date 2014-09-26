/***************************************************************
 *
 * Implements the particle filter
 *
 *
 *
 * Author: Ke Sun
 * Date  : 09/21/2014 (MM/DD/YYYY)
 ***************************************************************/

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <string>
#include <random>
#include <opencv2/opencv.hpp>

namespace lab1 {

    struct OdometryData {
        // Coordinates of the robot in standard odometry frame
        float x;
        float y;
        float theta;
        // Time stamp of the odometry reading
        double ts;
    };

    struct LaserData {
        // Coordinates of the robot in standard odometry frame
        OdometryData odom_robot;
        // Coordinates of the laser sensor in the standard odometry frame
        OdometryData odom_laser;
        // Laser readings in the couterclockwise order
        std::vector<float> readings;
        // Time stamp
        double ts;
    };

    struct WorldMap {
        // Size of the map
        int map_size_x;
        int map_size_y;
        // Resolution of the map
        int resolution;
        // What is this?????????????
        int auto_shifted_x;
        int auto_shifted_y;
        // Map
        // Note that the memory of the map should be dynamically allocated
        cv::Mat env_map;
    };

    struct Particle {
        // Pose of the particle instance
        float x;
        float y;
        float theta;
        // Weight of the particle
        float w;
    };

    struct ParticleFilterConfig {
        // Map of the world
        WorldMap wean;

        // Number of the particles
        int max_particle_size;

        // Position of the laser in the body frame of the robot
        float laser_x;
        float laser_y;
        float laser_theta;
        float laser_max_reading;

        // Parameters of the motions model
        float rot1_stddev;
        float rot2_stddev;
        float trans_stddev;

        float alpha1;
        float alpha2;
        float alpha3;
        float alpha4;

        // Parameters of the measurement model
        float dist_stddev;
        float z_hit;
        float z_random;

        // TODO: include other parameters
    };

    class ParticleFilter {

    public:
        // Particles
        std::vector<Particle> particles_old;
        std::vector<Particle> particles_predict;
        std::vector<Particle> particles_update;

        // Map of the world
        WorldMap wean;

        // Constructor
        ParticleFilter(ParticleFilterConfig& pf_config);

        // Destructor
        ~ParticleFilter() {}

        // Interface of the particle filter
        // Note that the interface is overloaded to handle the case where only
        // the input is available
        void estimation(OdometryData& odom_data);
        void estimation(OdometryData& odom_data, LaserData& laser_data);

    private:

        // Maximum number of particles
        int max_particle_size;

        // Position of the laser in the body frame of the robot
        float laser_x;
        float laser_y;
        float laser_theta;
        float laser_max_reading;

        // Parameters of the motions model
        float rot1_stddev;
        float rot2_stddev;
        float trans_stddev;

        float alpha1;
        float alpha2;
        float alpha3;
        float alpha4;

        // Parameters of the measurement model
        float dist_stddev;
        float valid_range;
        float z_hit;
        float z_random;

        // Previous odometry data
        OdometryData prev_odom_data;

        // Compute the cdf of normal distribution
        double normal_cdf(double d);
        // Categorize the map
        void mapCategorize();
        // Create new particles based on the control input (odom data)
        void motionModel(OdometryData& odom_data);
        // Compute the weight for each new sample
        void measurementModel(std::vector<float>& ldata);
        // Create new samples according to the weights
        void lowVarResample();


    };
}



#endif
