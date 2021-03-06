//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file:          particle-filter.cc
\brief:         Particle Filter for CS393r Autonomous Robots
\university:    The University of Texas at Austin
\class:         CS 393r Autonomous Robots
\assignment:    Assignment 2 - Particle Filter
\author:        Mary Tebben & Frank Regal
\adopted from:  Dr. Joydeep Biswas
*/
//========================================================================




#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "config_reader/config_reader.h"
#include "particle_filter.h"
#include "vector_map/vector_map.h"
#include <math.h>

#define _USE_MATH_DEFINES

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace {
  int predict_steps {0};
  double max_particle_weight {0};
  int updates_done {0};
  double distance_moved_over_predict {0};
}

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    odom_old_pos(0,0),
    odom_old_angle {0},
    odom_initialized_(false),
    predict_step_done_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) 
{
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Setting Up Output Vector
  vector<Vector2f>& scan = *scan_ptr;

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  
  // Initialize Max Distance Point Vector
  Eigen::Vector2f endpoint_of_max_distance_ray;

  // Step Size of Scan; set_parameter
  int step_size_of_scan {110}; // 75 best guess

  // Reduce the input vectors size to account for every 10th laser scan ray
  int length_of_scan_vec = num_ranges/step_size_of_scan;

  // Make Scan vector the size of input vector
  scan.resize(length_of_scan_vec);

  // Step 1: Predict Beginning and End Points of Each Ray within Theoretical Laser Scan
  // Points for Laser Scanner in Space using Distance between baselink and laser scanner on physical robot to be 0.2 meters
  float laser_scanner_loc_x = loc.x() + 0.2*cos(angle);
  float laser_scanner_loc_y = loc.y() + 0.2*sin(angle);
  Eigen::Vector2f laser_scanner_loc(laser_scanner_loc_x,laser_scanner_loc_y);

  // Starting Angle based on the orientation of the particle (angle)
  float parsed_angle = angle+angle_min;

  // Build Vector of Laser Scan Angles
  std::vector <float> predicted_scan_angles;

  // Step 2: Fill a Vector of each Scan Angle
  for (int i {0}; i < length_of_scan_vec; i++)
  {
    parsed_angle += ((angle_max-angle_min)/length_of_scan_vec);
    predicted_scan_angles.push_back(parsed_angle);
  }

  // Step 3: loop through each theoretical scan
  for (int j {0}; j < length_of_scan_vec; j++)
  {
    // Initialize the points of the predicted laser scan rays
    line2f laser_ray(1,2,3,4);
    laser_ray.p0.x() = laser_scanner_loc_x + range_min*cos(predicted_scan_angles[j]);
    laser_ray.p0.y() = laser_scanner_loc_y + range_min*sin(predicted_scan_angles[j]);
    laser_ray.p1.x() = laser_scanner_loc_x + range_max*cos(predicted_scan_angles[j]);
    laser_ray.p1.y() = laser_scanner_loc_y + range_max*sin(predicted_scan_angles[j]);
    
    // Fill i-th entry to return vector (scan) with each point of predicted laser scan of the max range
    scan[j] << laser_ray.p1.x(),
               laser_ray.p1.y();
    
    // Calculate the max distance of the current ray
    double max_distance_of_current_ray = (laser_scanner_loc - scan[j]).norm();

    // Loop Through Each Line from Imported Map Text File to See this Single Laser Ray
    // Intersects at the angle of predicted_scan_angles
    for (size_t k = 0; k < map_.lines.size(); ++k) 
    {
      // Assign Map Lines to Variable for Interestion Calculations
      const line2f map_line = map_.lines[k];
      
      // Initialize Return Variable of the Location where the Ray Intersects
      Eigen::Vector2f intersection_point;

      // Compare Map Text File Lines to Theoretical Laser Rays from Each Particle
      bool intersects = map_line.Intersection(laser_ray, &intersection_point);
      
      if (intersects) // is true
      {
        // Record Distance of that Intersecting Ray
        double distance_of_intersecting_ray = (intersection_point-laser_scanner_loc).norm();
        // If the Intersecting Distance Recorded above is less than of all of the previously recorded intersections for this ray at specific angele
        if ( distance_of_intersecting_ray < max_distance_of_current_ray)
        {
          // Set Distance Variable for Next Comparison
          max_distance_of_current_ray = distance_of_intersecting_ray;
          // Record the Endpoint of Where that Ray Intersects
          endpoint_of_max_distance_ray = intersection_point;
        }
      }
    }
    // Fill Output Vector of this Ray with the point at where the shortest distance ray intersects with the map
    scan[j] = endpoint_of_max_distance_ray; 
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {

  // Setting Up Output Variable
  Particle& particle = *p_ptr;

  // Initialize point cloud vector to fill from GetPredictedPointCloud
  vector<Vector2f> predicted_point_cloud;

  // Fill point cloud vector
  GetPredictedPointCloud(particle.loc, particle.angle, 
                         ranges.size(), range_min, range_max, 
                         angle_min, angle_max, &predicted_point_cloud);

  // Resize lidar scan range (Number of Rays)
  vector<float> resized_ranges(predicted_point_cloud.size());
  int lidar_ray_step_size = ranges.size() / predicted_point_cloud.size();

  // Calculating the Size of Predicted Point Cloud Length
  int predicted_point_cloud_length = predicted_point_cloud.size();

  // Resizing the Actual Laser Scan Ranges
  for(int i = 0; i < predicted_point_cloud_length; i++)
    resized_ranges[i] = ranges[lidar_ray_step_size*i];

  // Tuning parameters for the minimum and maximum distances of the laser scanner; set_parameter
  double dshort = 0.5;
  double dlong = 0.5;
  double gamma = 0.8;

  // Standard deviation of Physical LIDAR System; set_parameter
  double ray_std_dev = 0.15;

  // Reset variables between each input point cloud
  float weight {0};
  float total_weight {0};

  // Calculate Weight for Particle
  for(int i = 0; i < predicted_point_cloud_length; i++)
  {
    // Physical Laser Scanner Location is Offset From the Particle Location
    float laser_scanner_loc_x = particle.loc.x() + 0.2*cos(particle.angle);
    float laser_scanner_loc_y = particle.loc.y() + 0.2*sin(particle.angle);

    // Distance laser scanner actually found from obstacle (ACTUAL)
    float particle_actual_distance = resized_ranges[i];

    // Calculate distance between the laser scanner and the returned point cloud location (THEORETICAL)
    float theoretical_dist_x = predicted_point_cloud[i].x() - laser_scanner_loc_x;
    float theoretical_dist_y = predicted_point_cloud[i].y() - laser_scanner_loc_y;
    float particle_theoretical_distance = sqrt(pow(theoretical_dist_x, 2)+pow(theoretical_dist_y, 2));

    // Calculating Distance Comparison
    float delta_distance = particle_actual_distance - particle_theoretical_distance;

    // (MAIN WEIGHT FUNCTION) Function for weighting the probability of each particle being in the location found
    if(particle_actual_distance < range_min or particle_actual_distance > range_max)
      continue;
    else if (particle_actual_distance < (particle_theoretical_distance - dshort))
      weight = exp(-(pow(dshort,2) / pow(ray_std_dev,2)));
    else if (particle_actual_distance > (particle_theoretical_distance + dlong))
      weight = exp(-(pow(dlong,2) / pow(ray_std_dev,2)));
    else
      weight = exp(-(pow(delta_distance,2) / pow(ray_std_dev,2)));

    // Update Total Probability for this particle
    total_weight += weight;
  }

  // Set particle weight to a scaled total weight for tuning. Gamma set at begginning of method.
  particle.weight = gamma * total_weight;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable.

  // **** Low Variance Resampling Method **** 
  
  // Reset
  int num_of_resamples {0}; 

  // Predefine the Number of Resamples based on number of particles; set_parameter
  num_of_resamples = particles_.size();

  // Initializations
  std::vector <Particle> reduced_particle_vec;      // return vector (vector of the kept particles)
  std::vector <double> norm_particle_weight;        // norm particle vec
  std::vector <double> bin_edges(num_of_resamples); // vector for low variance resample bins   
  double total_weight {0};                          // total weight of all particles
  int input_vec_length = particles_.size();         // Get Length of Input Vector for Looping

  // Step 1: Normalize Particle Weights
  // Repopulate the weights with normalized weights (Reference 51:00 min in 2021.09.22 Lecture)
  for (int k {0}; k < input_vec_length; k++)
  {
    norm_particle_weight.push_back(abs(exp(particles_[k].weight - max_particle_weight)));
    total_weight += norm_particle_weight[k];
    bin_edges[k] = total_weight;
  }

  // Step 2: Draw A Random Number
  // get equidistant location for low variance resample
  float equidistant_loc = total_weight/num_of_resamples;

  // Check to ensure update was run
  if (equidistant_loc == 0)
    return;
  
  // Get a random number
  float random_num = rng_.UniformRandom(0, equidistant_loc);

  // Step 3: Low Variance Resample Method, starting from first bin
  for (int m {0}; m < num_of_resamples; m++)
  {
    while (bin_edges[m] > random_num)
    {
      reduced_particle_vec.push_back(particles_[m]);
      random_num += equidistant_loc;
    }
  }

  // Step 4: Reset Variables and Set New Particle Weights to output Particle Vector
  max_particle_weight = 0;
  particles_.clear();
  particles_ = reduced_particle_vec;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // Check to make sure particles are populated and odom is initialized
  if (particles_.empty() || odom_initialized_ == false)
    return;

  // Call Update Every n'th Predict; set_parameter
  if (predict_steps >= 1 and distance_moved_over_predict > 0.01)
  {
    for(auto &particle : particles_)
    {
      // Call to Update
      Update(ranges, range_min, range_max, angle_min, angle_max, &particle);

      // Update Max Log Particle Weight Based On Return from Update
      if (particle.weight > max_particle_weight)
        max_particle_weight = particle.weight;
    } 

    // Call Resample Every n'th Update; set_parameter
    if(updates_done == 7)
    {
      // Call to Resample, fills `particles_` vector with new weights
      Resample();

      // Reset Number of Updates Done
      updates_done = 0;
    }

    predict_steps = 0;                // Reset Number of Predicted Steps Done
    distance_moved_over_predict = 0;  // Reset Distance Traveled
    updates_done++;                   // Increment how many times Update has been called to determine when to call Resample
  }

}

void ParticleFilter::Predict(const Eigen::Vector2f &odom_cur_pos, const float &odom_cur_angle ) {

  // Reference CS393r Lecture Slides "06 - Particle Filters" Slides 26 & 27

  // Capture Length of Particle Vector
  int particle_vector_length = particles_.size();
  Eigen::Vector2f variance;
  float variance_angle;

  // // Variance Parameters, set_parameter
  double a1 = 0.4;  // 0.08 // angle 
  double a2 = 0.1;  //0.01; // angle 
  double a3 = 0.2;  //0.1; // trans
  double a4 = 0.1;  //0.1; // trans

  // Location Translation to Baselink
  Eigen::Vector2f delT_baselink = Eigen::Rotation2Df(-odom_old_angle) * (odom_cur_pos - odom_old_pos);
  // Angle Distance to Baselink
  float delAngle_baselink = math_util::AngleDiff(odom_cur_angle,odom_old_angle);

  // (MAIN FUNCTION) Calculate Variance and Predict Particles Forward
  if (odom_initialized_ and delT_baselink.norm() < 1)
  {
    for (int i {0}; i < particle_vector_length; i++) 
    {   
        // Rotate from Baselink frame to Map frame
        Eigen::Vector2f delT_map = Eigen::Rotation2Df(particles_[i].angle)*delT_baselink;

        // Add Variance to deltas
        variance.x() = rng_.Gaussian(0, ( a1*delT_map.norm()+ a2*abs(delAngle_baselink) )); 
        variance.y() = rng_.Gaussian(0, ( a1*delT_map.norm() + a2*abs(delAngle_baselink) )); 
        variance_angle = rng_.Gaussian(0, ( a3*delT_map.norm() + a4*abs(delAngle_baselink) )); 

        // Update Particle Location
        particles_[i].loc += delT_map + variance;

        // Update Particle Angle
        particles_[i].angle += delAngle_baselink + variance_angle;
        
        // Set current odom values to previous values for next call to predict
        odom_old_pos = odom_cur_pos;
        odom_old_angle = odom_cur_angle;
    }

    // Make sure we moved a large enough distance to control update
    distance_moved_over_predict += delT_baselink.norm(); 

    // Keep track of how many predicts done to control update
    predict_steps++;   
  }
  else
  {
    // Set current odom values to previous values for next call to predict if odom not initialized or distance traveled is greater than one
    odom_old_pos = odom_cur_pos;
    odom_old_angle = odom_cur_angle;
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {

  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

  // Load Desired Map
  map_.Load(map_file);

  // Clear out particle vector to start fresh
  particles_.clear();

  // Initialize Variables
  Particle init_particle_cloud;
  odom_initialized_ = true;
  predict_steps = 1;
  int num_of_init_particle_cloud = 50;  // set_parameter; number of particles to initialize with

  // Create Random Particles based on zero mean Gaussian; set_parameter
  for(int i {0}; i < num_of_init_particle_cloud; i++){
    init_particle_cloud.loc.x() = loc.x() + rng_.Gaussian(0.0, 0.75);
    init_particle_cloud.loc.y() = loc.y() + rng_.Gaussian(0.0, 0.5);
    init_particle_cloud.angle = angle + rng_.Gaussian(0.0, 0.1);
    init_particle_cloud.weight = 0;
    particles_.push_back(init_particle_cloud);
  }

}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;

  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:


  if (odom_initialized_ == true)
  {
    // Initialize Variables
    double sum_x {0};
    double sum_y {0};
    double sum_cos_theta {0};
    double sum_sin_theta {0};
    double total_particle_weight {0};

    // Summations for all variables
    for (auto particles:particles_)
    {
      // Normalize Particle Weights
      double particle_norm_weight = abs(exp(particles.weight - max_particle_weight));

      // Add All Normalized Particle Weights
      total_particle_weight += particle_norm_weight;

      // Calculate Numertors for Output location
      sum_x += particles.loc.x() * particle_norm_weight;
      sum_y += particles.loc.y() * particle_norm_weight;
      sum_cos_theta += cos(particles.angle) * particle_norm_weight;
      sum_sin_theta += sin(particles.angle) * particle_norm_weight;
      
    }

    // (BEST ROBOT LOCATION ESTIMATE) Averages for x, y, and theta;
    loc.x() = sum_x/total_particle_weight;
    loc.y() = sum_y/total_particle_weight;
    angle = atan2(sum_sin_theta/total_particle_weight,sum_cos_theta/total_particle_weight);
  }
}


}  // namespace particle_filter