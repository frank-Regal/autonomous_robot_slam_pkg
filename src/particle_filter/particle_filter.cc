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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\university:    The University of Texas at Austin
\class:         CS 393r Autonomous Robots
\assignment:    Assignment 2 - Particle Filter
\author:        Mary Tebben & Frank Regal
\adopted from:  Dr. Joydeep Biswas
//
// Original Author of particle filter: Team Ka-Chow
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

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

CONFIG_INT(num_particles, "num_particles");
CONFIG_FLOAT(init_x_sigma, "init_x_sigma");
CONFIG_FLOAT(init_y_sigma, "init_y_sigma");
CONFIG_FLOAT(init_r_sigma, "init_r_sigma");

CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");
CONFIG_FLOAT(k5, "k5");
CONFIG_FLOAT(k6, "k6");

CONFIG_FLOAT(laser_offset, "laser_offset");

CONFIG_FLOAT(min_dist_to_update, "min_dist_to_update");
CONFIG_DOUBLE(sigma_observation, "sigma_observation");
CONFIG_DOUBLE(gamma, "gamma");
CONFIG_DOUBLE(dist_short, "dist_short");
CONFIG_DOUBLE(dist_long, "dist_long");
CONFIG_DOUBLE(range_min, "range_min");
CONFIG_DOUBLE(range_max, "range_max");

CONFIG_DOUBLE(resize_factor, "resize_factor");
CONFIG_INT(resample_frequency, "resample_frequency");

namespace particle_filter {
  Vector2f first_odom_loc;
  float first_odom_angle;

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

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
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  scan.resize((int)(num_ranges / CONFIG_resize_factor));
  
  Vector2f sensor_loc = BaseLinkToSensorFrame(loc, angle);
  
  int v_start_index = std::lower_bound(horizontal_lines_.begin(), horizontal_lines_.end(), line2f(sensor_loc, sensor_loc), horizontal_line_compare) - horizontal_lines_.begin();
  int h_start_index = std::lower_bound(vertical_lines_.begin(), vertical_lines_.end(), line2f(sensor_loc, sensor_loc), vertical_line_compare) - vertical_lines_.begin();

  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) { // for each ray
    // Initialize the ray line
    line2f ray(0, 1, 2, 3);
    float ray_angle = angle + angle_min + CONFIG_resize_factor * i / num_ranges * (angle_max - angle_min);
    ray.p0 = sensor_loc;
    ray.p1[0] = sensor_loc[0] + range_max * cos(ray_angle);
    ray.p1[1] = sensor_loc[1] + range_max * sin(ray_angle);
    Vector2f final_intersection = sensor_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));

    // Return if no map is loaded
    if(!vertical_lines_.size() || !horizontal_lines_.size()){
      return;
    }

    int h_dir = math_util::Sign(ray.Dir().x());
    int v_dir = math_util::Sign(ray.Dir().y());

    uint64_t v_search_index = v_start_index;
    uint64_t h_search_index = h_start_index;

    if(h_dir < 0){
      h_search_index += 1;
    }

    if(v_dir > 0){
      v_search_index += 1;
    }
    
    Vector2f final_intersection_x = sensor_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    float curr_dist_x = 0;
    while(!ray.Intersection(vertical_lines_[h_search_index], &final_intersection_x) && curr_dist_x < abs(ray.p1.x() - ray.p0.x()) && h_search_index < vertical_lines_.size() ){ // vertical search
      curr_dist_x = abs(vertical_lines_[h_search_index].p0.x() - ray.p0.x());
      h_search_index += h_dir;
    }
    float curr_dist_y = 0;
    Vector2f final_intersection_y = sensor_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    while(!ray.Intersection(horizontal_lines_[v_search_index], &final_intersection_y) && curr_dist_y < abs((ray.p1.y() - ray.p0.y())) && v_search_index < horizontal_lines_.size() ){ // vertical search
      curr_dist_y = abs(horizontal_lines_[v_search_index].p0.y() - ray.p0.y());
      v_search_index += v_dir;
    }
    float curr_dist_angled = range_max;
    Vector2f final_intersection_angled = sensor_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    for (size_t i = 0; i < angled_lines_.size(); ++i) {
      if(ray.Intersection(angled_lines_[i], &final_intersection_angled)){
        float new_dist = (final_intersection_angled - ray.p0).norm();
        if(new_dist < curr_dist_angled){
          curr_dist_angled = new_dist;
        }
      }     
    }

    if((final_intersection_x - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_x;
    }
    if((final_intersection_y - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_y;
    }
    if((final_intersection_angled - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_angled;
    }

    scan[i] = final_intersection;
  }
}

double GetRobustObservationLikelihood(double measured, double expected, double dist_short, double dist_long){
  
  if(measured < CONFIG_range_min || measured > CONFIG_range_max){
    return 0;
  }
  else if(measured < (expected - dist_short)){
    return dist_short;
  }
  else if(measured > (expected + dist_long)){
    return dist_long;
  }
  else{
    return measured - expected;
  }
}

// Update the weight of the particle based on how well it fits the observation
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  
  // Get predicted point cloud
  Particle &particle = *p_ptr;
  vector<Vector2f> predicted_cloud; // map frame

  GetPredictedPointCloud(particle.loc, 
                         particle.angle, 
                         ranges.size(), 
                         range_min, 
                         range_max,
                         angle_min,
                         angle_max,
                         &predicted_cloud);
  Vector2f sensor_loc = BaseLinkToSensorFrame(particle.loc, particle.angle);
  // resize the ranges
  vector<float> trimmed_ranges(predicted_cloud.size());
  particle.weight = 0;

  // Calculate the particle weight
  for(std::size_t i = 0; i < predicted_cloud.size(); i++) {
    trimmed_ranges[i] = ranges[i * CONFIG_resize_factor];
    double predicted_range = (predicted_cloud[i] - sensor_loc).norm();
    double diff = GetRobustObservationLikelihood(trimmed_ranges[i], predicted_range, CONFIG_dist_short, CONFIG_dist_long);
    particle.weight += -CONFIG_gamma * Sq(diff) / Sq(CONFIG_sigma_observation);
  } 
}

void ParticleFilter::Resample() {
  vector<Particle> new_particles(particles_.size());
  vector<double> weight_bins(particles_.size());
  
  // Calculate weight sum, get bins sized by particle weights as vector
  double weight_sum = 0;
  for(std::size_t i = 0; i < particles_.size(); i++){
    weight_sum += particles_[i].weight;
    weight_bins[i] = weight_sum;
  }

  // During resampling:
  for(std::size_t i = 0; i < particles_.size(); i++){
    double rand_weight = rng_.UniformRandom(0, weight_sum);
    auto new_particle_index = std::lower_bound(weight_bins.begin(), weight_bins.end(), rand_weight) - weight_bins.begin();
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size());
  }
  
  // After resampling:
  particles_ = new_particles;
}

void ParticleFilter::LowVarianceResample() {
  vector<Particle> new_particles(particles_.size());

  double select_weight = rng_.UniformRandom(0, weight_sum_);

  for(std::size_t i = 0; i < particles_.size(); i++){
    int new_particle_index = std::lower_bound(weight_bins_.begin(), weight_bins_.end(), select_weight) - weight_bins_.begin();
    select_weight = std::fmod(select_weight + weight_sum_/((double) particles_.size()), weight_sum_);
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size()); // rng_.UniformRandom(); good for testing
  }
  weight_sum_ = 1.0;
  
  // After resampling:
  particles_ = new_particles;
}

void ParticleFilter::SetParticlesForTesting(vector<Particle> new_particles){
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if((last_update_loc_ - prev_odom_loc_).norm() > CONFIG_min_dist_to_update){
    //double start_time = GetMonotonicTime();
    max_weight_log_ = -1e10; // Should be smaller than any
    weight_sum_ = 0;
    weight_bins_.resize(particles_.size());
    std::fill(weight_bins_.begin(), weight_bins_.end(), 0);

    // Update each particle with log error weight and find largest weight (smallest negative number)
    for(Particle &p: particles_){
      Update(ranges, range_min, range_max, angle_min, angle_max, &p);
      max_weight_log_ = std::max(max_weight_log_, p.weight);
    }

    // Normalize log-likelihood weights by max log weight and transform back to linear scale
    // Sum all linear weights and generate bins
    for(std::size_t i = 0; i < particles_.size(); i++){
      particles_[i].weight = exp(particles_[i].weight - max_weight_log_);
      weight_sum_ += particles_[i].weight;
      weight_bins_[i] = weight_sum_;
    }

    if(!(resample_loop_counter_ % CONFIG_resample_frequency)){
      LowVarianceResample();
    }
    last_update_loc_ = prev_odom_loc_;
    resample_loop_counter_++;

    // double end_time = GetMonotonicTime();
    // std::cout << "First: " << 1000*(end_time - start_time) << std::endl;
  }                     
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // propagate particles forward based on odometry.

  // rotation matrix from last odom to last baselink
  auto rot_odom1_to_bl1 = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
  
  // Change in translation and angle from odometry
  Eigen::Vector2f delta_translation = rot_odom1_to_bl1 * (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);

  for(Particle &particle: particles_){
    // Get noisy angle
    float sigma_tht = CONFIG_k5 * delta_translation.norm() + CONFIG_k6 * abs(delta_angle);
    float noisy_angle = delta_angle + rng_.Gaussian(0.0, sigma_tht);

    // Get translation noise in Base Link 2
    float sigma_x = CONFIG_k1 * delta_translation.norm() + CONFIG_k2 * abs(delta_angle);;
    float sigma_y = CONFIG_k3 * delta_translation.norm() + CONFIG_k4 * abs(delta_angle);
    Eigen::Vector2f e_xy = Eigen::Vector2f((float) rng_.Gaussian(0.0, sigma_x),(float) rng_.Gaussian(0.0, sigma_y));

    // Transform noise to Base Link 1 using estimated angle to get noisy translation
    auto rot_b2_to_b1 = Eigen::Rotation2D<float>(delta_angle).toRotationMatrix();
    Eigen::Vector2f noisy_translation = delta_translation + rot_b2_to_b1 * e_xy; // in previous base_link
    
    // Transform noise to map using current particle angle
    auto rot_bl1_to_map = Eigen::Rotation2D<float>(particle.angle).toRotationMatrix();
    particle.loc += rot_bl1_to_map * noisy_translation;   
    particle.angle += noisy_angle;        
    cout << noisy_angle - delta_angle << endl;
  }

  // Update previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log.

  particles_.resize(CONFIG_num_particles);

  for(Particle &particle: particles_){
    particle.loc = Eigen::Vector2f(
      loc[0] + rng_.Gaussian(0, CONFIG_init_x_sigma),
      loc[1] + rng_.Gaussian(0, CONFIG_init_y_sigma)
      );
    particle.angle = angle + rng_.Gaussian(0, CONFIG_init_r_sigma);
    particle.weight = 1/((double)particles_.size());
  }
  max_weight_log_ = 0;
  last_update_loc_ = prev_odom_loc_;
  map_.Load(map_file);
  SortMap();
}

bool ParticleFilter::horizontal_line_compare(const geometry::line2f l1, const geometry::line2f l2){
  return l1.p0.y() < l2.p0.y();
}

bool ParticleFilter::vertical_line_compare(const geometry::line2f l1, const geometry::line2f l2){
  return l1.p0.x() < l2.p0.x();
}

void ParticleFilter::SortMap(){
   // Split lines in map into horizontal, vertical, and angled
  horizontal_lines_.clear();
  vertical_lines_.clear();
  angled_lines_.clear();
  
  for (size_t i = 0; i < map_.lines.size(); ++i) {
      const geometry::line2f line = map_.lines[i];
      if(line.p0.y() == line.p1.y()){
        horizontal_lines_.push_back(line);
      }
      else if(line.p0.x() == line.p1.x()){
        vertical_lines_.push_back(line);
      }
      else{
        angled_lines_.push_back(line);
      }
  }
  // Sort horizontal and vertical in ascending order
  std::sort(horizontal_lines_.begin(), horizontal_lines_.end(), horizontal_line_compare);
  std::sort(vertical_lines_.begin(), vertical_lines_.end(), vertical_line_compare);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles.

  Eigen::Vector2f angle_point = Eigen::Vector2f(0, 0);
  for(Particle particle: particles_){
    loc += particle.loc * particle.weight;
    angle_point += Eigen::Vector2f(cos(particle.angle), sin(particle.angle)) * particle.weight;
  }

  loc /= weight_sum_;
  angle_point /= weight_sum_;
  angle = atan2(angle_point[1], angle_point[0]);
}

Eigen::Vector2f ParticleFilter::BaseLinkToSensorFrame(const Eigen::Vector2f &loc, const float &angle){
  return loc + Vector2f(CONFIG_laser_offset*cos(angle), CONFIG_laser_offset*sin(angle));
}

}  // namespace particle_filter