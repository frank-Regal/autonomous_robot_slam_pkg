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
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include <iostream>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"
#include "config_reader/config_reader.h"
#include "shared/math/line2d.h"
#include "visualization/visualization.h"

#include "particle_filter/particle_filter.h"
#include "navigation.h"

using amrl_msgs::Localization2DMsg;
using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;
using visualization::DrawCross;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "maps/GDC1.txt", "Name of vector map file");
// RRT Add 
DEFINE_string(set_pose_topic,
              "/set_pose",
              "Name of ROS topic for initialization");

// Particle Filter Add
DECLARE_int32(v);

// Create config reader entries
CONFIG_STRING(map_name_, "map");
CONFIG_FLOAT(init_x_, "init_x");
CONFIG_FLOAT(init_y_, "init_y");
CONFIG_FLOAT(init_r_, "init_r");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

bool run_ = true;
bool set_pose_ = false; // RRT Add 
Eigen::Vector2f init_pose_ (0,0); // RRT Add
sensor_msgs::LaserScan last_laser_msg_;
Navigation* navigation_ = nullptr;
particle_filter::ParticleFilter particle_filter_;

ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
ros::Publisher laser_publisher_;
VisualizationMsg vis_msg_;

vector<Vector2f> trajectory_points_;

void PublishParticles() {
  vector<particle_filter::Particle> particles;
  particle_filter_.GetParticles(&particles);
  
  for (const particle_filter::Particle& p : particles) {
    DrawParticle(p.loc, p.angle, vis_msg_);
  }
}

void PublishPredictedScan() {
  const uint32_t kColor = 0x9842f5;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  vector<Vector2f> predicted_scan;
  particle_filter_.GetPredictedPointCloud(
      robot_loc,
      robot_angle,
      last_laser_msg_.ranges.size(),
      last_laser_msg_.range_min,
      last_laser_msg_.range_max,
      last_laser_msg_.angle_min,
      last_laser_msg_.angle_max,
      &predicted_scan);
  for (const Vector2f& p : predicted_scan) {
    DrawPoint(p, kColor, vis_msg_);
  }
}

void PublishTrajectory() {
  const uint32_t kColor = 0xadadad;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static Vector2f last_loc_(0, 0);
  if (!trajectory_points_.empty() &&
      (last_loc_ - robot_loc).squaredNorm() > Sq(1.5)) {
    trajectory_points_.clear();
  }
  if (trajectory_points_.empty() ||
      (robot_loc - last_loc_).squaredNorm() > 0.25) {
    trajectory_points_.push_back(robot_loc);
    last_loc_ = robot_loc;
  }
  for (size_t i = 0; i + 1 < trajectory_points_.size(); ++i) {
    DrawLine(trajectory_points_[i],
             trajectory_points_[i + 1],
             kColor,
             vis_msg_);
  }
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.05) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  PublishParticles();
  PublishPredictedScan();
  PublishTrajectory();
  visualization_publisher_.publish(vis_msg_);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);

  static vector<Vector2f> point_cloud_; //RRT Add
  // Convert the LaserScan to a point cloud
  // The LaserScan parameters are accessible as follows:
  // msg.angle_increment // Angular increment between subsequent rays
  // msg.angle_max // Angle of the first ray
  // msg.angle_min // Angle of the last ray
  // msg.range_max // Maximum observable range
  // msg.range_min // Minimum observable range
  // msg.ranges[i] // The range of the i'th ray
  
  static vector<Vector2f> cloud(msg.ranges.size());
  
  for(std::size_t i = 0; i < msg.ranges.size(); i++){

    // Polar to Cartesian conversion, transforms to base link frame
    float angle = msg.angle_min + msg.angle_increment*i;
    float x = msg.ranges[i]*cos(angle) + kLaserLoc[0];
    float y = msg.ranges[i]*sin(angle) + kLaserLoc[1];
    
    cloud[i] = Vector2f(x, y);
  }

  navigation_->ObservePointCloud(cloud, msg.header.stamp.toNSec()); // Upstream uses .toSec()?
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);
  PublishVisualization();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  //msg.header.stamp = ros::Time::now();
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z,
      ros::Time::now().toNSec());

  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.Predict(odom_loc, odom_angle);
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  amrl_msgs::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  localization_publisher_.publish(localization_msg);
  PublishVisualization();
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_->SetNavGoal(loc, angle);

  // RRT Add
  if (set_pose_ == true){
    std::cout << "Building RRT..." << std::endl;
    navigation_->BuildRRT(init_pose_, loc);
    //navigation_->IsPathPlanned(true);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

// RRT Add
void InitCallback(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f init_loc(msg.pose.x, msg.pose.y);
  const float init_angle = msg.pose.theta;
  const string map = "maps/" + msg.map + ".txt";
  printf("Initialized Location: (%f,%f) %f\u00b0\n",
         init_loc.x(),
         init_loc.y(),
         RadToDeg(init_angle));
  init_pose_ = init_loc;
  particle_filter_.Initialize(map, init_loc, init_angle);
  trajectory_points_.clear();
  
  // Added for RRT
  navigation_->InitMap(map.c_str());

  set_pose_ = true;

  std::cout << "Set Navigation Goal..." << std::endl;

  
}

void ProcessLive(ros::NodeHandle* n) {
  
  ros::Subscriber initial_pose_sub = n->subscribe(
      FLAGS_set_pose_topic.c_str(),
      1,
      InitCallback);

  /* Added in Particle Filter, but called in main
  ros::Subscriber laser_sub = n->subscribe(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  
  ros::Subscriber odom_sub = n->subscribe(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  */

  while (ros::ok() && run_) {
    ros::spinOnce();
    PublishVisualization();
    Sleep(0.01);
  }
}

// Particle Filter Add
void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;
  vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);

  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  InitializeMsgs();
  navigation_ = new Navigation(FLAGS_map, &n);

  std::cout << "Waiting for set pose..." << std::endl;
  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);
  ros::Subscriber initial_pose_sub = 
      n.subscribe(FLAGS_set_pose_topic.c_str(), 1, &InitCallback);
  
  // Particle Filter
  visualization_publisher_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);
  laser_publisher_ =
      n.advertise<sensor_msgs::LaserScan>("scan", 1);


  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    navigation_->Run();
    loop.Sleep();
  }
  delete navigation_;
  return 0;
}
