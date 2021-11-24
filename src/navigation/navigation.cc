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
\file    navigation.cc
\brief   Starter code for navigation.
//       Obstacle Avoidance implementation from Ka-Chow
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "rrt_graph.h"
#include "shared/math/line2d.h"
#include "vector_map/vector_map.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "obstacle_avoidance/car_params.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;
using vector_map::VectorMap;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;
using visualization::DrawCross;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

std::vector<Eigen::Vector2f> q_near_;
std::vector<Eigen::Vector2f> q_rand_;
std::vector<Eigen::Vector2f> q_trim_;
std::vector<Eigen::Vector2f> q_new_;
Eigen::Vector2f nav_goal_;
float nav_goal_angle_;
int i_;
Eigen::Vector2f goal_point_;


namespace navigation {
// Given Methods
Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),//angle w.r.t global x- axis (rad)
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    path_planned_(false),
    path_goal_(0,0),
    last_path_point(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  vel_commands_ = std::vector<CommandStamped>(10);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_ = loc;
  nav_goal_angle_ = angle;
  i_ = 1;
  std::cout << "Initialized Odom (odom_loc: x= " << odom_loc_.x() << "; y= " << odom_loc_.y() << "; theta= " << odom_angle_ << ")" << std::endl;
  goal_point_.x() = nav_goal_.x();
  goal_point_.y() = nav_goal_.y();
  /*
  Eigen::Affine2f A_global_local = GetTransform(robot_loc_, odom_loc_, DegToRad(robot_angle_), odom_angle_);
  path_goal_ = LocallySmoothedPathFollower(robot_loc_); // returns global frame point
  //goal_point_ = A_global_local * path_goal_;            // Transform to local
  std::cout << "path_goal_ x: " << path_goal_.x() << "; y: " << path_goal_.y() << std::endl;
  Eigen::Vector2f delta = path_goal_ - robot_loc_;
  goal_point_.x() =  abs(delta.x())+odom_loc_.x();
  goal_point_.y() =  abs(delta.y())+odom_loc_.y();
  */


}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;

  /*
  std::cout << "robot_loc_ x: " << robot_loc_.x() 
            << "; robot_loc_ y: " << robot_loc_.y()
            << "; robot_angle_  " << robot_angle_ << std::endl;
            */
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel,
                                uint64_t time) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    odom_stamp_ = time - car_params::sensing_latency;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;

  /*
  std::cout << "odom_loc_ x: " << odom_loc_.x()
            << "; odom_loc_ y: " << odom_loc_.y()
            << "; odom_angle_: " << odom_angle_ 
            << "\n\n" <<std::endl;
            */

  odom_stamp_ = time - car_params::sensing_latency;
  last_odom_stamp_ = odom_stamp_;
  
  has_new_odom_ = true;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   uint64_t time) {
  point_cloud_ = cloud;
  point_cloud_stamp_ = time - car_params::sensing_latency;
  has_new_points_= true;
}

//Solve the TOC Problem / Obstacle Avoidance
void Navigation::TimeOptimalControl(const PathOption& path) {
    double current_speed = odom_state_tf.speed;
    double min_stop_distance = car_params::safe_distance-0.5*current_speed*current_speed/car_params::min_acceleration; //calculate the minimum stopping distance at current velocity
    double set_speed = (path.free_path_length>min_stop_distance)?car_params::max_velocity:0; //decelerate if free path is is smaller than minimum stopping distance otherwise accelerate
    
    //std::cout << "free path: " << path.free_path_length << std::endl;
    //std::cout << "min stop: " << min_stop_distance << std::endl;
    
    //publish command to topic 
    drive_msg_.header.seq++;
    drive_msg_.header.stamp = ros::Time::now();
    drive_msg_.curvature = path.curvature;
    drive_msg_.velocity = set_speed;

    // std::cout << path.free_path_length << std::endl;

    drive_pub_.publish(drive_msg_);
    //TODO: record the commands used for latency compensation
}

void Navigation::TransformPointCloud(TimeShiftedTF transform){

  Eigen::Matrix2f R;
  R << cos(transform.theta), sin(transform.theta), -sin(transform.theta), cos(transform.theta);

  transformed_point_cloud_.resize(point_cloud_.size());

  for(std::size_t i = 0; i < point_cloud_.size(); i++){
    transformed_point_cloud_[i] =  R*(point_cloud_[i] - transform.position);
  }
}

void Navigation::Run(){
  //This function gets called 20 times a second to form the control loop.
  uint64_t start_loop_time = ros::Time::now().toNSec();
  uint64_t actuation_time = start_loop_time + car_params::actuation_latency;
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized or path is not planned we can't do anything.
  if (!odom_initialized_ or !path_planned_) return;

  //Latency Compensation
  obstacle_avoidance::CleanVelocityBuffer(vel_commands_, std::min(odom_stamp_, point_cloud_stamp_));

  if(has_new_odom_){
    odom_state_tf.position = Eigen::Vector2f(0, 0);
    odom_state_tf.speed = robot_vel_[0];
    odom_state_tf.theta = 0;
    odom_state_tf.stamp = odom_stamp_;
    has_new_odom_ = false;
  }

  // Find drive cmd directly before odom msg
  int cmd_start_index = std::lower_bound(vel_commands_.begin(), vel_commands_.end(), odom_state_tf.stamp) - vel_commands_.begin() - 1;

  // Integrate odometry into the future across the vector of vehicle commands
  for(std::size_t i = cmd_start_index; i < vel_commands_.size() - 1; i++){
    odom_state_tf = obstacle_avoidance::IntegrateState(odom_state_tf, vel_commands_[i], vel_commands_[i+1].stamp - odom_state_tf.stamp);
  }
  odom_state_tf = obstacle_avoidance::IntegrateState(odom_state_tf, vel_commands_.back(), actuation_time - odom_state_tf.stamp);

  // odom_state_tf is now transform from last odom msg to future actuation time
  
  // get transform from point cloud msg to odom msg
  uint64_t latest_sensor_msg_stamp; // whichever msg is newer
  TimeShiftedTF pc_to_odom_transform; // Empty transform to populate as we integrate between sensor msgs
  pc_to_odom_transform.speed = robot_vel_[0];

  if(point_cloud_stamp_ > odom_stamp_){
    latest_sensor_msg_stamp = point_cloud_stamp_;
    pc_to_odom_transform.stamp = odom_stamp_;
  }
  else{
    latest_sensor_msg_stamp = odom_stamp_;
    pc_to_odom_transform.stamp = point_cloud_stamp_;
  }

  // Integrates from older sensor message to newer message
  int pc_to_odom_index = 0;
  while(vel_commands_[pc_to_odom_index+1].stamp < latest_sensor_msg_stamp){
    pc_to_odom_transform = obstacle_avoidance::IntegrateState(pc_to_odom_transform, vel_commands_[pc_to_odom_index], vel_commands_[pc_to_odom_index+1].stamp - pc_to_odom_transform.stamp);
    pc_to_odom_index++;
  }
  pc_to_odom_transform = obstacle_avoidance::IntegrateState(pc_to_odom_transform, vel_commands_[pc_to_odom_index], latest_sensor_msg_stamp - pc_to_odom_transform.stamp);

  // With odom->future_odom and point_cloud->odom, we can find transform for point cloud at future actuation time
  if(point_cloud_stamp_ > odom_stamp_){
    pc_to_odom_transform.theta = odom_state_tf.theta - pc_to_odom_transform.theta;
    pc_to_odom_transform.position = odom_state_tf.position - pc_to_odom_transform.position;
  }
  else{
    pc_to_odom_transform.theta += odom_state_tf.theta;
    pc_to_odom_transform.position += odom_state_tf.position;
  }

  // Transform point cloud into future actuation time, storing as transformed_point_cloud_
  TransformPointCloud(pc_to_odom_transform);

  // Visualize Latency Compensation
  //obstacle_avoidance::LatencyPointCloud(local_viz_msg_, transformed_point_cloud_);
  //obstacle_avoidance::DrawCarLocal(local_viz_msg_, odom_state_tf.position, odom_state_tf.theta);

  // "Carrot on a stick" goal point, and resulting goal curvature

  Eigen::Vector2f goal_point;  
  
  /*
  // Transformation from robot in map frame to goal location in map frame
  Eigen::Vector2f delta_trans_tmg = robot_loc_ - nav_goal_;
  float delta_angle_tmg = AngleDiff(robot_angle_,nav_goal_angle_);
  Eigen::Affine2f Tmg = Eigen::Translation2f(delta_trans_tmg) * Eigen::Rotation2Df(delta_angle_tmg);

  // Transformation from robot in map frame to odom frame
  Eigen::Vector2f delta_trans_Tmo= robot_loc_ - odom_loc_;
  float delta_angle_Tmo = -AngleDiff(DegToRad(robot_angle_),odom_angle_);
  Eigen::Affine2f Tmo = Eigen::Translation2f(delta_trans_Tmo) * Eigen::Rotation2Df(delta_angle_Tmo);
  
  // Transformation from robot odom frame to baselink frame
  Eigen::Vector2f baselink_loc (0.2,0);
  Eigen::Vector2f delta_trans_Tor = odom_loc_ - baselink_loc;
  float delta_angle_Tor = 0;
  Eigen::Affine2f Tor = Eigen::Translation2f(delta_trans_Tor) * Eigen::Rotation2Df(delta_angle_Tor);
  
  goal_point = Tor.inverse() * Tmo.inverse() * Tmg * nav_goal_;
  */

 // Nav Goal (point relative to m)
  

 // transformation s relative to m (map)
  float del_t_sm = robot_angle_; // angle from robot map m frame to fixed map frame s
  Eigen::Vector2f del_loc_sm = robot_loc_;
  Eigen::MatrixXf TSM(3,3);
  TSM  << cos(del_t_sm), -sin(del_t_sm), del_loc_sm.x(),
         sin(del_t_sm),  cos(del_t_sm) , del_loc_sm.y(),
         0, 0, 1;

  Eigen::Vector2f dist_from_path_goal;
  dist_from_path_goal.x() = robot_loc_.x() - path_goal_.x();  
  dist_from_path_goal.y() = robot_loc_.x() - path_goal_.x(); 
  if (dist_from_path_goal.norm() == 0){
    tree_.ClearTree();
    std::cout << "rebuilding tree" << std::endl;
    BuildRRT(robot_loc_, nav_goal_);
  }

  path_goal_ = LocallySmoothedPathFollower(robot_loc_); // returns global frame point

  // std::cout << "path goal_  x: " << path_goal_.x() << "; y: " << path_goal_.y() << std::endl;

  if (path_goal_.x() == 0 and path_goal_.y() == 0){
    last_path_point = 0; 
    path_goal_ = robot_loc_;
    return;
  }

  Eigen::Vector3f Vs (path_goal_.x(),path_goal_.y(), 1);
  Eigen::Vector3f Vm = TSM.inverse() * Vs;
  Eigen::Vector2f Vm2 (Vm.x(),Vm.y());

  goal_point = Vm2;

  Eigen::Vector2f check;
  check.x() = nav_goal_.x() - robot_loc_.x(); // Transform to local
  check.y() = nav_goal_.y() - robot_loc_.y();
  if(check.norm() < 0.5)
    return;
    
  float goal_curvature = obstacle_avoidance::GetCurvatureFromGoalPoint(goal_point);
  goal_curvature = Clamp(goal_curvature, car_params::min_curvature, car_params::max_curvature);

  // 4) Generate range of possible paths centered on goal_curvature, using std::vector<struct PathOption>
  static std::vector<struct PathOption> path_options(car_params::num_curves);

  // 5) For possible paths and point_cloud:
  for(std::size_t curve_index = 0; curve_index < path_options.size(); curve_index++){
    float curvature = obstacle_avoidance::GetCurvatureOptionFromRange(curve_index, goal_curvature, car_params::min_curvature, car_params::curvature_increment);
    
    // Initialize path_option and collision bounds for curvature
    obstacle_avoidance::PathBoundaries collision_bounds(abs(curvature));
    path_options[curve_index] = navigation::PathOption{
      curvature,                    // curvature
      10,                           // default clearance
      car_params::max_path_length,  // free Path Length
      Eigen::Vector2f(0, 0),        // obstacle point
      Eigen::Vector2f(0, 0)};       // closest point

    obstacle_avoidance::EvaluatePathWithPointCloud(path_options[curve_index], collision_bounds, transformed_point_cloud_);
    obstacle_avoidance::LimitFreePath(path_options[curve_index], goal_point);
    obstacle_avoidance::EvaluateClearanceWithPointCloud(path_options[curve_index], collision_bounds, transformed_point_cloud_);

    // Visualization test code
    //visualization::DrawPathOption(path_options[curve_index].curvature, path_options[curve_index].free_path_length, path_options[curve_index].clearance, local_viz_msg_);
    //visualization::DrawCross(path_options[curve_index].obstruction, 0.1,  0x0046FF, local_viz_msg_);
  }

  // 6) Select best path from scoring function
  struct PathOption best_path = obstacle_avoidance::ChooseBestPath(path_options,goal_point);
  obstacle_avoidance::VisualizeObstacleAvoidanceInfo(goal_point,path_options,best_path,local_viz_msg_);
  
  // 7) Publish commands with 1-D TOC, update vector of previous vehicle commands
  TimeOptimalControl(best_path);

  drive_pub_.publish(drive_msg_);
  // Remove for obstacle avoidance
    
  CommandStamped drive_cmd(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toNSec() + car_params::actuation_latency);
  vel_commands_.push_back(drive_cmd);

  // Vizualize RRT
  Vizualize();

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

// Added the Following for RRT
void Navigation::InitMap(const string& map_file)
{ // Load Map File. Called in navigation_main.cc
  map_.Load(map_file);
  std::cout << "Loaded Map: " << map_file << std::endl;
}

void Navigation::IsPathPlanned(bool path_planned)
{
  path_planned_ = path_planned;
}

Eigen::Vector2f Navigation::FindIntersection(const Eigen::Vector2f q_near, const Eigen::Vector2f q_new_cur)
{ // Set a new node if the map intersects

  // Create Line from closest node to the new node
  line2f q_line (q_near.x(),q_near.y(), q_new_cur.x(), q_new_cur.y());
  Eigen::Vector2f output_q = q_new_cur;

  Eigen::Vector2f delta_q (0,0);
  Eigen::Vector2f closest_q (0,0);
  float magnitude {0};
  float min_dist {500000};  //potential bug


  for (size_t i {0}; i < map_.lines.size(); ++i){

    const line2f map_line = map_.lines[i];
    bool intersects = map_line.Intersection(q_line, &closest_q);
    
    if (intersects == true){
      delta_q = q_near - closest_q;
      magnitude = delta_q.norm();

      // find the shortest intersecting line
      if (magnitude < min_dist){
        output_q = closest_q;
        min_dist = magnitude;
      }
    }
  }

  return output_q;
}

void Navigation::BuildRRT(const Eigen::Vector2f q_init, const Eigen::Vector2f q_goal)
{ // Rapidly Exploring Random Tree
  int k = 0;

  Eigen::Vector2f del = q_goal - q_init;
  float mag_w_buff = del.norm()+5;
  Eigen::Vector2f C (abs(q_init.x())+mag_w_buff, abs(q_init.y())+mag_w_buff); // c-space / exploration space

  const float delta_q = 0.2;     // max distance for rrt
  float goal_threshold = 0.3;    // meters
  Eigen::Vector2f q_rand (0,0);  // random node within the c-space
  Eigen::Vector2f q_near (0,0);  // nears node
  Eigen::Vector2f q_trim (0,0);  // holder for new node
  Eigen::Vector2f q_new  (0,0);  // holder for new node
  bool goal_reached = false;     // check if robot is at goal yet
  
  tree_.SetInitNode(q_init);     // initialize the starting point of rrt

  while (goal_reached == false)
  {
    q_rand = tree_.GetRandq(C.x(), C.y());            // get a random node
    q_near = tree_.GetClosestq(q_rand);               // get the closest node to the random node
    q_trim = tree_.GetNewq(q_near, q_rand, delta_q);  // get a new q based on the max distance    
    q_new  = FindIntersection(q_near, q_trim);        // determine if the map intersects with the new node
    goal_reached = tree_.IsNearGoal(q_new, q_goal,goal_threshold);  // is the node within the threshold of the goal

    if (goal_reached == true){
      std::cout << "path found!" << std::endl;
      tree_.FindPathBack(q_near,q_new);
      path_planned_ = true;
    } else if (k > 1000000) {
      std::cout << "path not found :( \n\nretry!" << std::endl;
      break;
    } else {
      tree_.AddVertex(q_new);
      tree_.AddEdge(q_near,q_new);
    }
    k++;
  } 
}

void Navigation::Vizualize()
{ // Vizualize the RRT Tree

  // Set Colors
  const uint32_t black = 0x000000;
  const uint32_t green = 0x034f00;
  const uint32_t magenta = 0xe303fc;
  /*
  // Vizualize the Tree Nodes
  for (auto& p:tree_.GetVertices()){
    DrawPoint(p, magenta, global_viz_msg_);
  }

  // Vizualize the Tree Edges
  for (auto& l:tree_.GetEdges()){
    DrawLine(l[0],l[1],magenta,global_viz_msg_);
  }
  */
  // Vizualize the Path Back
  int i = 0;
  Eigen::Vector2f buff;
  for (auto& f:tree_.GetPathBack()){
    DrawPoint(f,black,global_viz_msg_);
    if (i != 0){
      DrawLine(buff, f, black, global_viz_msg_);
    }
    buff = f;
    i++;
  }

  // Vizualize the Navigation Goal
  DrawCross(nav_goal_,0.2,green,global_viz_msg_);

  DrawCross(path_goal_,0.2,magenta,global_viz_msg_);

}

Eigen::Vector2f Navigation::LocallySmoothedPathFollower(const Eigen::Vector2f robot_loc)
{
  // Find farthest waypoint that can be reached with a straight line
  // robot_loc
  // tree_.GetPathBack()
  Eigen::Vector2f closest_point (0,0);
  Eigen::Vector2f path_goal (0,0);
  int size = tree_.GetPathBack().size();
  //float max_carrot_dist = 4;

  //Eigen::Vector2f delta = robot_loc - path_goal;
  //float mag = delta.norm();
  //float max_carrot_dist = 4;
  int n = last_path_point;

  for (int j = n ; j < size; j++){
  

    // compare virtual line between vehicle location and path waypoints to map
    line2f robot_line (robot_loc.x(),robot_loc.y(), tree_.GetPathBack()[j].x(), tree_.GetPathBack()[j].y());

    for (size_t i {0}; i < map_.lines.size(); ++i){
      const line2f map_line = map_.lines[i];
      bool intersects = map_line.Intersection(robot_line, &closest_point);
      if (intersects == true) //or (robot_line.Length() > max_carrot_dist))
        return path_goal;  
    }

    path_goal = tree_.GetPathBack()[j];
    last_path_point = j;
    if(j == (size-1))
      path_goal = nav_goal_;

  }
  return path_goal;
}

Eigen::Affine2f Navigation::GetTransform(const Eigen::Vector2f& from, const Eigen::Vector2f& to, const float& from_angle, const float& to_angle)
{
  Eigen::Vector2f delta_pos;
  delta_pos = from - to;
  float delta_angle = AngleDiff(from_angle,to_angle);
  Eigen::Affine2f A_from_to = Eigen::Translation2f(delta_pos.x(),delta_pos.y()) * Eigen::Rotation2Df(delta_angle);
  std::cout << "\n" << A_from_to.matrix() << std::endl;
  return A_from_to;
}
}  // namespace navigation
