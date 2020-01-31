#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "controller.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

Controller::Controller(const float total_distance) :
  current_speed(0.0),
  distance_travelled(0.0) {
    this->total_distance = total_distance;
  }

bool Controller::distance_left(Phase p) {
  if ((total_distance - distance_travelled) < 0){
    return false;
  }
  switch (p) {
    case ACCEL: {
      float dist_left = current_speed * timestep + 0.5 * max_acceleration * std::pow(timestep, 2) +
        std::pow(std::min(max_speed, current_speed + max_acceleration * timestep), 2) / (2 * max_acceleration); 
      return dist_left < (total_distance - distance_travelled);
    }
    case CRUISE: {
      float dist_left = max_speed * timestep + std::pow(max_speed, 2) / (2 * max_acceleration); 
      return dist_left < (total_distance - distance_travelled);
    }
    default:
      return false; 
  }
}

float Controller::getVelocity(){
  if (current_speed < max_speed && distance_left(ACCEL)) {
    // update distance travelled
    distance_travelled += current_speed * timestep + 0.5 * max_acceleration * std::pow(timestep, 2);
    // accelerate
    current_speed = std::min(current_speed + max_acceleration * timestep, max_speed);
  } else if (current_speed == max_speed && distance_left(CRUISE)) {
    // update distance travelled
    distance_travelled += max_speed * timestep;
    // cruise
  } else {
    // calculate deceleration given the distance left
    float distance_left = total_distance - distance_travelled;
    float deceleration = std::pow(current_speed, 2) / (2 * distance_left);

    distance_travelled += current_speed * timestep - 0.5 * deceleration * std::pow(timestep, 2);
    current_speed = std::max(0.0f, current_speed - deceleration * timestep); 
  }
  return current_speed;
}

float Controller::getVelocity(float distance, float speed){
  current_speed = speed;
  distance_travelled = distance;
  if (current_speed < max_speed && distance_left(ACCEL)) {
    // accelerate
    current_speed = 1.0;
  } else if (current_speed == max_speed && distance_left(CRUISE)) {
    current_speed = 1.0;
  } else {
    // calculate deceleration given the distance left
    float distance_left = total_distance - distance_travelled;
    float deceleration = std::pow(current_speed, 2) / (2 * distance_left);

    distance_travelled += current_speed * timestep - 0.5 * deceleration * std::pow(timestep, 2);
    if(distance_left < 0){
      current_speed = 0.0;
    } else {
      current_speed = std::max(0.0f, current_speed - deceleration * timestep);
    }
  }
  return current_speed;
}
