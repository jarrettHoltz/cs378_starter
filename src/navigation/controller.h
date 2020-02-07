#include <vector>

#include "eigen3/Eigen/Dense"

#ifndef CONTROLLER_H
#define CONTROLLER_H

enum Phase {ACCEL, CRUISE, DECEL};

class Controller {
  public:

    // Constructor
    explicit Controller(const float total_distance, const float curvature);

    bool distance_left(Phase p);
    float getVelocity();
    float getVelocity(float distance, float speed);
    float getCurvature();

  private:
    // speed constraints
    float current_speed;
    float distance_travelled;
    float total_distance;
    float curvature;
    static constexpr float max_speed = 1.0f;
    static constexpr float max_acceleration = 3.0f;
    static constexpr float timestep = 0.05f;
    static constexpr float kEpsilon = 1e-5;
    static constexpr float latency = 0.1;
};

#endif  // CONTROLLER_H
