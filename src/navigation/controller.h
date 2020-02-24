#include <vector>

#include "eigen3/Eigen/Dense"
#include "simple_queue.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H

enum Phase {ACCEL, CRUISE, DECEL};

class Controller {
  public:

    // Constructor
    explicit Controller();

    bool distance_left(Phase p);
    float getVelocity();
    float getVelocity(float distance, float speed);
    float getVelocity(float distance, float speed, float free_path_length);
    float compensate_latency();

  private:
    // speed constraints
    float current_speed;
    float distance_travelled;
    float total_distance;
    static constexpr float max_speed = 1.0f;
    static constexpr float max_acceleration = 3.0f;
    static constexpr float timestep = 0.05f;
    static constexpr float kEpsilon = 1e-5;
    static constexpr float latency = 0.12;

    // command buffer to forward predict latency
    std::deque<std::pair<Phase, float>> command_buf;    
};

#endif  // CONTROLLER_H
