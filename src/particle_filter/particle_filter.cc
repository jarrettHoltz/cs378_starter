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
  \author  Joydeep Biswas, (C) 2019
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

using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement localization using a particle filter.

// Milestone 2 will be implemented here.

namespace particle_filter {

  config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

  ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

  void ParticleFilter::GetParticles(vector<Particle>* particles) const {
    *particles = particles_;
  }


  // CP 5
  void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
      const float angle,
      int num_ranges,
      float range_min,
      float range_max,
      float angle_min,
      float angle_max,
      vector<Vector2f>* scan_ptr) {
  	vector<float> scan;
    map_.GetPredictedScan(loc, range_min, range_max, angle_min, angle_max,num_ranges, &scan);

    // construct point cloud
    const Vector2f kLaserLoc(0.2, 0);
	  vector<Vector2f> point_cloud_;
	  int range_idx = 0;
	  float angle_increment = (angle_max - angle_min)/num_ranges;
	  for (float this_angle = angle_min; this_angle < angle_max; this_angle += angle_increment){
	    float this_range = scan[++range_idx];
	    if (this_range > range_min && this_range < range_max){
	      point_cloud_.push_back(loc + kLaserLoc + Vector2f(cos(this_angle) * this_range, sin(this_angle) * this_range));
	    }
	  }
  	*scan_ptr = point_cloud_;
  }


  void ParticleFilter::Update(const vector<float>& ranges,
      float range_min,
      float range_max,
      float angle_min,
      float angle_max,
      Particle* p_ptr) {
  }

  void ParticleFilter::Resample() {
  }

  void ParticleFilter::ObserveLaser(const vector<float>& ranges,
      float range_min,
      float range_max,
      float angle_min,
      float angle_max) {
  }


  // CP 4
  void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
      const float odom_angle) {
    if (!odom_initialized_){
      odom_initialized_ = true;
    } else {
      // translation
      Eigen::Rotation2Df rot(-prev_odom_angle_);
      Vector2f delta_t_base_link = rot * (odom_loc - prev_odom_loc_);
      Eigen::Rotation2Df rot1(theta_map);
      Vector2f new_map_loc = map_loc + rot1 * delta_t_base_link;
      Vector2f expected_translation = new_map_loc - map_loc;
      map_loc = new_map_loc;
      double translation_magnitude = expected_translation.norm();

      //rotation
      float delta_theta_base_link = odom_angle - prev_odom_angle_;
      float new_theta_map = theta_map + delta_theta_base_link;
      float diff_angle = new_theta_map - theta_map;
      float rotation_magnitude = std::abs(new_theta_map - theta_map); 
      
      theta_map = new_theta_map;
      for (int i = 0; i < (int)particles_.size(); i++){
        particles_[i].loc[0] += rng_.Gaussian(expected_translation[0], k1*translation_magnitude + k2 * rotation_magnitude);
        particles_[i].loc[1] += rng_.Gaussian(expected_translation[1], k1*translation_magnitude + k2 * rotation_magnitude);
        particles_[i].angle += rng_.Gaussian(diff_angle, k3*translation_magnitude + k4 * rotation_magnitude);
      }
    }
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
  }

  // CP 4
  void ParticleFilter::Initialize(const string& map_file,
      const Vector2f& loc,
      const float angle) {

    particles_.clear();
    theta_map = angle;
    map_loc = loc;
    for (int i = 0; i < FLAGS_num_particles; i++){
      Particle this_particle;
      this_particle.loc[0] = loc[0] + rng_.Gaussian(0, 0.1);
      this_particle.loc[1] = loc[1] + rng_.Gaussian(0, 0.1);
      this_particle.angle = angle + rng_.Gaussian(0, 0.1);
      this_particle.weight = 1.0f;
      particles_.push_back(this_particle);
    }
    map_ = VectorMap(map_file);
    odom_initialized_ = false;
  }

  void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
  	Vector2f mean_loc(0,0);
  	Vector2f mean_angle_vector(0,0);
  	for (int i = 0; i < (int)particles_.size(); i++){
  		mean_loc += particles_[i].loc;
  		mean_angle_vector += Vector2f(sin(particles_[i].angle),cos(particles_[i].angle));
    }
    mean_loc /= (int)particles_.size();
    mean_angle_vector /= (int)particles_.size();
    *loc = mean_loc;
    if (mean_angle_vector == Vector2f(0,0)){
    	*angle = 0;
    } else {
    	*angle = atan2(mean_angle_vector[0],mean_angle_vector[1]);
  	}
  }


}  // namespace particle_filter
