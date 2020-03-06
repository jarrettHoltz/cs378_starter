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

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {

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

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
	// if (!odom_initialized_) {
	// 	prev_odom_loc_ = odom_loc;
	//   prev_odom_angle_ = odom_angle;
	//   odom_initialized_ = true;
	// } 
	// for (auto it = particles_.begin(); it!=particles_.end(); ++it)
	// {
	// 	Particle this_particle = *it;
	// 	// cout <<this_particle<<endl;
	//   this_particle.loc += (odom_loc - prev_odom_loc_);
	//   this_particle.angle += (odom_angle - prev_odom_angle_);
	// }
	if (!odom_initialized_){
		odom_initialized_ = true;
	} else {
		auto expected_translation = odom_loc - prev_odom_loc_;
    auto translation_magnitude = expected_translation.norm();
    auto expected_rotation = std::abs(odom_angle - prev_odom_angle_); 
		for (int i = 0; i < (int)particles_.size(); i++){
			// particles_[i].loc += (odom_loc - prev_odom_loc_);
			particles_[i].loc[0] += rng_.Gaussian(expected_translation[0], k1*translation_magnitude + k2 * expected_rotation);
			particles_[i].loc[1] += rng_.Gaussian(expected_translation[1], k1*translation_magnitude + k2 * expected_rotation);
			particles_[i].angle += rng_.Gaussian(odom_angle - prev_odom_angle_, k3*translation_magnitude + k4 * expected_rotation);
		}
	}
	prev_odom_loc_ = odom_loc;
	prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {

	particles_.clear();
  prev_odom_loc_= loc;
  prev_odom_angle_ = angle;
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
}


}  // namespace particle_filter
