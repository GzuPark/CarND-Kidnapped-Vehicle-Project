/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  //Set number of particles
  num_particles = 100;

  //Initialize all particles to first position and all weights to 1.
  particles.resize(num_particles);
  weights.resize(num_particles);

  double std_x, std_y, std_theta;
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];

  default_random_engine gen;

  normal_distribution<double> dist_x(x, std_x);

  normal_distribution<double> dist_y(y, std_y);

  normal_distribution<double> dist_theta(theta, std_theta);

  for (unsigned i=0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0f;

    weights[i] = 1.0f;
    particles[i] = p;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  double std_x, std_y, std_theta;
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];

  for (unsigned i=0; i < num_particles; i++) {
    Particle p = particles[i];
    if (fabs(yaw_rate) > 1e-6) {
      float delta_yaw = yaw_rate * delta_t;
      p.x += (velocity / yaw_rate) * (sin(p.theta + delta_yaw) - sin(p.theta));
      p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + delta_yaw));
      p.theta += delta_yaw;
    }
    else {
      p.x += (velocity * delta_t * cos(p.theta));
      p.y += (velocity * delta_t * sin(p.theta));
    }

    normal_distribution<double> dist_x(p.x, std_x);
		normal_distribution<double> dist_y(p.y, std_y);
		normal_distribution<double> dist_theta(p.theta, std_theta);

    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);

    particles[i] = p;
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations)
{
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
  // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html
	std::vector<double> tmp_weights;
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  float gauss_norm = 1.0 / (2 * M_PI * std_x * std_y);

  for (unsigned i=0; i < num_particles; i++) {
    particles[i].weight = 1.0f;
    std::vector<Map::single_landmark_s> valid_landmarks;

    for (auto landmark : map_landmarks.landmark_list) {
      if (fabs(landmark.x_f-particles[i].x) <= sensor_range && fabs(landmark.y_f-particles[i].y) <= sensor_range) {
        valid_landmarks.push_back(landmark);
      }
    }

    std::vector<double> sense_x;
    std::vector<double> sense_y;
    std::vector<int> associations;

    for (auto obs : observations) {
      LandmarkObs world_observation;
      Particle p = particles[i];

      world_observation.x = p.x + (cos(p.theta) * obs.x - sin(p.theta) * obs.y);
      world_observation.y = p.y + (sin(p.theta) * obs.x + cos(p.theta) * obs.y);
      sense_x.push_back(world_observation.x);
      sense_y.push_back(world_observation.y);
      Map::single_landmark_s nearest_landmark;

      double min_dist = std::numeric_limits<double>::max();

      for (auto landmark : valid_landmarks) {
        double distance = dist(world_observation.x, world_observation.y, landmark.x_f, landmark.y_f);
        if (distance < min_dist) {
          nearest_landmark.id_i = landmark.id_i;
          nearest_landmark.x_f = landmark.x_f;
          nearest_landmark.y_f = landmark.y_f;
          min_dist = distance;
        }
      }

      associations.push_back(nearest_landmark.id_i);
      particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);

      float mu_x = nearest_landmark.x_f;
      float mu_y = nearest_landmark.y_f;
      float diff_x = world_observation.x - mu_x;
      float diff_y = world_observation.y - mu_y;

      float weight = gauss_norm * exp(- 0.5 * pow(diff_x/std_x,2) - 0.5 * pow(diff_y/std_y,2));
      particles[i].weight *= weight;
    }

    tmp_weights.push_back(particles[i].weight);
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample()
{
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  discrete_distribution<int> distribution(weights.begin(), weights.end());

  vector<Particle> resample_particles;
  for (unsigned i=0; i < num_particles; i++) {
    resample_particles.push_back(particles[distribution(gen)]);
  }
  particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
