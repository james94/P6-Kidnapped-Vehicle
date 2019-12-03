/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Updated on: Dec 2, 2019
 * Contributor: James Medel
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;

/**
 * init() function
 * 
 * Initializes Particle Filter by initializing all the filter's particles and
 * sampling from a gaussian distribution taking into account gaussian sensor
 * noise around the initial gps position estimate and initial heading estimate.
 */
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  default_random_engine gen;
  // Set the number of particles
  num_particles = 10;

  // Create normal (Gaussian) distribution for particle x, y, theta
  normal_distribution<double> dist_particle_x(x, std[0]);
  normal_distribution<double> dist_particle_y(y, std[1]);
  normal_distribution<double> dist_particle_theta(theta, std[2]);

  // Initialize all particles to first position and all weights to 1
  Particle particle;
  particle.weight = 1;

  for(int i = 0; i < num_particles; ++i)
  {
    particle.id = i;
    // Add random Gaussian noise to each particle's x,y,theta using gen
    particle.x = dist_particle_x(gen);
    particle.y = dist_particle_y(gen);
    particle.theta = dist_particle_theta(gen);
    particles.push_back(particle);
    weights.push_back(particle.weight);
  }

  // particle filter is initialized
  is_initialized = true;
}

/**
 * prediction() function
 * 
 * Predicts the vehicle's position at the next time step
 * by updating based on yaw rate and velocity while accounting
 * for Gaussian sensor noise. Adds measurements to each particle
 * using prediction step equation, then adds random Gaussian noise.
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  double predict_x, predict_y, predict_theta;

  default_random_engine gen;

  for(int i = 0; i < num_particles; ++i)
  {
    // Calculate Prediction Step to add measurements to particles
    if (yaw_rate != 0)
    {
      predict_x = particles[i].x + (velocity/yaw_rate) *
                  (sin(particles[i].theta + yaw_rate*delta_t) -
                  sin(particles[i].theta));
      predict_y = particles[i].y + (velocity/yaw_rate) *
                  (cos(particles[i].theta) -
                  cos(particles[i].theta + yaw_rate*delta_t));
      predict_theta = particles[i].theta + yaw_rate*delta_t;
    }
    else if (yaw_rate == 0)
    {
      predict_x = particles[i].x + (velocity * delta_t) *
                  (cos(particles[i].theta))
      predict_y = particles[i].y + (velocity * delta_t) *
                  (sin(particles[i].theta))
      predict_theta = particles[i].theta;
    }

    // Create normal (Gaussian) distribution for particle predict x, y, theta
    normal_distribution<double> dist_predict_x(predict_x, std_pos[0]);
    normal_distribution<double> dist_predict_y(predict_y, std_pos[1]);
    normal_distribution<double> dist_predict_theta(predict_theta, std_pos[2]);

    // Add random Gaussian noise to each particle's predict x,y,theta using gen
    particles[i].x = dist_predict_x(gen);
    particles[i].y = dist_predict_y(gen);
    particles[i].theta = dist_predict_theta(gen);
  }
}

/**
 * dataAssociation() function
 * 
 * Finds the predicted landmark measurement closest to each observed
 * measurement, then assigns the observed measurement with the
 * particular id of the predicted landmark in the map. Thus, each
 * observation is associated with a predicted landmark identifier.
 */ 
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  // id of particular predicted landmark
  int pred_landmark_id;
  // Initialize x, y, closest distance and current distance
  double delta_x = 0.0, delta_y = 0.0, closest_dist = 0.0, dist = 0.0;
  // Find the predicted landmark measurement closest to each observed
  // measurement, then assign observed measurement with id of predicted
  for(int i = 0; i < observations.size(); ++i)
  {
    // Initialize closest distance to a huge number
    closest_dist = numeric_limits<double>::max();
    // Initialize predicted landmark identifier
    pred_landmark_id = 0;
    // check all predicted measurements until closest one to observed
    // measurement is found, then save the predicted id 
    for(int j = 0; j < predicted.size(); ++j)
    {
      delta_x = observations[i].x - predicted[j].x;
      delta_y = observations[i].y - predicted[j].y;
      dist = sqrt( pow(delta_x, 2.0) + pow(delta_y, 2.0) );
      if (dist < closest_dist)
      {
        closest_dist = dist;
        pred_landmark_id = predicted[j].id;
      }
    }
    // assign observed measurement with id of predicted landmark in the map
    observations[i].id = pred_landmark_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}