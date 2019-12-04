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
using std::numeric_limits;
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
      predict_x = particles[i].x + (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      predict_y = particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      predict_theta = particles[i].theta + yaw_rate*delta_t;
    }
    else if (yaw_rate == 0)
    {
      predict_x = particles[i].x + (velocity * delta_t) * (cos(particles[i].theta));
      predict_y = particles[i].y + (velocity * delta_t) * (sin(particles[i].theta));
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
  // Initialize number of observations and predicted measurements
  unsigned int num_observations = observations.size();
  unsigned int num_predictions = predicted.size();
  // id of particular predicted landmark
  int pred_landmark_id;
  // Initialize x, y, closest distance and current distance
  double delta_x = 0.0, delta_y = 0.0, closest_dist = 0.0, dist = 0.0;
  // Find the predicted landmark measurement closest to each observed
  // measurement, then assign observed measurement with id of predicted
  for(unsigned int i = 0; i < num_observations; ++i)
  {
    // Initialize closest distance to a huge number
    closest_dist = numeric_limits<double>::max();
    // Initialize predicted landmark identifier
    pred_landmark_id = 0;
    // check all predicted measurements until closest one to observed
    // measurement is found, then save the predicted id 
    for(unsigned int j = 0; j < num_predictions; ++j)
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

/**
 * updateWeights() function
 * 
 * Updates the weights of each particle using a multi-variate gaussian
 * distribution. Loop through every particle, transform each observation
 * from vehicle coordinate to map coordinate system, filter through
 * each map landmark keeping only map landmarks within sensor range of
 * current particle, associate each transformed observation to its
 * nearest predicted map landmark and calculate weight of each particle
 * using multi-variate gaussian distribution.
 */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  // Initialize Transform Observation related variables
  double x_part, y_part, x_obs, y_obs, theta;
  LandmarkObs t_observation;

  // Initialize filter map landmark related variables
  LandmarkObs curr_landmark;

  // Initialize Multivariate Gaussian related variables
  double sig_x, sig_y, x_tobs, y_tobs, mu_x, mu_y, weight, final_weight;
  unsigned int tobs_landmark_id;
  sig_x = std_landmark[0];
  sig_y = std_landmark[1];

  for(int i = 0; i < num_particles; ++i)
  {
    x_part = particles[i].x;
    y_part = particles[i].y;
    theta = particles[i].theta;

    // 1. Transform from Vehicle to Map Coordinates with current particle
    vector<LandmarkObs> transformed_observations;
    for(unsigned int j = 0; j < observations.size(); ++j)
    {
      x_obs = observations[i].x;
      y_obs = observations[i].y;

      t_observation.id = observations[i].id;
      // transform obs x car to map x coordinate
      t_observation.x = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      // transform obs y car to map y coordinate
      t_observation.y = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);   
      transformed_observations.push_back(t_observation); 
    }

    // 2. Keep only map landmarks within sensor range of current particle
    vector<LandmarkObs> predicted_landmarks;
    for(unsigned int m = 0; m < map_landmarks.landmark_list.size(); ++m)
    {
      curr_landmark.x = map_landmarks.landmark_list[m].x_f;
      curr_landmark.y = map_landmarks.landmark_list[m].y_f;
      curr_landmark.id = map_landmarks.landmark_list[m].id_i;
      double landmark_dist = dist(curr_landmark.x, curr_landmark.y, x_part, y_part);
      if (landmark_dist <= sensor_range)
      {
        predicted_landmarks.push_back(curr_landmark);
      }
    }

    // 3. Find predicted landmarks closest to each transformed observation
    dataAssociation(predicted_landmarks, transformed_observations);

    final_weight = 1.0;
    // 4. Calculate weight of each particle using Multivariate Gaussian
    for(unsigned int a = 0; a < transformed_observations.size(); ++a)
    {
      x_tobs = transformed_observations[a].x;
      y_tobs = transformed_observations[a].y;
      tobs_landmark_id = transformed_observations[a].id;

      // mapped predicted landmarks closest to tobs
      mu_x = predicted_landmarks[tobs_landmark_id].x;
      mu_y = predicted_landmarks[tobs_landmark_id].y;

      // calculate each transformed observation weight for current particle
      weight = multiv_prob(sig_x, sig_y, x_tobs, y_tobs, mu_x, mu_y);
      // calculate particle's final weight multiply each tobs weight
      final_weight *= weight;
    }
    particles[i].weight = final_weight;
    weights[i] = particles[i].weight;
  }
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