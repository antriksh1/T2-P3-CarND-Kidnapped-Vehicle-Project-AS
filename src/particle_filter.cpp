/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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
#include <iostream>

#include "helper_functions.h"

using std::string;
using std::vector;

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 20;  // TODO: Set the number of particles

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  std::default_random_engine gen;

  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  for(int i = 0; i < num_particles; i++) {
    double sample_x = dist_x(gen);
    double sample_y = dist_y(gen);
    double sample_theta = dist_theta(gen);

    Particle particle;
    particle.id = i;
    particle.x = sample_x;
    particle.y = sample_y;
    particle.theta = sample_theta;
    particle.weight = 1.0;

    particles.push_back(particle);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  std::default_random_engine gen;

  std::normal_distribution<double> dist_x(0.0, std_x);
  std::normal_distribution<double> dist_y(0.0, std_y);
  std::normal_distribution<double> dist_theta(0.0, std_theta);
  // cout << "here p1" << endl;
  for(int i = 0; i < num_particles; i++) {
    // we will get NaN due to division by zero if this check not done
    if(yaw_rate > -0.0001 && yaw_rate < 0.0001) {
      particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
      particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
    } else {
      particles[i].x = particles[i].x + (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta)) + dist_x(gen);
      particles[i].y = particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)) + dist_y(gen);
      particles[i].theta = particles[i].theta + yaw_rate*delta_t + dist_theta(gen);  
    }
  }
  // cout << "here p1" << endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  // cout << "here da1" << endl;
  for(int i = 0; i < observations.size(); i++) {
    int closestPredictedId = -1;
    double closestDistance = std::numeric_limits<double>::max();
    
    for(int j = 0; j < predicted.size(); j++) {
      double currentDistance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if(currentDistance < closestDistance) {
        closestPredictedId = predicted[j].id;
        closestDistance = currentDistance;
      }
    }

    // cout << "here da2" << endl;
    observations[i].id = closestPredictedId;
  }
  // cout << "here da3" << endl;

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

  // cout << "here uw1" << endl;
  for(int i = 0; i < num_particles; i++) {

    //////////////////////////////////////////////////////////////
    // Find landmarks in range of this particle
    //////////////////////////////////////////////////////////////
    // cout << "here uw2" << endl;
    vector<LandmarkObs> landmarksInRange;
    for(int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      double dist = (particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
      if(dist <= sensor_range) {
        LandmarkObs landmarkObs;
        landmarkObs.id = map_landmarks.landmark_list[j].id_i;
        landmarkObs.x = map_landmarks.landmark_list[j].x_f;
        landmarkObs.y = map_landmarks.landmark_list[j].y_f;

        landmarksInRange.push_back(landmarkObs);
      }
    }

    // cout << "here uw3" << endl;
    //////////////////////////////////////////////////////////////
    // Transform - Rotate & Translate - particle to map coordinates
    //////////////////////////////////////////////////////////////
    vector<LandmarkObs> observationsOnMapCoordinates;
    double x_part = particles[i].x;
    double y_part = particles[i].y;
    double theta = particles[i].theta;
    for(int j = 0; j < observations.size(); j++) {
      double x_obs = observations[j].x;
      double y_obs = observations[j].y;

      double x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      double y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

      // if(y_map > 100.0) {
      //   cout << "x_obs: " << x_obs << ", ";
      //   cout << "y_obs: " << x_obs << endl;
      //   cout << "x_part: " << x_part << ", ";
      //   cout << "y_part: " << y_part << ", ";
      //   cout << "theta: " << theta << endl;
      //   cout << "x_map: " << x_map << ", ";
      //   cout << "y_map: " << x_map << endl;
      // }

      LandmarkObs landmarkObs;
      landmarkObs.id = observations[j].id;
      landmarkObs.x = x_map;
      landmarkObs.y = y_map;

      observationsOnMapCoordinates.push_back(landmarkObs);
    }
    
    // cout << "here uw4" << endl;
    //////////////////////////////////////////////////////////////
    // DataAssociation of Observation to Landmark
    //////////////////////////////////////////////////////////////
    dataAssociation(landmarksInRange, observationsOnMapCoordinates);

    // cout << "here uw5" << endl;
    //////////////////////////////////////////////////////////////
    // Use Multi-Variate Gaussian to update the weights
    //////////////////////////////////////////////////////////////
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];

    // initial weight - which will be multiplied to
    particles[i].weight = 1.0;

    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    // for all observations
    // cout << "here uw6" << endl;
    for(int j = 0; j < observationsOnMapCoordinates.size(); j++) {
      double x_obs = observationsOnMapCoordinates[j].x;
      double y_obs = observationsOnMapCoordinates[j].y;

      //obtain nearest landmark & get its x and y to be mu_x and mu_y
      int nearestLandmarkId = observationsOnMapCoordinates[j].id;
      double mu_x = 0.0;
      double mu_y = 0.0;
      for(int k = 0; k < landmarksInRange.size(); k++) {
        if(landmarksInRange[k].id == nearestLandmarkId) {
          mu_x = landmarksInRange[k].x;
          mu_y = landmarksInRange[k].y;
          break;
        }
      }
      
      double prob = multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);

      // cout << "sig_x: " << sig_x << ", sig_y: " << sig_y << ", x_obs: " << x_obs << ", y_obs: " << y_obs << ", mu_x: " << mu_x << ", mu_y: " << mu_y << endl;
      // if(particles[i].y > 100.0 || prob < 0.0001) { 
      //   cout << "BAD" << endl;
      // }
      // cout << "prob: " << prob << endl; 
      
      // We can get weights to be 0 otherwise
      if(prob < 0.0001) {
        particles[i].weight *= 0.0001;  
      } else {
        particles[i].weight *= prob;
      }
      
      // keep assosiations
      associations.push_back(observationsOnMapCoordinates[j].id);
      sense_x.push_back(observationsOnMapCoordinates[j].x);
      sense_y.push_back(observationsOnMapCoordinates[j].y);
    }

    // cout << "here uw7" << endl;
    // cout << "i: " << i << ", \tweight: " << particles[i].weight << " \t(" << particles[i].x << "," << particles[i].y << " \t@ " << particles[i].theta << ")" << endl;
    weights.push_back(particles[i].weight);

    SetAssociations(particles[i], associations, sense_x, sense_y);
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> particles2;

  std::random_device rd;
  std::mt19937 gen(rd());
  
  // cout << "here r1" << endl;
  // get max weight to use to genetate beta and distribution
  double maxWeight = 0.0;
  for(int i = 0; i < num_particles; i++) {
    if(particles[i].weight > maxWeight) {
      maxWeight = particles[i].weight;
    }
  }  

  // this does job for us - without using beta
  std::discrete_distribution<> d(weights.begin(), weights.end());

  // cout << "here r2" << endl;    
  for(int i = 0; i < num_particles; i++) {
    int index = d(gen);
    Particle particle2 = particles[index];
    particles2.push_back(particle2);
  }

  // clean before next cycle - otherwise get segmentation fault :)
  particles.clear();
  weights.clear();

  // cout << "here r3" << endl;    
  particles = particles2;
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

  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
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