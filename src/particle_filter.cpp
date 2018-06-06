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

#define EPS 0.00001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (is_initialized) {
		return;
	}	

	default_random_engine gen;

	num_particles = 100;
	
	// set standard deviations for x, y, and theta
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	// normal distributions for x, y, and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// initialize each particle
	for (int i = 0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

        default_random_engine gen;
	
	// set standard deviations for x, y, and theta
        double std_x = std_pos[0];
        double std_y = std_pos[1];
        double std_theta = std_pos[2];

        // normal distributions for x, y, and theta
        normal_distribution<double> dist_x(0, std_x);
        normal_distribution<double> dist_y(0, std_y);
        normal_distribution<double> dist_theta(0, std_theta);
	
	// prediction
	for (int i = 0; i < num_particles; i++) {
		double theta = particles[i].theta;
		
		// zero check
		if ( fabs(yaw_rate) < EPS ) {
			particles[i].x += delta_t * velocity * cos(theta);
			particles[i].y += delta_t * velocity * sin(theta);
		} else {
			// prediction formula
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}
		
		// add noise to it
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen); 
	}	
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); i++) {
		double threshold_dist = numeric_limits<double>::max();
		int item_id = -1;

		for (int j = 0; j < predicted.size(); j++) {
			double dx = observations[i].x - predicted[j].x;
			double dy = observations[i].y - predicted[j].y;
			double dist = dx * dx + dy * dy;
			
			if (dist < threshold_dist) {
				threshold_dist = dist;
				item_id = predicted[j].id;
			}
		}
		observations[i].id = item_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
	double range_mark_std = std_landmark[0];
	double bearing_mark_std = std_landmark[1];
	
	for (int i = 0; i < num_particles; i++) {
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		
		// find usable landmarks	
		vector<LandmarkObs> landmarks_vec;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			double lm_x = map_landmarks.landmark_list[j].x_f;
			double lm_y = map_landmarks.landmark_list[j].y_f;
			int id = map_landmarks.landmark_list[j].id_i;
			double dx = x - lm_x;
			double dy = y - lm_y;
			double sensor_range_sq = sensor_range * sensor_range;
			if (dx * dx + dy * dy <= sensor_range_sq) {
				LandmarkObs single_landmark{id, lm_x, lm_y};
				landmarks_vec.push_back(single_landmark);
			}
		}
		
		// observation coordinate transformation
		vector<LandmarkObs> transformed_obs;
		for (int j = 0; j < observations.size(); j++) {
			// expand the transformation matrix
			double x_trans = cos(theta) * observations[j].x - sin(theta) * observations[j].y + x;
			double y_trans = sin(theta) * observations[j].x + cos(theta) * observations[j].y + y;
			LandmarkObs trans_ob{observations[j].id, x_trans, y_trans};
			transformed_obs.push_back(trans_ob);
		}
		
		// associtate data via the implementation above
		dataAssociation(landmarks_vec, transformed_obs);
		
		// calculate weights for each particle
		particles[i].weight = 1.0;
		for (int j = 0; j < transformed_obs.size(); j++) {
			double ob_x = transformed_obs[j].x;
			double ob_y = transformed_obs[j].y;
			double mark_x, mark_y;
			int mark_id = transformed_obs[j].id;
			
			// group observation into landmarks
			bool is_found = false;
			int k = 0;
			while (!is_found && k < landmarks_vec.size()) {
				if (landmarks_vec[k].id == mark_id) {
					is_found = true;
					mark_x = landmarks_vec[k].x;
					mark_y = landmarks_vec[k].y;
				}
				k++;
			}
			double dx = ob_x - mark_x;
			double dy = ob_y - mark_y;
			double gauss_norm = 1/(2*M_PI*range_mark_std*bearing_mark_std);
			double exponent = dx*dx/(2*range_mark_std*range_mark_std) + dy*dy/(2*bearing_mark_std*bearing_mark_std);
			double weight = gauss_norm * exp(-exponent);

			// check weight
			if (weight == 0) {
				weight = EPS;
			}
			particles[i].weight *= weight;
		}
	}	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
        default_random_engine gen;

	vector<double> weights;
	double max_weight = numeric_limits<double>::min();
	for (int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
		if (particles[i].weight > max_weight) {
			max_weight = particles[i].weight;
		}
	}
	
	uniform_real_distribution<double> dist_real(0.0, max_weight);
	uniform_int_distribution<double> dist_int(0, num_particles-1);
	int index = dist_int(gen);
	double beta = 0.0;
	vector<Particle> resample_particle_vec;
	for (int i = 0; i < num_particles; i++) {
		beta += dist_real(gen) * 2.0;
		while (beta > weights[index]) {
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resample_particle_vec.push_back(particles[index]);
	}
	particles = resample_particle_vec;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
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
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
