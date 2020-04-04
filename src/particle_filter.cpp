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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    num_particles = 100;
    particles.resize(num_particles);
    weights.resize(num_particles);

    // Standard deviations
    double std_x, std_y, std_theta;
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];
    
	//Gaussian
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    for(int i=0; i<num_particles; ++i){
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1.0;

        weights[i] = 1.0;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    //Standard deviations
    double std_x, std_y, std_theta;
    double x0, y0, theta0;
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];
    
	//Gaussian distributions
    default_random_engine gen;
    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_theta(0, std_theta);
    
    for(int i=0; i<num_particles;++i){
        //extract pose info
        x0 = particles[i].x;
        y0 = particles[i].y;
        theta0 = particles[i].theta;

        //predict next pose and avoid division by zero
        if(fabs(yaw_rate)>0.001){
            particles[i].x = x0 + velocity/yaw_rate * (sin(theta0+yaw_rate*delta_t) - sin(theta0));
            particles[i].y = y0 + velocity/yaw_rate * (cos(theta0) - cos(theta0+yaw_rate*delta_t));
            particles[i].theta = theta0 + yaw_rate*delta_t;
        } 
        else{
            particles[i].x = x0 + velocity*delta_t*cos(theta0);
            particles[i].y = y0 + velocity*delta_t*sin(theta0);
        }
        
        //Random gaussian noise
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

    double xm, ym, xc, yc;
    double exponent, mu_x, mu_y;

    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
    double gauss_den_x = 2 * pow(sig_x,2);
    double gauss_den_y = 2 * pow(sig_y,2);

    vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;

    double xp, yp, thetap, weight;
    double landmark_dist, obs_to_mark;

    for(int i=0; i<num_particles; ++i){
        xp = particles[i].x;
        yp = particles[i].y;
        thetap = particles[i].theta;
        weight = 1.0;

        vector<LandmarkObs> nearby_landmarks;
        LandmarkObs nearby_landmark;

        // get the nearby landmarks
        for(unsigned int k=0; k<landmarks.size(); ++k){
            //distance between particle and landmark
            landmark_dist = dist(xp, yp, landmarks[k].x_f, landmarks[k].y_f);

            // select landmarks in sensor range
            if(landmark_dist < sensor_range){
                nearby_landmark.id = landmarks[k].id_i;
                nearby_landmark.x = landmarks[k].x_f;
                nearby_landmark.y = landmarks[k].y_f;
                nearby_landmarks.push_back(nearby_landmark);
            } 
        }

        for(unsigned int j=0; j<observations.size(); ++j){
            //transform observation position from vehicle to map coordinate
            xc = observations[j].x;
            yc = observations[j].y;
            xm = xp + cos(thetap)*xc - sin(thetap)*yc;
            ym = yp + sin(thetap)*xc + cos(thetap)*yc;

            double obs_to_mark_min = 999;
            int min_idx = 0;
            //get the nearest landmark index
            for(unsigned int n=0; n<nearby_landmarks.size(); ++n){
                obs_to_mark = dist(xm, ym, nearby_landmarks[n].x, nearby_landmarks[n].y);
                if(obs_to_mark < obs_to_mark_min){
                    min_idx = nearby_landmarks[n].id;
                    obs_to_mark_min = obs_to_mark;
                }
            }
            //get the nearest landmark postion
            for(unsigned int n=0; n<nearby_landmarks.size(); ++n){
                if(nearby_landmarks[n].id == min_idx){
                    mu_x = nearby_landmarks[n].x;
                    mu_y = nearby_landmarks[n].y;
                    break;
                }
            }

            //Multivariate-Gaussian probability 
            exponent = pow(xm-mu_x,2)/gauss_den_x + pow(ym-mu_y,2)/gauss_den_y;
            weight *= gauss_norm * exp(-exponent);
        }
        particles[i].weight = weight;
        weights[i] = weight;
    }
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    vector<Particle> new_particles (num_particles);

    double beta = 0;
    int index = rand() % num_particles;
    double w_max = *max_element(weights.begin(), weights.end());
    
    for(int i=0; i<num_particles; ++i){

        beta += (rand() / (RAND_MAX + 1.0)) * (2*w_max);
        while(weights[index]<beta){
            beta -= weights[index];
            index = (index+1) % num_particles;
        }
        new_particles[i] = particles[index];
    }
    particles = new_particles;
}


Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
