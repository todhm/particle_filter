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
#include <numeric>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = 100;
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];
    
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);
    
    for (int i = 0; i < num_particles; i++){
        double sample_x = dist_x(gen);
        double sample_y = dist_y(gen);
        double sample_theta = dist_theta(gen);

        Particle p = {i, sample_x, sample_y,sample_theta, double(1/ num_particles)};
        particles.push_back(p);
        weights.push_back(double(1/ num_particles));

    
    }
    is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];
    
    default_random_engine gen;
    for(int i = 0 ; i < num_particles ; i++){

        double theta = particles[i].theta;
        normal_distribution<double> dist_x(particles[i].x, std_x);
        normal_distribution<double> dist_y(particles[i].y, std_y);
        normal_distribution<double> dist_theta(particles[i].theta, std_theta);
        
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);

        if(fabs(yaw_rate) >= 1e-6 ){
          particles[i].x +=  velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
          particles[i].y +=  velocity / yaw_rate * ( cos(theta)- cos(theta + yaw_rate * delta_t));
          particles[i].theta  += yaw_rate * delta_t;
        }
        else{
            particles[i].x +=  velocity * delta_t * cos(theta);
            particles[i].y +=  velocity  * delta_t * sin(theta);
        }
        
           }
}




void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

    double sum_weight = 0.0 ;
    for(int i =0; i< num_particles; i ++){
        std::vector<LandmarkObs> associated;
        std::vector<LandmarkObs> transformed;
        for(int j= 0 ; j < observations.size(); j++){
            //transforming observed coordinate from sensor to map's coordinate system.
            LandmarkObs obs;
            obs.x  = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
            obs.y  = particles[i].y + observations[j].y * cos(particles[i].theta) + observations[j].x * sin(particles[i].theta);
            transformed.push_back(obs);
            
            //find the nearest landmark from transformed map in each observation.
            LandmarkObs obs_map;
            double min_distance = 0;
            for(int k = 0 ; k < map_landmarks.landmark_list.size();k ++){
                double map_x = map_landmarks.landmark_list[k].x_f;
                double map_y = map_landmarks.landmark_list[k].y_f;
                if(k ==0){
                    double distance = dist(obs.x,obs.y,map_x, map_y) ;
                    min_distance = distance;
                }
                //Find out if the landmark located inside the sensor_range.
                if(map_x >= obs.x - sensor_range && map_x <= obs.x + sensor_range
                   && map_y  >= obs.y - sensor_range && map_y <= obs.y  + sensor_range){
                    double distance = dist(obs.x,obs.y,map_x, map_y) ;
                    if(distance < min_distance){
                        min_distance = distance;
                        obs_map.x = map_x;
                        obs_map.y = map_y;
                    }
                }
            }
           associated.push_back(obs_map);
        }
        
        //calculate weight by multiplying all the multi-variate Gaussian distribution value from each obs;
        double weight = 1;
        for(int l = 0 ; l < associated.size();l++){
            
            //coordinate of nearest landmark
            double x = associated[l].x;
            double y = associated[l].y ;
            
            //coordinate of transformed value.
            double mu_x = transformed[l].x;
            double mu_y = transformed[l].y;
            
            double xdiff = x - mu_x;
            double ydiff = y - mu_y;
            double denominator = 1 / (2* M_PI *std_landmark[0] * std_landmark[1]);
            double power = -0.5 * (pow(xdiff,2.0)/(pow(std_landmark[0],2.0)) + pow(ydiff,2.0)/(pow(std_landmark[1],2.0)));
            double temp_weight = denominator * exp(power);
            if(temp_weight > 0){
                      weight *= temp_weight;
            }
        }
        particles[i].weight = weight;
        weights[i] = weight;
        sum_weight += weight;
    }
    for (int i =0 ; i< num_particles; i ++ ){
        particles[i].weight /= sum_weight;
        weights[i] = particles[i].weight;
    }
}



void ParticleFilter::resample() {
    

    std::vector<Particle> particles_temp;
    default_random_engine gen;
    discrete_distribution<int> prob(weights.begin(), weights.end());

    for(int i = 0 ; i < particles.size() ; i++){
        particles_temp.push_back(particles[prob(gen)]);
    }
    particles = particles_temp;
    

}



 
Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
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
