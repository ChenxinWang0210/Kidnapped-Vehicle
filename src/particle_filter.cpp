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
  
  // number of particles (tuning parameter)
  num_particles = 100;
  
  default_random_engine gen;
  
  // normal (Gaussian) distribution for x, y, theta
  normal_distribution<double> dist_x(x,std[0]);
  normal_distribution<double> dist_y(y,std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);
  
  // initialize particle positions and weights
  for ( unsigned int i = 0; i<num_particles; i++) 
  {
    Particle sample;
    sample.x =  dist_x(gen);
    sample.y =  dist_y(gen);
    sample.theta =  dist_theta(gen);
    sample.weight = 1.0;
    
    particles.push_back(sample);
    weights.push_back(sample.weight);
  }
  
is_initialized = true; 
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  	default_random_engine gen;
  
  	 // random Gaussian noise distribution for x, y, theta
    normal_distribution<double> dist_x(0.0,std_pos[0]);
    normal_distribution<double> dist_y(0.0,std_pos[1]);
    normal_distribution<double> dist_theta(0.0,std_pos[2]);
  
  	if(fabs(yaw_rate)<0.00001)
    {
      // Going straight
        for ( unsigned int i = 0; i<num_particles; i++) 
        {
          // predicted mean position and heading
          double xf = particles[i].x + velocity * delta_t *  cos ( particles[i].theta ) ;
          double yf = particles[i].y + velocity * delta_t *  sin ( particles[i].theta ) ;
          double thetaf = particles[i].theta + yaw_rate * delta_t ;
          
          // add noise
          particles[i].x = xf + dist_x(gen);
          particles[i].y = yf + dist_y(gen);
          particles[i].theta = thetaf + dist_theta(gen);
        }
    }
    else 
    {
      // turning 
        for ( unsigned int i = 0; i<num_particles; i++) 
      {
        // predicted mean position and heading
        double thetaf = particles[i].theta + yaw_rate * delta_t ;  
        double xf = particles[i].x + velocity / yaw_rate * ( sin( thetaf ) - sin( particles[i].theta ) );
        double yf = particles[i].y + velocity / yaw_rate * ( cos ( particles[i].theta ) - cos( thetaf ) );
        
		// add noise
        particles[i].x = xf + dist_x(gen);
        particles[i].y = yf + dist_y(gen);
        particles[i].theta = thetaf + dist_theta(gen);

      }
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  // associate each observation to the closest landmark
  for ( unsigned int i =0; i< observations.size(); i++)
  {
    // Initialize min distance as a big number
    float min_dist = 100000000;
    float px = observations[i].x;
    float py = observations[i].y;
    // Initialize the closet landmark (not exist)  to the observation
    int map_id = -1;
    
    for (unsigned int j =0; j< predicted.size(); j++)
    {
       float distance = dist(px, py, predicted[j].x, predicted[j].y);
      // If the "distance" is less than min_dist, update min_dist and store the map_id.
      if(distance<min_dist)
      {
        min_dist = distance;
        map_id = predicted[j].id;
      }
    }
    // associate the observation id with the closest landmark id;
    observations[i].id = map_id; 
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
  
  
  	// mult-variate Gaussian distribution constants
  	float c0 = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);
  	float c1 = 1/(2*pow(std_landmark[0],2));
  	float c2 = 1/(2*pow(std_landmark[1],2));
  
  		for ( unsigned int i = 0; i<num_particles; i++) 
        { 
          float px = particles[i].x ; 
          float py = particles[i].y ; 
          float ptheta = particles[i].theta ; 
          
          std::vector<LandmarkObs> predicted;
          
          	// Check whether one landmark is within sensor range, if yes, put it into the "predicted" landmark vector
          
          for (unsigned int k = 0; k< map_landmarks.landmark_list.size(); k++)
          {
            float lm_x =map_landmarks.landmark_list[k].x_f;
            float lm_y =map_landmarks.landmark_list[k].y_f;
            int lm_id = map_landmarks.landmark_list[k].id_i;
            float distance = dist(px,py,lm_x, lm_y);
            
            if (distance< sensor_range)
            {
              LandmarkObs predicted_landmark;
              
              predicted_landmark.x = lm_x;
              predicted_landmark.y = lm_y;
              predicted_landmark.id = lm_id;
              predicted.push_back(predicted_landmark);
            }
          }
          
          
          std::vector<LandmarkObs> observed;
          // Transform observations to map's coordinates
          
          for(unsigned int j = 0; j< observations.size(); j++)
          {
            LandmarkObs observed_map_landmark;
            
            observed_map_landmark.x = px + observations[j].x * cos ( ptheta ) - observations[j].y * sin ( ptheta ) ;
            observed_map_landmark.y = py + observations[j].x * sin ( ptheta ) + observations[j].y * cos ( ptheta ) ;
            
            observed.push_back(observed_map_landmark);
          }
          
          // associate observations to nearest landmark
          dataAssociation(predicted, observed); 
          
          
          // Reset weights
          particles[i].weight = 1.0; 
          
          // optional : record associations
          std::vector<int> associations;
          std::vector<double> sense_x;
		  std::vector<double> sense_y;
          
            for (unsigned int j =0; j<observed.size(); j++)
            {
              // optional : record associations
              associations.push_back(observed[j].id);
              sense_x.push_back(observed[j].x);
              sense_y.push_back(observed[j].y);
              
              float x_m = map_landmarks.landmark_list[observed[j].id-1].x_f ;
              float y_m = map_landmarks.landmark_list[observed[j].id-1].y_f ;
              // calculate particle weight
              particles[i].weight *= c0*exp(-c1*pow((observed[j].x-x_m),2) - c2 * pow((observed[j].y-y_m),2));
            }
          
          // optional: record associations
          SetAssociations(particles[i], associations, sense_x, sense_y);
        }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
// set maxmium weight to be 0.0
  float max_weight = 0.0 ; 
  
  for ( unsigned int i = 0; i<num_particles; i++)
  {
    // update particle weights
    weights[i] = particles[i].weight ;
    // get max_weight
    if (particles[i].weight>max_weight)
    {
      max_weight = particles[i].weight;
    }
  }
  
  default_random_engine gen;
  // create uniform distributions 
  uniform_int_distribution<int> pindex_distribution(0,num_particles-1);
  uniform_real_distribution<float> beta_increment_dist(0.0, 2*max_weight);
  
  // generate index and initialize beta
  unsigned int index = pindex_distribution(gen);
  float beta =0.0;
 
  std::vector<Particle> resample_ps;
 
  // resampling wheel 
  for ( unsigned int i = 0; i<num_particles; i++)
  {
    beta = beta + beta_increment_dist(gen);
    while(weights[index]<beta)
    {
      beta = beta - weights[index];
      index = (index + 1)%num_particles;
    }
    resample_ps.push_back(particles[index]);
  }
  
  particles = resample_ps;
  
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
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
