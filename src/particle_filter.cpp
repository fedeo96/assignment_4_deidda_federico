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

static default_random_engine gen;

VectorXd ParticleFilter::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rmse(2);
	rmse << 0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//TODO: accumulate squared residuals

    for(unsigned int i = 0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];  //differenza
        residual = residual.array()*residual.array();  //elevo al quadrato
        rmse += residual;  //sommatoria
    }

    rmse = rmse / estimations.size();   // 1/n
    rmse = rmse.array().sqrt();  //tutto al quadrato

	//return the result
	return rmse;
}


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	
	// TODO: Set the number of particles.
    num_particles = 100;

	// random Gaussian noise to each particle.
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for(int i=0;i<num_particles;i++){
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1; 
        particles.push_back(p);
    }
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

    for(int i=0;i<num_particles;i++){
		//TODO: CORRECT the prediction motion formula
		//REMINDER: particles[i].x or particles[i].theta gives you the x and orientation respectively 

        double x,y,theta;
        if (fabs(yaw_rate) < 0.00001) { //car going straight
            x = particles[i].x + velocity*delta_t*cos(yaw_rate);
            y = particles[i].y + velocity*delta_t*sin(yaw_rate);
			theta = particles[i].theta;
        }else{ //yaw rate is not equal to zero, car turning
            x = particles[i].x + (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            y = particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
            theta = particles[i].theta + (yaw_rate*delta_t);
        }

        //RANDOM GAUSSIAN NOISE
        // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
        //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
        //  http://www.cplusplus.com/reference/random/default_random_engine/
        normal_distribution<double> dist_x(x, std_pos[0]);
        normal_distribution<double> dist_y(y, std_pos[1]);
        normal_distribution<double> dist_theta(theta, std_pos[2]);

        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> inRangeLandmark, std::vector<LandmarkObs>& observations) {
    
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
    
	/*
		HINT:
		For each observation in J //use the observations.size() function
		Min_dist=INFINITY
		For each prediction in K
			diff_x=inRangeLandmark[K].x-observation[J].x
			diff_y=inRangeLandmark[K].y-observation[J].y
			dist=//FILL with euclidean distance
			if(dist<min_dist)
				min_dist=dist
				index_min=inRangeLandmark[K].id
			end if
		End for
		Observations[J].id=index_min
		End for
	*/

    int index_min = 0;

    for(int j = 0; j > observations.size(); j++){
        double dist_min = std::numeric_limits<double>::max();
        for(int k = 0; k < inRangeLandmark.size(); k++){
            double diff_x = inRangeLandmark[k].x - observations[j].x;
            double diff_y = inRangeLandmark[k].y - observations[j].y;
            double distance = sqrt((diff_x * diff_x) + (diff_y * diff_y));
            if(distance < dist_min){
                dist_min = distance;
                index_min = inRangeLandmark[k].id;
            }
            observations[j].id = index_min;
        }
    }
   
}

//TODO: This function transform a local (car) observation into a global (map) coordinates
LandmarkObs transformation(LandmarkObs observation, Particle p){
    LandmarkObs local;
    
    local.id = observation.id;
    //TODO Rotation performed in(3.33) : http://planning.cs.uiuc.edu/node99.html
    local.x = observation.x * cos(p.theta) - observation.y * sin(p.theta) + p.x;
    local.y = observation.x * sin(p.theta) + observation.y * cos(p.theta) + p.y;

    return local;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
    // In this function we update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html
    
    //create a predicted_landmark vector
    for(int i=0;i<particles.size();i++){

        std::vector<LandmarkObs> inRangeLandmark;
        for(int j=0;j<map_landmarks.landmark_list.size();j++){

            //we have to consider only landmarks within the range of the sensor
            double diff_x, diff_y, dist;
            diff_x = map_landmarks.landmark_list[j].x_f - particles[i].x;
            diff_y = map_landmarks.landmark_list[j].y_f - particles[i].y;

            if (fabs(diff_x) <= sensor_range && fabs(diff_y) <= sensor_range) 
                inRangeLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
        }
    

        //before applying the association we have to transform the observations in the global coordinates
        std::vector<LandmarkObs> transformed_observations;
        for(int k=0;k<observations.size();k++)
            transformed_observations.push_back(transformation(observations[k],particles[i]));

        //associate the landmarks to the observations
        dataAssociation(inRangeLandmark,transformed_observations);
	
        particles[i].weight = 1.0;

        //compute the probability
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;
            
            //get the associated landmark
            for (unsigned int p = 0; p < inRangeLandmark.size(); p++) {
                if (transformed_observations[k].id == inRangeLandmark[p].id) {
                    l_x = inRangeLandmark[p].x;
                    l_y = inRangeLandmark[p].y;
                }
            }			
			//how likely a set of landmarks measurements are, given a prediction state of the car (we compute the weight)
            double w = exp( -( pow(l_x-obs_x,2)/(2*pow(std_landmark[0],2)) + pow(l_y-obs_y,2)/(2*pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );

            //update the weights
            particles[i].weight = particles[i].weight*w;
        }

    }    
}

void ParticleFilter::resample() {

    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    uniform_int_distribution<int> dist_distribution(0,num_particles-1);
    double beta  = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> p;
    p.resize(num_particles);

    for(int i=0;i<num_particles;i++)
        weights.push_back(particles[i].weight);
																
    double max_w = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> uni_dist(0.0, max_w);

	//TODO: Write the resample algorithm, resample particles with replacement with probability proportional to their weight. 
	//HINT: uni_dist(gen) //gives you the random number between 0 and the maximum weight

    for (int i = 0; i < num_particles; ++i){
        beta = beta + uni_dist(gen)*2*max_w;
        while (weights[index] < beta){
            beta = beta - weights[index];
            index = (index + 1) % num_particles;
        }
        particles.push_back(p[index]);
    }
    particles = std::move(p);

    for (int i = 0; i < num_particles; ++i) {
        auto index = uni_dist(gen);
        p[i] = std::move(particles[index]);
    }
    particles = std::move(p);
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
