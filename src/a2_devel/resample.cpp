#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <random>
#include <chrono>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

// ============================================================================================================================
// RESAMPLE PARTICLES

// Helper Function: Random Number Generator
// CHECK
double get_random_double(int min, int max)
{
    // Set Bounds
    double lower_bound = min;
    double upper_bound = max;

    // Build Distribution of Range
    std::uniform_real_distribution<double> distribution(lower_bound,upper_bound);

    // Pseudo-random number engine initialization
    std::default_random_engine re;

    // ReSeed
    re.seed(std::chrono::system_clock::now().time_since_epoch().count());

    // Sample
    double a_random_double = distribution(re);

    return a_random_double;
}

// Main Function: Resample Each Particle
std::vector <Particle> resample(std::vector <Particle> const &particle_vec)
{
    
    // Predefine the Number of Resamples; set_parameter
    int num_of_resamples {100};

    // Initializations
    std::vector <Particle> reduced_particle_vec; // return vector (vector of the kept particles)
    double weight_sum {0};                       // comparison variable 
    double total_weight {0};                     // total weight of all particles
    double random_num {0};                       // normal random number variable
    int k {0};                                   // edge case variable

    // Get Length of Input Vector for Looping
    int input_vec_length = particle_vec.size();

    // Compute Sum of Particle Weights to Get Upper Container Bound
    for (auto get_particle: particle_vec)
        total_weight += get_particle.weight;
    
    // Main Loop; RESAMPLE
    for (int i {0}; i < num_of_resamples; i++)
    {   
        // reset comparison variable
        weight_sum = 0;

        // get a random number
        random_num = get_random_double(0.00,total_weight);
        std::cout << "\n[Iter: " << i << "]\n Random Number: " << random_num << std::endl; // _____________ debug

        // loop through each particle weight/bucket
        for (int j {0}; j < input_vec_length; j++)
        {
            // zero case
            if (random_num == 0.00) 
            {
                reduced_particle_vec.push_back(particle_vec[0]);
                //std::cout << " Particle (" << iter << ") equaled min; added to output vec" << std::endl; // debug
                break;
            }
            // max case
            else if (random_num == total_weight)
            {
                reduced_particle_vec.push_back(particle_vec[input_vec_length-1]);
                //std::cout << " Particle (" << iter << ") equaled max; added to output vec" << std::endl; // debug
                break;
            }
            // middle bucket cases; keep adding buckets if the random number is not equal
            else if (random_num > weight_sum)
            {
                // Add a weight
                weight_sum += particle_vec[j].weight;

                // Check if random number is on the edge of two buckets
                if (random_num == weight_sum)
                {
                    k = (particle_vec[j].weight <= particle_vec[j+1].weight) ? j+1 : j; 
                    reduced_particle_vec.push_back(particle_vec[k]);
                    //std::cout << " Particle (" << iter << ") on edge; added to output vec" << std::endl; // debug
                }
                // Check if random number is less than the next bucket, and add to output vector
                else if (random_num < weight_sum)
                {
                    reduced_particle_vec.push_back(particle_vec[j]);
                    //std::cout << " Particle (" << iter << ") added to output vec" << std::endl; // ________ debug
                }
            }
        }
    }
    return reduced_particle_vec;
}

// ============================================================================================================================
// FIND BEST GUESS LOCATION

// Main Function: Average over the every particle
Particle get_optimal_particle(std::vector <Particle> reduced_particle_vec)
{
    // Get Total Length of Input Vector
    int vector_length = reduced_particle_vec.size();

    // Initializations
    double sum_x {0};
    double sum_y {0};
    double sum_cos_theta {0};
    double sum_sin_theta {0};
    Particle optimal_particle;

    // Summations for all variables
    for (auto vec:reduced_particle_vec)
    {
        sum_x += vec.loc.x();
        sum_y += vec.loc.y();
        sum_cos_theta += cos(vec.angle);
        sum_sin_theta += sin(vec.angle);
    }

    // Averages for x, y, and theta; weight hard coded to 1
    optimal_particle.loc.x() = sum_x/vector_length;
    optimal_particle.loc.y() = sum_y/vector_length;
    optimal_particle.angle = atan2(sum_sin_theta/vector_length,sum_cos_theta/vector_length);
    optimal_particle.weight = 1;
    
    return optimal_particle;
}


int main(void)
{
    // Example of Updated Weighted Particles
    Particle particle_dict_1 {Eigen::Vector2f (1.0,2.0), 0, 0.320};
    Particle particle_dict_2 {Eigen::Vector2f (1.4,2.3), 0.1, 0.60};
    Particle particle_dict_3 {Eigen::Vector2f (0.8,1.8), 0.4, 0.1};
    Particle particle_dict_4 {Eigen::Vector2f (0.9,2.1), 0.01, 0.7};
    Particle particle_dict_5 {Eigen::Vector2f (0.87,1.87), 0.01, 0.9};
    Particle particle_dict_6 {Eigen::Vector2f (1.7,0.98), 0.01, 0.8};

    // Build Sample Vector of Particle Vectors
    std::vector <Particle> particle_vec;
    particle_vec.push_back(particle_dict_1);
    particle_vec.push_back(particle_dict_2);
    particle_vec.push_back(particle_dict_3);
    particle_vec.push_back(particle_dict_4);
    particle_vec.push_back(particle_dict_5);
    particle_vec.push_back(particle_dict_6);
    
    int entry {0};
    std::cout << "********** Initial Vector ********" << std::endl;
    double total_weight {0};
    for(auto vector: particle_vec)
    {
        std::cout << "[Particle: " << entry << "]"
                  << "\n x: " << vector.loc.x()
                  << "\n y: " << vector.loc.y()
                  << "\n theta: " << vector.angle
                  << "\n weight: " << vector.weight
                  << std::endl;
        entry++;
    }

    // Reduced Vector of Particles
    std::vector <Particle> particle_vec_reduced;
    // CALL MAIN RESAMPLE FUNCTION
    particle_vec_reduced = resample(particle_vec);

    std::cout << "\n======== Resampled Vector =========" << std::endl;

    int vector_entry {0};
    for (auto reduced_vec: particle_vec_reduced)
    {
        
        std::cout << "[Particle: " << vector_entry<< "]"
                  << "\n x: " << reduced_vec.loc.x()
                  << "\n y: " << reduced_vec.loc.y()
                  << "\n theta: " << reduced_vec.angle
                  << "\n weight: " << reduced_vec.weight
                  << std::endl;
        vector_entry++;
    }

    // Get Optimal Particle
    std::cout << "\n-------- Optimal Particle --------" << std::endl;

    Particle opt_part = get_optimal_particle(particle_vec_reduced);

    std::cout << "[Optimal Particle]"
                  << "\n x: " << opt_part.loc.x()
                  << "\n y: " << opt_part.loc.y()
                  << "\n theta: " << opt_part.angle
                  << "\n weight: " << opt_part.weight
                  << std::endl;

    return 0;
}