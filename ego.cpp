#include<iostream>
#include<random>
#include"leadcar.hpp"
#include"leadcar.cpp"
#include"ego.hpp"

// need define static member variables !!!
double acc::cl_ego::leadCar_curr_velocity;
double acc::cl_ego::dist_betw_cars;
double acc::cl_ego::ego_curr_velocity;


// constructor implementation
acc::cl_ego::cl_ego(/* args */)
{
    // Initial distance between cars and Ego's initial velocity
    acc::cl_ego::dist_betw_cars = 500.00;
    acc::cl_ego::ego_curr_velocity = 0.00;
}
// function implementation
// get the new velocity/speed after the first element of optimized force is applied on Ego/Plant
void acc::cl_ego::fn_EgoStepVelocity(const double &accl_applied)
{
    // since here we don't have the speedometer, we will introduce random measurement noise here 
    // IN FUTURE, we need to add random function here 
    acc::cl_ego::prev_ego_velocity = acc::cl_ego::ego_curr_velocity;
    // v = u + a*t + Random Noise (measurement noise)
    acc::cl_ego::ego_curr_velocity = acc::cl_ego::ego_curr_velocity + (acc::controller_timestep*accl_applied); //- 2;
    // v^2 - u^2 = 2*a*s
    if (accl_applied!=0)
    {
        acc::cl_ego::dist_covered_ego = ((pow(acc::cl_ego::ego_curr_velocity,2) - pow(acc::cl_ego::prev_ego_velocity,2)))/(2*accl_applied);
    }else
    {
        acc::cl_ego::dist_covered_ego = acc::cl_ego::prev_ego_velocity*acc::controller_timestep;    
    }
}
// get distance between 2 cars. In future, LIDAR output would replace this whole function
void acc::cl_ego::fn_getDistanceBetweenCars(const int &current_timestep)
{
    acc::cl_ego::prev_dist_betw_cars = acc::cl_ego::dist_betw_cars;   
    // Distance travelled by lead car in 1 MPC Timestep =  USER INPUT lead car speed at new timestep x time gap (assumed the lead car maintained its speed in this timestep) 
    double dist_by_leadCar =  acc::cl_leadCar::get_curr_leadCarSpeed(current_timestep)*acc::controller_timestep;
    
    // Get the simulated LIDAR output
    acc::cl_ego::dist_betw_cars = acc::cl_ego::dist_betw_cars + dist_by_leadCar - acc::cl_ego::dist_covered_ego;

    // handle negative scenario
    if (acc::cl_ego::dist_betw_cars < 0)
    {
        std::cout<< "car has crashed - Distance between cars negative!!!" << std::endl;
    }

        // Define random generator with Gaussian distribution
    double mean = acc::cl_ego::dist_betw_cars;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, lidar_stddev);

    //std::cout<< "Dist prev = " << acc::cl_ego::dist_betw_cars << std::endl;
    acc::cl_ego::dist_betw_cars =+dist(generator); 
    //std::cout<< "after prev = " << acc::cl_ego::dist_betw_cars << std::endl;
}
// get lead car speed based on distance between 2 cars
void acc::cl_ego::fn_estimateLeadCarSpeed()
{
    
    // Estimate the new speed of the lead car based on the LIDAR reading
    acc::cl_ego::leadCar_curr_velocity = ((acc::cl_ego::dist_covered_ego + acc::cl_ego::dist_betw_cars - acc::cl_ego::prev_dist_betw_cars)/acc::controller_timestep);

    // handle negative scenario
    if (acc::cl_ego::leadCar_curr_velocity < 0)
    {
        std::cout<< "car has crashed !!! - Lead car has negative velocity" << std::endl;
    }
}