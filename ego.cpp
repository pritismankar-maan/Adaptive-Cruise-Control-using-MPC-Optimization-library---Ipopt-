#include<iostream>
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
    
    // UPDATE PLANT STEP: v = u + a*t + Random Noise (measurement noise)
    acc::cl_ego::ego_curr_velocity = acc::cl_ego::ego_curr_velocity + (acc::controller_timestep*accl_applied); //- 2;
    
    // Below logic is standard car behavior: currently in Drive gear and hence braking won't make Ego move backwards 
    if (acc::cl_ego::ego_curr_velocity < 0)
    {
        acc::cl_ego::ego_curr_velocity = 0;
    }
    
    // UPDATE INTERNAL STATE VARIABLE: Find 's' using v^2 - u^2 = 2*a*s
    if (accl_applied!=0)    // handle 0 denominator
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
    
    // Get the simulated LIDAR output, used vector maths
    acc::cl_ego::dist_betw_cars = acc::cl_ego::dist_betw_cars + dist_by_leadCar - acc::cl_ego::dist_covered_ego;

    // Handle negative scenario and let user know ACC failed in preventing crash
    if (acc::cl_ego::dist_betw_cars < 0)
    {
        std::cout<< "car has crashed - Distance between cars negative!!!" << std::endl;
    }

    // Add random noise to LIDAR readings
    std::mt19937 gen(rd()); // Seeded generator
    std::uniform_real_distribution<double> dist(min_noise, max_noise);
    noise = dist(gen);
    acc::cl_ego::dist_betw_cars = acc::cl_ego::dist_betw_cars + noise; 
}

// get lead car speed based on distance between 2 cars
void acc::cl_ego::fn_estimateLeadCarSpeed()
{
    
    // Estimate the new speed of the lead car based on the LIDAR reading
    // The intentional noise builds over time and can induce little oscillation while maintaining safe following distance
    // In real-world, use a kalman filter to get a low noise radar reading
    acc::cl_ego::leadCar_curr_velocity = ((acc::cl_ego::dist_covered_ego + acc::cl_ego::dist_betw_cars - acc::cl_ego::prev_dist_betw_cars)/acc::controller_timestep);

    // handle negative scenario
    if (acc::cl_ego::leadCar_curr_velocity < 0)
    {
        acc::cl_ego::leadCar_curr_velocity = 0;
        std::cout<< "Reading the front car has stopped !!" << std::endl;
    }
}