//#include"acc_ns.hpp"
//#include"acc_ns.cpp"
#include"leadcar.hpp"

//int acc::control_horizon =  int(std::floor(acc::control_horizon_in_secs/acc::controller_timestep));

// constructor implementation
// This is where we will set the Input/Lead car speed profile - As of now, hardcoding the USER Input
acc::cl_leadCar::cl_leadCar()
{
    // define the speed profile
    std::fill(leadCar_speed_profile.begin(), leadCar_speed_profile.end(), 15.00);
    std::fill(std::begin(leadCar_speed_profile)+4000, std::end(leadCar_speed_profile), 29.00);
    //control_horizon =  int(std::floor(acc::control_horizon_in_secs/acc::controller_timestep));
}
// function implementation
// Send the lead car speed at the current timestep
double acc::cl_leadCar::get_curr_leadCarSpeed(const int &current_timestamp)
{
    return leadCar_speed_profile[current_timestamp];
}