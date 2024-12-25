#include"leadcar.hpp"

// constructor implementation
// This is where we will set the Input/Lead car speed profile - As of now, hardcoding the USER Input
acc::cl_leadCar::cl_leadCar()
{
    // define the speed profile
    // std::fill(leadCar_speed_profile.begin(), leadCar_speed_profile.end(), 20.00);
    
    // Add user-selected points to the vector
    acc::cl_leadCar::user_inputs.emplace_back(0, 10.0); // Using emplace_back
    acc::cl_leadCar::user_inputs.emplace_back(600, 10.0);
    acc::cl_leadCar::user_inputs.emplace_back(2000, 5.0);
    acc::cl_leadCar::user_inputs.emplace_back(3000, 5.0);
    acc::cl_leadCar::user_inputs.emplace_back(5000, 12.0);
    acc::cl_leadCar::user_inputs.emplace_back(6000, 12.0);
    acc::cl_leadCar::user_inputs.emplace_back(8000, 0.0);
    acc::cl_leadCar::user_inputs.emplace_back(9000, 0.0);
    acc::cl_leadCar::user_inputs.emplace_back(10000, 10.0);
    acc::cl_leadCar::user_inputs.emplace_back(11000, 15.0);
    acc::cl_leadCar::user_inputs.emplace_back(11999, 10.0);

    // Logic to create lenthy leadCar speed values for each timeStep and store it in Vector
    // The exact leadCar speed at any timeStep would be fetched in real-time from this vector
    counter = 1;
    p1 = user_inputs[0];
    p2 = user_inputs[1];
    if ((p2.x-p1.x) == 0)
    {
        m = 99999999;
    }else
    {
        m = (p2.y-p1.y)/(p2.x-p1.x);
    }
    c = p1.y - m*p1.x;

    for (size_t i = 0; i < 12000; i++)
    {
        if(i >= p2.x)
        {
            // p1, p2, m, c can be updated once unless 'i' value has exceeded 'p2.x'
            p1 = p2;
            counter++;
            p2 = user_inputs[counter];
            if ((p2.x-p1.x) == 0)
            {
                m = 99999999;
            }else
            {
                m = (p2.y-p1.y)/(p2.x-p1.x);
            }
            c = p2.y - m*p2.x;
        }
        // need to do it for each loop
        leadCar_speed_profile[i] = m*(i) + c;
    }
    

}

// function implementation
// Send the lead car speed at the current timestep
double acc::cl_leadCar::get_curr_leadCarSpeed(const int &current_timestamp)
{
    return leadCar_speed_profile[current_timestamp];
}