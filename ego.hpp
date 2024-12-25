#ifndef EGO_H
#define EGO_H
#include"leadcar.hpp"
#include<random>

namespace acc{
// Class declaration
// Ego
// Handles - Calc of new velocity, processing new radar output,
//           Estimate lead car speed needed for optimization
//           Define car properties, initial lidar reading, speed limit 
class cl_ego:public acc::cl_leadCar
{
    public:
        // variables
        constexpr static double ego_mass {1800};                                                            // Ego's mass (static cuz nlopt func can accept it) 
        constexpr static double speed_limit {29.058};                                                       // speed limit = 65 miles per hour
        static double dist_betw_cars;                                                                       // Latest Lidar reading
        double prev_dist_betw_cars;                                                                         // Previous Lidar reading
        static double ego_curr_velocity;                                                                    // Ego's speed at the latest timestep 
        double prev_ego_velocity;                                                                           // Ego's speed at previous timestep
        static double leadCar_curr_velocity;                                                                // Estimated Lead car speed based on lidar readings 
        double dist_covered_ego {0.00};                                                                     // Distance covered by the Ego from zero-point
        const double lidar_stddev {0.1};                                                                    // lidar measurement noise - std dev value

        // Define variabled needed to introduce random noise in LIDAR data
        double min_noise {-0.1};    
        double max_noise {0.1};
        std::random_device rd;                                                                              // Random number generator
        double noise; // Store random noise value in specific timeSteps
        
        // constructor declaration+
        cl_ego();

        // functions declaration
        void fn_EgoStepVelocity(const double &acc_applied);                                                 // Get the latest Ego's speed
        void fn_getDistanceBetweenCars(const int &current_timestep);                                        // LIDAR should do this job but now calculated manually
        void fn_estimateLeadCarSpeed();                                                                     // Estimate lead car velocity based on the LIDAR reading 
};
}
#endif // !EGO_H