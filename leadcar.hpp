#ifndef LEADCAR_H
#define LEADCAR_H
#include<vector>

namespace acc{

    // structure
    typedef struct {    
        size_t step_no;
    }my_sim_step;                                                                                       // will be used to send data into NLOPT function
    
    // const variables
    static const double controller_timestep (0.05);                                                     // time gap between the current and next MPC trigger (in secs) 
    const double control_horizon_in_secs (0.3);                                                        // control horizon (in secs)
    static const int control_horizon(6);                                                               // optimization variables = no of MPC triggers to control horizon

    // class
    // Lead car  
    // In future - It Will handle LEAD CAR SPEED PROFILE as Input
    class cl_leadCar
    {
        private:
            // variable
            std::vector<double> leadCar_speed_profile {std::vector<double>(12000)};                     // speed profile of the lead vehicle = USER INPUT FOR 10 mins  
        public:
            // constructor declaration              
            cl_leadCar();
        protected:
            // function declaration    
            double get_curr_leadCarSpeed(const int &current_timestep);                                  // Get the current user input speed of the lead car                
    };
}
#endif // !LEADCAR_H
