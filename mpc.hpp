#ifndef MPC_H
#define MPC_H
#include "/usr/local/include/nlopt.hpp"
#include"ego.hpp"

namespace acc{

// class declaration
// MPC functionality - algorithm
// logic for doing optimization, define opti parameters, MPC step
class cl_MPC: public acc::cl_ego
{
private:

    static size_t sim_step_forward_counter;                                                 // counter variable   
    
    std::vector<double> opt_A {std::vector<double>(control_horizon)};                       // vector that stores the optimized acceleration vector after finding min cost

    // declare object
    nlopt::opt opt;
    // cl_ego cl_real_ego; - not needed, check
    
public:
    // constructor
    cl_MPC();
    // member function
    void fn_sendControlOutputToPlant(const size_t step_forward_counter, const double opt_acc_value);                    // Send the optimized acceleration to actual Ego. i.e.Update velocity, 
                                                                                           // update lidar reading and estimate lead car speed for the next MPC step                                                                                       
    void fn_evaluteMPC();                                                                  // Start the process of real time optimization for set of time

    // Below function would be called from NLOPT library and hence the 'static keyword'
    

};
}
#endif // !MPC_H