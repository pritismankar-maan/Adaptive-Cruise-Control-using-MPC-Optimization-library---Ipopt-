#include<iostream>
#include "/usr/local/include/nlopt.hpp"
#include"ego.hpp"
#include"ego.cpp"
#include"mpc.hpp"
#include "MyNLP.hpp"
#include "/usr/local/include/coin-or/IpIpoptApplication.hpp"
#include "/usr/local/include/coin-or/IpSolveStatistics.hpp"

#include "MyNLP.cpp"

// need to define the non-const static variables
size_t acc::cl_MPC::sim_step_forward_counter;
// constructor 
acc::cl_MPC::cl_MPC()
{
    // initialize the optimizer, All LD algorithm can be used - might need some parameters tweaking
    //acc::cl_MPC::opt = nlopt::opt(nlopt::LD_SLSQP, control_horizon);    // can use LD_MMA or LD_SLSQP interchangably
}


// find and send the optimied force to the real plant/Ego  
void acc::cl_MPC::fn_sendControlOutputToPlant(const size_t step_forward_counter, const double opt_acc_value)
{
    // Send force that will update the velocity
    acc::cl_ego::fn_EgoStepVelocity(opt_acc_value);

    // Get the latest radar reading after Ego has moved a timestep
    acc::cl_ego::fn_getDistanceBetweenCars(step_forward_counter+1);

    // estimate the lead car velocity for the next iteration
    acc::cl_ego::fn_estimateLeadCarSpeed();
}

// final function 
void acc::cl_MPC::fn_evaluteMPC()
{
    // Initialize cost function. UP&LW bounds 
    //acc::cl_MPC::fn_initializeMPC();
    // Create an instance of your nlp...
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new MyNLP(70);
    
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    //app->Options()->SetNumericValue("tol", 3.82e-4);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("check_derivatives_for_naninf","yes");
    app->Options()->SetStringValue("check_bounds","yes");
    app->Options()->SetIntegerValue("print_level", 2);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetNumericValue("nlp_scaling_max_gradient", 100);
    app->Options()->SetNumericValue("obj_scaling_factor", 0.01);
    app->Options()->SetNumericValue("bound_relax_factor", 1e-6);

    //app->Options()->SetStringValue("output_file", "results.out");

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Ipopt::Solve_Succeeded )
    {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        //return (int) status;
    }

        
    // 'step_forward_counter' is wrt to real Ego/Plant, run the plant for 10mins
    for(size_t step_forward_counter = 0; step_forward_counter < 12000; step_forward_counter++)
    {
        // get feedback (kalman filters can be used in real measurement to get the pose = Distance between cars & speed)
        // For simplicity, we won't be implementing the kalman filter
        

        status = app->OptimizeTNLP(mynlp);

        if( status == Ipopt::Solve_Succeeded )
        {
            // Retrieve some statistics about the solve
            Ipopt::Index iter_count = app->Statistics()->IterationCount();
            std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

            Ipopt::Number final_obj = app->Statistics()->FinalObjective();
            std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.'
                        << std::endl;
        
        }
        else 
        //if (status == Ipopt::)
        
        {
            
            //acc::cl_MPC::fn_sendControlOutputToPlant(int(MyNLP::A),-4.00);            
            //std::cout<<"Timestep in the real world = "<<MyNLP::A<<std::endl;
            //std::cout<<"Measure Speed = "<<acc::cl_ego::ego_curr_velocity<<std::endl;
            //std::cout<< "Update lidar reading = "<<acc::cl_ego::dist_betw_cars<<std::endl;
            //MyNLP::A++;
        }

    }
}