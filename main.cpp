//*****************************************************************************************************
//******************** ADAPTIVE CRUISE CONTROL using MODEL PREDICTIVE CONTROL************************** 
// ASSUMPTION / PROPERTIES
// 1. Ego is a point mass
// 2. Using NLOPT for MPC
// 3. Used 3 laws of motion assuming the vehicle acc. would remain constant in between timestep
// 4. Most of the time 'controller_timestep' means the time gap in between 2 MPC step
// 5. Algorithm = LD_MMA
// 6. Control horizon = TBA (5 controller timesteps)
// 7. Random noise has been added to the LIDAR readings to test MPC
//*****************************************************************************************************

//************************************ Tweak parameters ***********************************************
// NOTE - 'Control horizon steps', 'controller timestep (sec)' & control horizon (sec) MUST BE IN SYNQ    
// Hardcode control horizon steps   - mpc.cpp -- 'fn_findIneqtyForAllSimEgoSteps' parameters defination
//                                  - mpc.hpp -- 'fn_findIneqtyForAllSimEgoSteps' parameters declartion
//                                  - leadcar.hpp 
// Change controller timestep (sec) - leadcar.hpp
// Hardcode control horizon (sec)   - leadcar.hpp
// Total no of simulation step      - leadcar.hpp
//                                  - mpc.cpp
// Fill leadcar speed profile       - leaccar.cpp
// Initial dist betw cars           - ego.cpp
// Change min dist to lead car      - mpc.hpp 
// Change mass of the car           - ego.hpp 
// Change speed limit               - ego.hpp
// Change optimization algorithm    - mpc.cpp
// TWeak optimization algorithm     - mpc.cpp
// Change acc & braking limit       - mpc.hpp
// *****************************************************************************************************   


#include<iostream>
#include"mpc.cpp"                        

int main()
{
    // create instance of the ACC-MPC object
    acc::cl_MPC cl_my_acc;
    // start ACC simulation
    cl_my_acc.fn_evaluteMPC();
    // end of simulation
    std::cout<< "End of Simulation !!" << std::endl;    
}


// Copyright (C) 2004, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

/*
#include "/usr/local/include/coin-or/IpIpoptApplication.hpp"
#include "/usr/local/include/coin-or/IpSolveStatistics.hpp"
#include "MyNLP.hpp"
#include "MyNLP.cpp"

#include <iostream>

using namespace Ipopt;

int main(
   int,
   char**
)
{
   // Create an instance of your nlp...
   SmartPtr<TNLP> mynlp = new MyNLP(5);

   // Create an instance of the IpoptApplication
   //
   // We are using the factory, since this allows us to compile this
   // example with an Ipopt Windows DLL
   SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
   
   //app->Options()->SetNumericValue("tol", 3.82e-4);
   //app->Options()->SetStringValue("mu_strategy", "adaptive");
   //app->Options()->SetIntegerValue("print_level", 6);
   //app->Options()->SetStringValue("output_file", "results.out");

   // Initialize the IpoptApplication and process the options
   ApplicationReturnStatus status;
   status = app->Initialize();
   if( status != Solve_Succeeded )
   {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return (int) status;
   }

   status = app->OptimizeTNLP(mynlp);
   //status = app->ReOptimizeTNLP(mynlp);
   
   //status = app->OptimizeTNLP(mynlp);

   if( status == Solve_Succeeded )
   {
      // Retrieve some statistics about the solve
      Index iter_count = app->Statistics()->IterationCount();
      std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

      Number final_obj = app->Statistics()->FinalObjective();
      std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.'
                << std::endl;
   }

   status = app->OptimizeTNLP(mynlp);
   //status = app->ReOptimizeTNLP(mynlp);
   
   //status = app->OptimizeTNLP(mynlp);

   if( status == Solve_Succeeded )
   {
      // Retrieve some statistics about the solve
      Index iter_count = app->Statistics()->IterationCount();
      std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

      Number final_obj = app->Statistics()->FinalObjective();
      std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.'
                << std::endl;
   }

   return (int) status;
}
*/