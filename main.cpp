//*****************************************************************************************************
//******************** ADAPTIVE CRUISE CONTROL using MODEL PREDICTIVE CONTROL************************** 
// ASSUMPTION / PROPERTIES
// 1. Ego is a point mass
// 2. Using IPOPT for MPC's optimization
// 3. Used 3 laws of motion assuming the vehicle acc. would remain constant in between timestep
// 4. Most of the time 'controller_timestep' means the time gap in between 2 MPC step
// 5. Algorithm = LD_MMA
// 6. Control horizon = 3.5 secs (70 controller timesteps)
// 7. Random noise has been added to the LIDAR readings to test MPC
//*****************************************************************************************************

//************************************ Tweak parameters ***********************************************
// NOTE - 'Control horizon steps', 'controller timestep (sec)' & control horizon (sec) MUST BE IN SYNQ    
// Hardcode control horizon steps   - mpc.cpp -- 'fn_findIneqtyForAllSimEgoSteps' parameters defination
//                                  - mpc.hpp -- 'fn_findIneqtyForAllSimEgoSteps' parameters declartion
//                                  - leadcar.hpp 
// Change controller timestep (sec) - leadcar.hpp ~ 50 ms
// Hardcode control horizon (sec)   - leadcar.hpp ~ 3500 ms
// Total no of simulation step      - leadcar.hpp ~ 70 horizon steps
//                                  - mpc.cpp
// Fill leadcar speed profile       - leadccar.cpp
// Initial dist betw cars           - ego.cpp     ~ 500 meters
// Initial Ego Speed                - ego.cpp     ~ 0 mps
// Change min dist to lead car      - mpc.hpp     ~ 12 meters
// Change mass of the car           - ego.hpp     ~ 1800 kg
// Change speed limit               - ego.hpp     ~ 65 mph / 29.058 mps
// Change optimization algorithm    - mpc.cpp
// TWeak optimization alg option    - mpc.cpp
// Change acc & braking limit       - mpc.hpp     ~ [-4, 2] mpss
// Change acc weighs on Cost func   - MyNLP.hpp   ~ 0.001
// Change speed weighs on Cost func - MyNLP.hpp   ~ 10
// Change dist weighs on Cost func  - MyNLP.hpp   ~ 0.001  
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