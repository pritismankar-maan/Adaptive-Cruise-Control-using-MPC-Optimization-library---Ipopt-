

#include<iostream>
#include <iomanip>
#include <iostream>
#include <vector>   
#include "/usr/local/include/nlopt.hpp"

/*
INPUTS - 
1. Speed profile of the front car

OUTPUT - 
1. Pose of the ego vehicle

ASSUMPTION -
1. All objects are point mass 
*/


// adaptive cruise control
namespace acc
{
    typedef struct 
    {
        size_t step_no;
    }my_sim_step;
    
    // hardware variable
    static const double controller_timestep (0.05);                                                         // time ste50 ms
    const double control_horizon_in_secs (0.25);                                                        // 8 secs
    static const int control_horizon =  int(std::floor(control_horizon_in_secs/controller_timestep));          // control horizon in number ... 
                                                                                                        // .. of timestemp
    
    // Lead car
    class cl_leadCar
    {
        private:
            // variable
            std::vector<double> leadCar_speed_profile {std::vector<double>(12000)};       // speed profile of the lead vehicle = USER INPUT FOR 10 mins  
        public:
            // constructor declaration              
            cl_leadCar();
            // function declaration
            //void fn_getDistanceBetweenCars(const int &current_timestep, const double &ego_curr_velocity, double &distance_betw_cars);
            //void fn_estimateLeadCarSpeed();
        protected:    
            double get_curr_leadCarSpeed(const int &current_timestep);                
    };
    // constructor implementation
    // This is where we will set the Input/Lead car speed profile
    cl_leadCar::cl_leadCar()
    {
        // define the speed profile
        std::fill(leadCar_speed_profile.begin(), leadCar_speed_profile.end(), 20.00);
    }
    // function implementation
    double cl_leadCar::get_curr_leadCarSpeed(const int &current_timestamp)
    {
        return leadCar_speed_profile[current_timestamp];
    }
    
    
    // Ego
    class cl_ego:public cl_leadCar
    {
        private:
            /* data */
        public:
            // variables
            constexpr static double ego_mass {1800};                                                                // Ego's mass (static cuz nlopt func can accept it) 
            constexpr static double speed_limit {29.058};                                                           // speed limit = 65 miles per hour
            static double dist_betw_cars;                                                                       // Distance between cars at any point
            double prev_dist_betw_cars;
            static double ego_curr_velocity;                                                                    // Ego velocity at any point of time
            double prev_ego_velocity;
            static double leadCar_curr_velocity;                                                                // Lead car velocity at any point
            double dist_covered_ego {0.00};

            // constructor declaration+
            cl_ego();
            // functions declaration
            void fn_EgoStepVelocity(const double &acc_applied);
            void fn_getDistanceBetweenCars(const int &current_timestep);
            void fn_estimateLeadCarSpeed();
    };

    // need define static member variables !!!
    double cl_ego::leadCar_curr_velocity;
    double cl_ego::dist_betw_cars;
    double cl_ego::ego_curr_velocity;

    // constructor implementation
    cl_ego::cl_ego(/* args */)
    {
        // Initial distance between cars and Ego's initial velocity
        dist_betw_cars = 500.00;
        ego_curr_velocity = 0.00;
    }
    // function implementation
    // get the new velocity/speed after the first element of optimized force is applied on Ego/Plant
    void cl_ego::fn_EgoStepVelocity(const double &accl_applied)
    {
        // since here we don't have the speedometer, we will introduce random measurement noise here 
        // need to add random function here 
        prev_ego_velocity = ego_curr_velocity;
        // v = u + a*t + Random Noise (measurement noise)
        ego_curr_velocity = ego_curr_velocity + (controller_timestep*accl_applied); //- 2;
        // v^2 - u^2 = 2*a*s
        dist_covered_ego = ((pow(ego_curr_velocity,2) - pow(prev_ego_velocity,2)))/(2*accl_applied); 
    }
    // get distance between 2 cars, Usually will be taken care by lidar on a real sceanrio
    void cl_ego::fn_getDistanceBetweenCars(const int &current_timestep)
    {
        prev_dist_betw_cars = dist_betw_cars;   
        // calculate relative speed inbetween cars without any observation errors
        double dist_by_leadCar =  acc::cl_leadCar::get_curr_leadCarSpeed(current_timestep)*controller_timestep;
        
        dist_betw_cars = dist_betw_cars + dist_by_leadCar - dist_covered_ego;

        // handle negative scenario
        if (dist_betw_cars < 0)
        {
            std::cout<< "car has crashed - Distance between cars negative!!!" << std::endl;
        }
    }
    // get lead car speed based on distance between 2 cars
    void cl_ego::fn_estimateLeadCarSpeed()
    {
        
        // calculate estimated speed of the lead car
        leadCar_curr_velocity = ((dist_covered_ego + dist_betw_cars - prev_dist_betw_cars)/controller_timestep);

        // handle negative scenario
        if (leadCar_curr_velocity < 0)
        {
            std::cout<< "car has crashed !!! - Lead car has negative velocity" << std::endl;
        }

    }    
    
    
    class cl_MPC: public cl_ego
    {
    private:
        static size_t sim_step_forward_counter;   // static so that this can be used/called inside NLOPT function
        //my_sim_step sim_step;
        static std::vector<double> accl_sum;
        static std::vector<double> accl_sum_sum;
        static std::vector<double> sim_ego_dist_vector;
        constexpr static double min_dist_betwn_cars {12.00};  // 12 meters
        static int count;

        double const acc_high_limit {2};          // 2ms-2 accelration limit
        double const brk_high_limit {4};          // 0.4G braking force


        std::vector<double> opt_A {std::vector<double>(control_horizon)};
        double min_cost_value;

        std::vector<double> lb {std::vector<double>(control_horizon)};
        std::vector<double> ub {std::vector<double>(control_horizon)};

        // declare object
        cl_ego cl_real_ego;
        nlopt::opt opt;
        /* data */
    public:
        cl_MPC();
        ~cl_MPC();
        void fn_setUpperLowerBound();
        void fn_initializeMPC();
        void fn_findIneqtyForAllSimEgoSteps(my_sim_step sim_step_ptrs[5]);
        void fn_getInitialEstimate();
        void fn_sendControlInputToPlant(const size_t step_forward_counter);
        void fn_evaluteMPC();

        static double fn_evaluteCostFunction(const std::vector<double> &A, std::vector<double> &grad, void *my_func_data);
        static double fn_evaluteSpeedIneqty(const std::vector<double> &A, std::vector<double> &grad, void *data);
        static double fn_evaluteDistIneqty(const std::vector<double> &A, std::vector<double> &grad, void *data); 

    };

    // need to define the non-const static variables
    size_t cl_MPC::sim_step_forward_counter;
    std::vector<double> cl_MPC::accl_sum {};
    std::vector<double> cl_MPC::accl_sum_sum {};
    std::vector<double> cl_MPC::sim_ego_dist_vector {};
    int cl_MPC::count {};

    cl_MPC::cl_MPC()
    {
        int control_horizon =  int(std::floor(control_horizon_in_secs/controller_timestep));
        
        //setenv("OMP")
        // initialize the optimizer
        opt = nlopt::opt(nlopt::LD_MMA, control_horizon);

        //opt.set_num_threads
        // declare initial F values before optimization 
        std::vector<double> opt_F(control_horizon,0);
    }
    
    cl_MPC::~cl_MPC()
    {
    }

    // set upper and lower bounds for optimization
    void cl_MPC::fn_setUpperLowerBound()
    {
        // declare and initialize lower bound and upper bound
        std::fill(lb.begin(), lb.end(), (-brk_high_limit));
        std::fill(ub.begin(), ub.end(), (acc_high_limit));
        
        // send the upper bound and upper bound to optimization object
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        opt.set_maxeval(32);
        //opt.set_maxtime(60);
        
    }
    
    // cost function implementation
    double cl_MPC::fn_evaluteCostFunction(const std::vector<double> &A, std::vector<double> &grad, void *my_func_data)
    {
        double sum {0.0};
        double sim_speed {0.0};

        ++count;
        accl_sum.clear();
        accl_sum_sum.clear();
        sim_ego_dist_vector.clear();

        // single summation of Accl vector
        for (size_t i = 0; i < A.size(); i++)
        {
            sum = sum + A[i];
            accl_sum.push_back(sum);

            // Below vector would be used for Distance Inequality
            if(i ==0)
            {
                sim_speed = ego_curr_velocity;
                sim_ego_dist_vector.push_back((sim_speed*controller_timestep) + (0.5*A[i]*pow(controller_timestep,2)));
            }else
            // for all other simulated timestep
            {
                sim_ego_dist_vector.push_back(sim_ego_dist_vector[i-1] + (sim_speed*controller_timestep) + (0.5*A[i]*pow(controller_timestep,2)));            
            }
            sim_speed = sim_speed + A[i]*controller_timestep;
        }

        // double summation of Accl vector but from end to beginning    
        sum = 0.0;
        for(auto riter = accl_sum.rbegin(); riter != accl_sum.rend(); ++riter)
        {
            sum = sum + (*riter);
            accl_sum_sum.push_back(sum);
        }

        // compute grad/partial differentiation of cost function wrt to each element in F
        if (!grad.empty()) 
        {
            for (size_t i = 0; i < control_horizon; i++)
            {
                grad[i] = (2*A[i]) 
                            + (2*pow(controller_timestep,2)*(accl_sum_sum[control_horizon-i-1])) 
                            - (2*(control_horizon-i)*controller_timestep*(speed_limit-ego_curr_velocity));
            }
        }

        // compute cost function value
        double costfunc_value {0.0};
        for(size_t i = 0; i<A.size(); i++)
        {
            costfunc_value = costfunc_value + pow(A[i],2) + pow((speed_limit - ego_curr_velocity - (controller_timestep*accl_sum[i])),2);    
        }    
        
        return costfunc_value;
    }
    
    // Speed Inequality Implementation
    double cl_MPC::fn_evaluteSpeedIneqty(const std::vector<double> &A, std::vector<double> &grad, void *data)
    {
        my_sim_step *sim_step = reinterpret_cast<my_sim_step*>(data);
        size_t eqn_no = sim_step->step_no;
        
        // evalute grad vector and reqd summation of force needed to form the speed inequality constraint value 
        if (!grad.empty())
        {
            std::fill(grad.begin(), grad.end(),0);
            std::fill_n(grad.begin(), eqn_no, controller_timestep); 
        }    
        // evalute f(x) where inequality equation is in the form f(x) < 0
        return ego_curr_velocity - speed_limit + (controller_timestep*accl_sum[eqn_no-1]);
    }
    
    // Distance Inequality Implementation (ASSUMPTION - The lead car velocity remains constant through out the simulated steps)
    double cl_MPC::fn_evaluteDistIneqty(const std::vector<double> &A, std::vector<double> &grad, void *data)
    {
        my_sim_step *sim_step = reinterpret_cast<my_sim_step*>(data);
        size_t eqn_no = sim_step->step_no;

        // can be made better by storing dist_vector in 1 go instead of storing them again and again
        // below loop is highly inefficient =  Need to unify all the summation force and find dist_vector ...  
        // ... if F stays the same for both speed & Inequal}ity func & costfunction
        // Dist(k) =  Dist(k-1) + ego_speed(k-1)*timestep + 0.5*acc*(timestep)^2
        for (size_t i = 0; i < control_horizon; i++)
        {
            if (!grad.empty()) 
            // grad[i] = (timestep^2)*(0.5 + sim_counter - (i+1)) if sim_counter >= i+1
            //         = 0                                        else
            {
                if (eqn_no >= (i+1)) // need to check this
                {
                    grad[i] = pow(controller_timestep,2)*(0.5+eqn_no-i-1);
                }else
                {
                    grad[i] = 0;
                }
            }     
        }

        //count++;

        // inequality eqn for distance where the inequality is in the form of f(k) < 0 where ...
        // f(k) = 'min distance to be maintained' - (ditance travelled by lead car + lidar reading before sim steps - distance travelled by ego ) 
        return (double(min_dist_betwn_cars + sim_ego_dist_vector[(int(eqn_no)-2)] - dist_betw_cars - (leadCar_curr_velocity*(controller_timestep*(eqn_no-1)))));
    }

    // Find & optimize all the Inequality Constraint using the Plant's Sim all over the Control Horizon time period 
    void cl_MPC::fn_findIneqtyForAllSimEgoSteps(my_sim_step sim_step_ptrs[5]) // change control horizon steps
    {
        //count = 1;
        // 'sim_step_forward_counter' is wrt to Simulated Ego/Plant
        for(sim_step_forward_counter = 1; sim_step_forward_counter <= control_horizon; sim_step_forward_counter++)
        {
            (sim_step_ptrs[sim_step_forward_counter - 1]).step_no = sim_step_forward_counter;
            // add speed constraint for each step taken by simulated Ego
            opt.add_inequality_constraint(fn_evaluteSpeedIneqty, &sim_step_ptrs[sim_step_forward_counter - 1], 1e-4);
            
            // add distance constraint for each step taken by simulated Ego
            opt.add_inequality_constraint(fn_evaluteDistIneqty, &sim_step_ptrs[sim_step_forward_counter - 1], 1e-4);    
        }
    }

    void cl_MPC::fn_getInitialEstimate()
    {
        // For Simplicity, assigning 0 everytime
        //std::fill(opt_A.begin(), opt_A.end(),0);
    }


    // initialize MPC - Set upper and lower bound & set cost function
    void cl_MPC::fn_initializeMPC()
    {
        //cl_ego cl_sim_ego;
        
        // set upper and lower bounds for MPC
        fn_setUpperLowerBound();
        
        // declare the cost function and its gardient
        opt.set_min_objective(fn_evaluteCostFunction, NULL);

        // set tolerance level before the opt is considered success
        opt.set_xtol_rel(1e-2);    
    }

    // find and send the optimied force to the real plant/Ego  
    void cl_MPC::fn_sendControlInputToPlant(const size_t step_forward_counter)
    {
        // Send force that will update the velocity
        fn_EgoStepVelocity(opt_A[0]);

        //if(step_forward_counter < (control_horizon-1)) // unsure why I added this in first place
        //{
            // Get the latest radar reading after Ego has moved a timestep
            fn_getDistanceBetweenCars(step_forward_counter+1);

            // estimate the lead car velocity for the next iteration
            fn_estimateLeadCarSpeed();
        //}
    }

    // final function 
    void cl_MPC::fn_evaluteMPC()
    {
        // Initialize cost function. UP&LW bounds 
        fn_initializeMPC();
        
        // 'step_forward_counter' is wrt to real Ego/Plant, run the plant for 10mins
        for(size_t step_forward_counter = 0; step_forward_counter < 12000; step_forward_counter++)
        {
            // get feedback (kalman filters can be used in real measurement to get the pose = Distance between cars & speed)
            // For simplicity, we won't be implementing the kalman filter

            // get current speed
            my_sim_step* sim_step_ptrs = new my_sim_step[control_horizon];

            for (size_t i = 0; i < control_horizon; i++)
            {
                *(sim_step_ptrs+i) = {i+1}; 
            }
            

            // get the current/updated distance between cars as well as the lead car velocity (updated internally/explicitly?? needed for dist inequality) 

            // add inequality constraints for the current timestep
            // cost function might be changed due to change in the ego current velocity, but that would be taken care automatically since we would be using the same class
            // lower and upper bound would remain same through out, so we don't need to update them for each completion of feedback loop  
            fn_findIneqtyForAllSimEgoSteps(sim_step_ptrs); 
            
            // assign initial estimate for opt_F for each timestep
            fn_getInitialEstimate();

            count = 0;

            // Evalute NLOPT for finding the optimized force
            try{
                nlopt::result result = opt.optimize(opt_A, min_cost_value);
                std::cout << "found minimum at accl = " << opt_A[0]
                    // << std::setprecision(10) << min_cost_value 
                    << std::endl;
                std::cout<<"One step completed"<<std::endl;
                std::cout<<"New Ego speed = "<<ego_curr_velocity<<std::endl;
                std::cout<< "Old lidar reading = "<<prev_dist_betw_cars<<std::endl;
                std::cout<< "New lidar reading = "<<prev_dist_betw_cars<<std::endl;
                std::cout<< "Max Iteration = "<<count<<std::endl;

            }
            catch(std::exception &e) {
                std::cout << "nlopt failed: " << e.what() << std::endl;
            }        

            // finally send the optimized force to plant and record the new radar output/estimate the latest leadCar velocity 
            fn_sendControlInputToPlant(step_forward_counter);

            delete[] sim_step_ptrs;

        }
    }
}



/*
int main()
{
    // create instance of the ACC-MPC object
    acc::cl_MPC cl_my_acc;
    // start ACC simulation
    cl_my_acc.fn_evaluteMPC();

    std::cout<< "End of Simulation !!" << std::endl;    
}
*/


