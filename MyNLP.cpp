#include "MyNLP.hpp"
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/* Constructor. */
MyNLP::MyNLP(const int &k):k_(k)
{ 
   
}

MyNLP::~MyNLP()
{ }

bool MyNLP::get_nlp_info(
   Ipopt::Index&          n,
   Ipopt::Index&          m,
   Ipopt::Index&          nnz_jac_g,
   Ipopt::Index&          nnz_h_lag,
   Ipopt::TNLP::IndexStyleEnum& Index_style
)
{
   // The problem described in MyNLP.hpp has 3k_ variables - v1,s1,a1,v2,s2,a2,...,
   n = 3*k_;

   // 2k_ equality constraint - Safe following dist & Speed inequality for each timeStep
   m = 2*k_;

   //nonzeros in the jacobian -
   nnz_jac_g = 7*k_ - 3;

   // nonzeros in the hessian of the lagrangian
   nnz_h_lag = 3*k_;

   // We use the standard fortran Ipopt::Index style for row/col entries
   Index_style = C_STYLE; //FORTRAN_STYLE;
   return true;
}

bool MyNLP::get_bounds_info(
   Ipopt::Index   n,
   Ipopt::Number* x_l,
   Ipopt::Number* x_u,
   Ipopt::Index   m,
   Ipopt::Number* g_l,
   Ipopt::Number* g_u
)
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.
   assert(n == 3*k_);
   assert(m == 2*k_);

   double prev_dist = 0;
   // upper and lower bound of parameters (v1,s1,a1,v2,s2,a2,........,vk,sk,ak) and constraint
   for (size_t i = 0; i < n; i++)
   {
      if(i<m)
      {
         // Inequalities: set lower and upper limit of constraints
         g_l[i] = 0;
         g_u[i] = 0;

         // State varibles(3k_): set lower and upper limit 
         if(i%3==0)
         {
            // speed limit
            x_l[i] = 0;
            x_u[i] = acc::cl_ego::speed_limit;
         }
         else if (i%3==1)
         {
            // distance limit
            x_l[i] = 0;
            //prev_dist = (acc::cl_ego::ego_curr_velocity*acc::controller_timestep*(i+2)/3)
            // distance upper limit = lidar reading + sim distance by lead car - 12 (distance limit)
            x_u[i] = acc::cl_ego::dist_betw_cars + (acc::cl_ego::leadCar_curr_velocity*acc::controller_timestep*(i+2)/3) - 12 - (acc::cl_ego::ego_curr_velocity*acc::controller_timestep*(i+2)/3);
            
            // Ego can't travel backwards and hence the negative future distanceStep are over-written to zero 
            if (x_u[i] < 0)
            {
               x_u[i] = 0;
            }
         }
         else
         {
            // acceleration limit
            x_l[i] = -4;
            x_u[i] = 2;
         }
         
      }
      else
      {
         // State varibles(3k_): set lower and upper limit
         if(i%3==0)
         {
            // speed limit
            x_l[i] = 0;
            x_u[i] = acc::cl_ego::speed_limit;
         }
         else if (i%3==1)
         {
            // distance limit
            x_l[i] = 0;
            // distance upper limit = lidar reading + sim distance by lead car - 12 (distance limit)
            x_u[i] = acc::cl_ego::dist_betw_cars + (acc::cl_ego::leadCar_curr_velocity*acc::controller_timestep*(i+2)/3) - 12 - (acc::cl_ego::ego_curr_velocity*acc::controller_timestep*(i+2)/3);
            
            // Ego can't travel backwards and hence the negative future distanceStep are over-written to zero 
            if (x_u[i] < 0)
            {
               x_u[i] = 0;
            }               
         }
         else
         {
            // acceleration limit
            x_l[i] = -4;
            x_u[i] = 2;
         }
      }
      
      // For debugging
      //if (acc::cl_ego::ego_curr_velocity < 0.7)
      //{
      //   std::cout<<"i th value for constraint display = "<<i<<std::endl;
      //   std::cout<<"x_l[i] = "<<x_l[i]<<std::endl;
      //   std::cout<<"x_u[i] = "<<x_u[i]<<std::endl;

      //   if(x_u[i] < 0)
      //   {
      //      x_u[i] = 0.1;
      //   }
      //}
   
   }

   return true;
}

bool MyNLP::get_starting_point(
   Ipopt::Index   n,
   bool    init_x,
   Ipopt::Number* x,
   bool    init_z,
   Ipopt::Number* z_L,
   Ipopt::Number* z_U,
   Ipopt::Index   m,
   bool    init_lambda,
   Ipopt::Number* lambda
)
{
   // Here, we assume we only have starting values for x, if you code
   // your own NLP, you can provide starting values for the others if
   // you wish.
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   // Not modifying the values for now - Ideally, one 'x' should be right shifted !
   //intialize to zero
   //for (Ipopt::Index i = 0; i < n; i++)
   //{
   //   x[i] = 0;
   //}
   
   return true;
}

bool MyNLP::eval_f(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number&       obj_value
)
{

   assert(n == 3*k_);
   // return the value of the objective function
   obj_value = 0;
   
   for (size_t i = 0; i < n; i++)
   {
      if(i%3==0)
      {
         obj_value = obj_value + sped_weight*pow((acc::cl_ego::speed_limit-x[i]),2);
      }
      else if(i%3==1)
      {
         obj_value = obj_value - dist_weight*pow((x[i] - acc::cl_ego::dist_betw_cars - 12),2);
      }
      else if(i%3==2)
      {
         obj_value = obj_value + accl_weight*pow(x[i],2);
      }
   }
   
   return true;
}

bool MyNLP::eval_grad_f(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number*       grad_f
)
{
   // return the gradient of the objective function grad_{x} f(x)
   assert(n == 3*k_);
   
   for (size_t i = 0; i < n; i++)
   {
      if(i%3==0)
      {
         // partial derivatives wrt 'v(n)'
         grad_f[i] = -sped_weight*(acc::cl_ego::speed_limit-x[i]);
      }
      else if(i%3==2)
      {
         // partial derivatives wrt 'a(n)'
         grad_f[i] = accl_weight*2*x[i];
      }
      else
      {
         // partial derivatives wrt 's(n)'
         grad_f[i] = -2*dist_weight*(x[i] - acc::cl_ego::dist_betw_cars - 12);
      }
   }

   return true;
}

bool MyNLP::eval_g(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Index         m,
   Ipopt::Number*       g
)
{
   assert(n == 3*k_);
   assert(m == 2*k_);

   for (size_t i = 0; i < m; i++)
   {
      if(i%2==0)
      {
         // speed equality function
         if(i==0)
         {
            g[i] = x[0] - acc::cl_ego::ego_curr_velocity - x[2]*acc::controller_timestep;         
         }else
         {
            g[i] = x[Ipopt::Index(1.5*i)] - x[Ipopt::Index(1.5*i-3)] - (x[Ipopt::Index(1.5*i+2)]*acc::controller_timestep);
         }
      }
      else
      {
         // distance equality function
         if(i==1)
         {
            g[i] = x[Ipopt::Index(1.5*i - 0.5)] - x[Ipopt::Index(1.5*i - 4.5)]*acc::controller_timestep - (0.5*x[Ipopt::Index(1.5*i + 0.5)]*pow(acc::controller_timestep,2));         
         }else
         {
            g[i] = x[Ipopt::Index(1.5*i - 0.5)] - x[Ipopt::Index(1.5*i - 3.5)] - x[Ipopt::Index(1.5*i - 4.5)]*acc::controller_timestep - (0.5*x[Ipopt::Index(1.5*i + 0.5)]*pow(acc::controller_timestep,2));
         }
      }
   }

   return true;
}

bool MyNLP::eval_jac_g(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Index         m,
   Ipopt::Index         nele_jac,
   Ipopt::Index*        iRow,
   Ipopt::Index*        jCol,
   Ipopt::Number*       values
)
{
   
   assert(n == 3*k_);
   assert(m == 2*k_);
   
   if( values == NULL )
   {
      Ipopt::Index index;
      // return the structure of the jacobian of the constraints

      iRow[0] = 0;
      jCol[0] = 0;

      iRow[1] = 0;
      jCol[1] = 2;

      iRow[2] = 1;
      jCol[2] = 1;

      iRow[3] = 1;
      jCol[3] = 2;

      index = 4;
      for (size_t i = 2; i < m; i++)
      {
         if (i%2==0)
         {
            
            iRow[index] = i;
            jCol[index] = 1.5*i-3;
            index++;

            iRow[index] = i;
            jCol[index] = 1.5*i;
            index++;

            iRow[index] = i;
            jCol[index] = 1.5*i+2;
            index++;

         }else
         {
            iRow[index] = i;
            jCol[index] = 1.5*i-4.5;
            index++;

            iRow[index] = i;
            jCol[index] = 1.5*i-3.5;
            index++;

            iRow[index] = i;
            jCol[index] = 1.5*i-0.5;
            index++;

            iRow[index] = i;
            jCol[index] = 1.5*i+0.5;
            index++;
         }  
      }      
   }
   else
   {
      // return the values of the jacobian of the constraints
      values[0] = 1;
      values[1] = -acc::controller_timestep;
      values[2] = 1;
      values[3] = -0.5*pow(acc::controller_timestep,2);

      Ipopt::Index index;
      index = 4;
      for (size_t i = 2; i < m; i++)
      {
         if (i%2==0)
         {
            
            values[index] = -1;
            index++;

            values[index] = 1;
            index++;

            values[index] = -acc::controller_timestep;;
            index++;
         }else
         {
            values[index] = -acc::controller_timestep;
            index++;

            values[index] = -1;
            index++;

            values[index] = 1;
            index++;

            values[index] = -0.5*pow(acc::controller_timestep,2);
            index++;
         }
      }
   }

   return true;
}

bool MyNLP::eval_h(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number        obj_factor,
   Ipopt::Index         m,
   const Ipopt::Number* lambda,
   bool          new_lambda,
   Ipopt::Index         nele_hess,
   Ipopt::Index*        iRow,
   Ipopt::Index*        jCol,
   Ipopt::Number*       values
)
{
   assert(n == 3*k_);
   assert(m == 2*k_);
   
   if( values == NULL )
   {
      Ipopt::Index index;
      // return the structure of the jacobian of the constraints

      iRow[0]=0;
      jCol[0]=0;

      index = 1;
      for (size_t i = 1; i < n; i++)
      {
         iRow[index] = i;
         jCol[index] = i;
         index++;   
      } 
   }
   else
   {
      // return the values of the jacobian of the constraints
      values[0] = sped_weight*2*obj_factor;
      
      Ipopt::Index index;
      index = 1;

      for (size_t i = 1; i < n; i++)
      {
         
         if ((i-1)%3 == 0)
         {
           values[index] = -dist_weight*2*obj_factor;  
         
         }else if ((i-1)%3 == 1)
         {
            values[index] = accl_weight*2*obj_factor; 
         }else
         {
            values[index] = sped_weight*2*obj_factor;
         }   
         index++;   
         
      }
   }

   //index = index - 1;
   //assert(nele_hess == int(index))

   return true;
}

void MyNLP::finalize_solution(
   Ipopt::SolverReturn               status,
   Ipopt::Index                      n,
   const Ipopt::Number*              x,
   const Ipopt::Number*              z_L,
   const Ipopt::Number*              z_U,
   Ipopt::Index                      m,
   const Ipopt::Number*              g,
   const Ipopt::Number*              lambda,
   Ipopt::Number                     obj_value,
   const Ipopt::IpoptData*           ip_data,
   Ipopt::IpoptCalculatedQuantities* ip_cq
)
{
   // Open file to save data
   if(MyNLP::A==1)
   {
      MyNLP::begin_time = std::chrono::steady_clock::now();
      out.open("../output/results.txt");
      out << "#Simulate Step number,Current lidar reading,Estimated lead car speed from lidar reading,Acceleration command from MPC,Speed command from MPC,Distance command from MPC,Measure Speed,Update lidar reading,Time taken by task" << std::endl;
   }else{
      out.open("../output/results.txt",std::ios_base::app);
   }
   
   // Display custom log on terminal in real-time
   std::cout << std::endl;
   std::cout << "**********************************************************************" << std::endl;
   std::cout << "********** Custom Optimization Log after " << MyNLP::A << " th timeStep **********" << std::endl;
   std::cout << "**********************************************************************" << std::endl;
   std::cout << std::endl;
   std::cout << "Estimated Sesnor Reading before Optimization" << std::endl;
   std::cout <<"-----------------------------------------------" << std::endl;
   std::cout << "Current Ego's Speed = " << acc::cl_ego::ego_curr_velocity << " m/s" << std::endl;
   std::cout << "Front Lidar Reading (with noise) - Simulated = " << acc::cl_ego::dist_betw_cars << " meters" << std::endl;
   std::cout << "Estimated LeadCar velocity from above Lidar reading = " << acc::cl_ego::leadCar_curr_velocity << " m/s" << std::endl;
   
   std::cout << std::endl;
   std::cout << "1st timestep optimized State variable" << std::endl;
   std::cout <<"----------------------------------------" << std::endl;
   std::cout << "Next Acceleration command from IPOPT = " << x[2] << " m/ss" << std::endl;
   std::cout << "Expected Speed value of Ego by next timeStep from IPOPT = " << x[0] << " m/s" << std::endl;
   std::cout << "Expected Distance Ego will travel by next timeStep from IPOPT = " << x[1] << " meters " << std::endl;
   
   // trigger plantStep
   if (x[0] < 0.1 && x[3] < x[0])
   {
      // Intentionally triggering brake command once following speed command is less than 0.1 m/s (0.3 kmph) and the Ego anticipate to slow down further
      // based on current planned & optimized path (chain of v,s,a for each time timestep upto event horizon)
      Ipopt::Number a = -4.00;
      acc::cl_MPC::fn_sendControlOutputToPlant(int(MyNLP::A),a);

      std::cout << "Acceleration command send to Plant/Ego = " << a << " m/ss" << std::endl;
   }else
   {
      // TriggerPlantStep based on 1st Accel Command
      acc::cl_MPC::fn_sendControlOutputToPlant(int(MyNLP::A),x[2]);

      std::cout << "Acceleration command send to Plant/Ego = " << x[2] << " m/ss" << std::endl;
   }

   // printing some values after Step Update   
   std::cout << std::endl;
   std::cout << "Latest Speed & Lidar Reading after applying gas/brake" << std::endl;
   std::cout <<"-------------------------------------------------------" << std::endl;
   std::cout<<"Latest Ego's Speed after PlantStep = "<< acc::cl_ego::ego_curr_velocity << " m/s" << std::endl;
   std::cout<< "Latest Lidar Reading after PlantStep = "<< acc::cl_ego::dist_betw_cars << " meters" << std::endl;

   // time taken between plantStep (currently using this as metric to find task time) 
   MyNLP::end_time = std::chrono::steady_clock::now();
   opt_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(MyNLP::end_time - MyNLP::begin_time).count();
   // store next start time
   MyNLP::begin_time = std::chrono::steady_clock::now();

   // write actual statevariables in file in real-time 
   out << std::fixed << std::setprecision(4) << MyNLP::A << "," << acc::cl_ego::dist_betw_cars << "," << acc::cl_ego::leadCar_curr_velocity << "," << x[2] << "," <<  x[0] << "," << x[1] << "," << acc::cl_ego::ego_curr_velocity << "," << acc::cl_ego::dist_betw_cars << "," << MyNLP::opt_time_ms << std::endl;
   out.close();

   // added for debugging
   if (acc::cl_ego::ego_curr_velocity < 0)
    {
        exit(1);
    }

   // update stepCounter (for tracking step numbers)
   MyNLP::A++;   
}
