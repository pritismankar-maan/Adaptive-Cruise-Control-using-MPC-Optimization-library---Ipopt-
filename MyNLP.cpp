// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include "MyNLP.hpp"
//#include "mpc.cpp"

#include <cassert>
#include <iostream>
#include <fstream>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

//using namespace Ipopt;

/* Constructor. */
MyNLP::MyNLP(const int &k):k_(k)
{ }

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
   // The problem described in MyNLP.hpp has 2 variables, x1, & x2,
   n = 3*k_;

   // one equality constraint,
   m = 2*k_;

   // 2 nonzeros in the jacobian (one for x1, and one for x2),
   nnz_jac_g = 7*k_ - 3;

   // and 2 nonzeros in the hessian of the lagrangian
   // (one in the hessian of the objective for x2,
   //  and one in the hessian of the constraints for x1)
   nnz_h_lag = 3*k_;

   // We use the standard fortran Ipopt::Index style for row/col entries
   Index_style = C_STYLE; //FORTRAN_STYLE;

   // declare few constants before starting optimization for the current timestep 
   //B = 1/(pow(acc::controller_timestep,2));
   //A = (B*(pow(acc::cl_ego::ego_curr_velocity,2)) + 5*(pow(acc::cl_ego::speed_limit,2)));

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
         // set lower and upper limit of constraints
         g_l[i] = 0;
         g_u[i] = 0;

         // set lower and upper limit of parameters/ optimization variables
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
         // set lower and upper limit of parameters/ optimization variables
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
         }
         else
         {
            // acceleration limit
            x_l[i] = -4;
            x_u[i] = 2;
         }
      }
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

   // give the last 
   //std::copy(opt_X, opt_X + n, x);

   //intialize to zero
   for (Ipopt::Index i = 0; i < n; i++)
   {
      x[i] = 0;
   }
   
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
   
   //obj_value = obj_value + 10000000*pow((1/(acc::cl_ego::dist_betw_cars - x[298])),2);

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
   
   if(A==1)
   {
      out.open("results.txt");
      out << "Simulate Step number " << "," << "Current lidar reading" << "," << "Estimated lead car speed from lidar reading" << "," << "Acceleration command from MPC" << "," <<  "Speed command from MPC" << "," << "Distance command from MPC" << "," << "Measure Speed" << "," << "Update lidar reading" << std::endl;
   }else{
      out.open("results.txt",std::ios_base::app);
   }
   

   // here is where we would store the solution to variables, or write to a file, etc
   // so we could use the solution. Since the solution is displayed to the console,
   // we currently do nothing here.
   std::cout << "*****************************************************" << std::endl;
   std::cout << "Current lidar reading = "<<acc::cl_ego::dist_betw_cars<<std::endl;
   std::cout << "Estimated lead car speed from lidar reading = "<<acc::cl_ego::leadCar_curr_velocity<<std::endl;

    

   if(A==1357)
   {
      double d = 1;
   }

   std::cout << "Acceleration command from MPC = " << x[2] << std::endl;
   std::cout << "Speed command from MPC  = " << x[0] << std::endl;
   std::cout << "Distance command from MPC = " << x[1] << std::endl;
   acc::cl_MPC::fn_sendControlOutputToPlant(int(A),x[2]);   
   scnd_acc_from_prev_step = x[5];


   out << A << "," << acc::cl_ego::dist_betw_cars << "," << acc::cl_ego::leadCar_curr_velocity << "," << x[2] << "," <<  x[0] << "," << x[1] << "," << acc::cl_ego::ego_curr_velocity << "," << acc::cl_ego::dist_betw_cars << std::endl;

   /* if(status == Ipopt::SolverReturn::SUCCESS or status == Ipopt::SolverReturn::STOP_AT_ACCEPTABLE_POINT)
   {
      std::cout << "Acceleration command from MPC = " << x[2] << std::endl;
      std::cout << "Speed command from MPC  = " << x[0] << std::endl;
      std::cout << "Distance command from MPC = " << x[1] << std::endl;
      acc::cl_MPC::fn_sendControlOutputToPlant(int(A),x[2]);   
      scnd_acc_from_prev_step = x[5];


      out << A << "," << acc::cl_ego::dist_betw_cars << "," << acc::cl_ego::leadCar_curr_velocity << "," << x[2] << "," <<  x[0] << "," << x[1] << "," << acc::cl_ego::ego_curr_velocity << "," << acc::cl_ego::dist_betw_cars << std::endl;
   }
   else
   {
      std::cout << "Acceleration command from MPC = " << scnd_acc_from_prev_step << std::endl;
      std::cout << "Speed command from MPC  = " << x[0] << std::endl;
      std::cout << "Distance command from MPC = " << x[1] << std::endl;
      std::cout << "Error in MPC =============================================== " << status << std::endl;
      acc::cl_MPC::fn_sendControlOutputToPlant(int(A),scnd_acc_from_prev_step);

      out << A << "," << acc::cl_ego::dist_betw_cars << "," << acc::cl_ego::leadCar_curr_velocity << "," << scnd_acc_from_prev_step << "," <<  x[0] << "," << x[1] << "," << acc::cl_ego::ego_curr_velocity << "," << acc::cl_ego::dist_betw_cars << std::endl;
   } */
   
   std::cout<<"Simulate Step number (Accl cmd send to car ...) = "<<A<<std::endl;
   
   std::cout<<"Measure Speed = "<<acc::cl_ego::ego_curr_velocity<<std::endl;
   std::cout<< "Update lidar reading = "<<acc::cl_ego::dist_betw_cars<<std::endl;

   out.close();

   A++;
   
   //std::copy(x, x + n, opt_X);
}
