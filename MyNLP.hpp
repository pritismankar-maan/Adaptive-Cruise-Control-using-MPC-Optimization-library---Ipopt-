// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MYNLP_HPP__
#define __MYNLP_HPP__
#include "/usr/local/include/coin-or/IpIpoptData.hpp"
#include "/usr/local/include/coin-or/IpTNLP.hpp"
#include "/usr/local/include/coin-or/IpTimingStatistics.hpp"
#include "mpc.hpp"
#include <fstream>

class MyNLP: public Ipopt::TNLP, acc::cl_MPC
{
public:
   // custom variables 
   std::chrono::steady_clock::time_point begin_time;
   std::chrono::steady_clock::time_point end_time;
   int64_t opt_time_ms;
   static double A;
   
   /** default constructor */
   MyNLP(const int &k);

   /** default destructor */
   virtual ~MyNLP();

   /**@name Overloaded from TNLP */
   //@{
   /** Method to return some info about the nlp */
   virtual bool get_nlp_info(
      Ipopt::Index&          n,
      Ipopt::Index&          m,
      Ipopt::Index&          nnz_jac_g,
      Ipopt::Index&          nnz_h_lag,
      Ipopt::TNLP::IndexStyleEnum& index_style
   );

   /** Method to return the bounds for my problem */
   virtual bool get_bounds_info(
      Ipopt::Index   n,
      Ipopt::Number* x_l,
      Ipopt::Number* x_u,
      Ipopt::Index   m,
      Ipopt::Number* g_l,
      Ipopt::Number* g_u
   );

   /** Method to return the starting point for the algorithm */
   virtual bool get_starting_point(
      Ipopt::Index   n,
      bool    init_x,
      Ipopt::Number* x,
      bool    init_z,
      Ipopt::Number* z_L,
      Ipopt::Number* z_U,
      Ipopt::Index   m,
      bool    init_lambda,
      Ipopt::Number* lambda
   );

   /** Method to return the objective value */
   virtual bool eval_f(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Number&       obj_value
   );

   /** Method to return the gradient of the objective */
   virtual bool eval_grad_f(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Number*       grad_f
   );

   /** Method to return the constraint residuals */
   virtual bool eval_g(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Index         m,
      Ipopt::Number*       g
   );

   /** Method to return:
    *   1) The structure of the Jacobian (if "values" is NULL)
    *   2) The values of the Jacobian (if "values" is not NULL)
    */
   virtual bool eval_jac_g(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Index         m,
      Ipopt::Index         nele_jac,
      Ipopt::Index*        iRow,
      Ipopt::Index*        jCol,
      Ipopt::Number*       values
   );

   /** Method to return:
    *   1) The structure of the Hessian of the Lagrangian (if "values" is NULL)
    *   2) The values of the Hessian of the Lagrangian (if "values" is not NULL)
    */
   virtual bool eval_h(
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
   );

   /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
   virtual void finalize_solution(
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
   );
   //@}

private:
   int k_; // number of timesteps in control horizon
   double accl_weight {0.001}; //0.01
   double sped_weight {10};   //10
   double dist_weight {0.001};//0.001
   std::ofstream out;
   /**@name Methods to block default compiler methods.
    *
    * The compiler automatically generates0 the following three methods.
    *  Since the default compiler implementation is generally not what
    *  you want (for all but the most simple classes), we usually
    *  put the declarations of these methods in the private section
    *  and never implement them. This prevents the compiler from
    *  implementing an incorrect "default" behavior without us
    *  knowing. (See Scott Meyers book, "Effective C++")
    */
   //@{
   MyNLP(
      const MyNLP&
   );

   MyNLP& operator=(
      const MyNLP&
   );
   //@}
};

// custom - need to initialize static variable outside of class and not in any functions !! 
double MyNLP::A = 0;
#endif
