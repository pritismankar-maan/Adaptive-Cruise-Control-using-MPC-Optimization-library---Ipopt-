// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MYNLP_HPP__
#define __MYNLP_HPP__

#include "/usr/local/include/coin-or/IpTNLP.hpp"
#include "mpc.hpp"
#include <fstream>

//using namespace Ipopt;

/** C++ Example NLP for interfacing a problem with IPOPT.
 *  MyNLP implements a C++ example showing how to interface with IPOPT
 *  through the TNLP interface. This example is designed to go along with
 *  the tutorial document (see Examples/CppTutorial/).
 *  This class implements the following NLP.
 *
 * min_x f(x) = -(x2-2)^2
 *  s.t.
 *       0 = x1^2 + x2 - 1
 *       -1 <= x1 <= 1
 *
 */
class MyNLP: public Ipopt::TNLP, acc::cl_MPC
{
public:
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
   Ipopt::Number* opt_X;
   Ipopt::Number scnd_acc_from_prev_step;
   double accl_weight {0.001}; //0.01
   double sped_weight {10};   //10
   double dist_weight {0.001};//0.001
   double B;
   double A;
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

#endif
