#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

const double Lf = 2.67;

/// variables position in vector
size_t x_start 		= 0;
size_t y_start 		= x_start + N;
size_t psi_start 	= y_start + N;
size_t v_start 		= psi_start + N;
size_t cte_start 	= v_start + N;
size_t epsi_start 	= cte_start + N;
size_t delta_start 	= epsi_start + N;
size_t a_start 		= delta_start + N - 1;

class FG_eval {
 public:
  /// Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    fg[0] = 0;
 
    /// The part of the cost based on the reference state.
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += w_cte 	* vars[cte_start + t] * vars[cte_start + t];
      fg[0] += w_epsi 	* vars[epsi_start + t] * vars[epsi_start + t];
      fg[0] += w_dv 	* (vars[v_start + t] - ref_v)*(vars[v_start + t] - ref_v);
    }

    /// Minimize the use of actuators.
    for (unsigned int t = 0; t < N - 1; t++) {
      fg[0] += w_delta 	* vars[delta_start + t] * vars[delta_start + t] ;
      fg[0] += w_a 		* vars[a_start + t] * vars[a_start + t];
    }

    /// Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; t++) {
      fg[0] += w_d_delta 	* CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_d_a 		* CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    /// Initial constraints
    /// Note: 1 added to each of the starting indices due to cost being located at 0

    fg[1 + x_start] 	= vars[x_start];
    fg[1 + y_start] 	= vars[y_start];
    fg[1 + psi_start] 	= vars[psi_start];
    fg[1 + v_start] 	= vars[v_start];
    fg[1 + cte_start] 	= vars[cte_start];
    fg[1 + epsi_start] 	= vars[epsi_start];

    // The rest of the constraints
    for (unsigned int t = 1; t < N; t++) {
      // The state at time t .
      AD<double> x1 	= vars[x_start + t];
      AD<double> y1 	= vars[y_start + t];
      AD<double> psi1 	= vars[psi_start + t];
      AD<double> v1 	= vars[v_start + t];
      AD<double> cte1 	= vars[cte_start + t];
      AD<double> epsi1 	= vars[epsi_start + t];

      // The state at time t-1
      AD<double> x0 	= vars[x_start + t - 1];
      AD<double> y0 	= vars[y_start + t - 1];
      AD<double> psi0 	= vars[psi_start + t - 1];
      AD<double> v0 	= vars[v_start + t - 1];
      AD<double> cte0 	= vars[cte_start + t - 1];
      AD<double> epsi0 	= vars[epsi_start + t - 1];

      // Actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 	= vars[a_start + t - 1];

      //AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      AD<double> f0 		= coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 ;
      //atan of the derivative
      AD<double> psides0 	= CppAD::atan(coeffs[1]+2* coeffs[2]* x0);
      
      /// steps itteration  
      fg[1 + x_start + t] 		= x1 - 		(x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] 		= y1 - 		(y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] 	= psi1 - 	(psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] 		= v1 - 		(v0 + a0 * dt);
      fg[1 + cte_start + t] 	= cte1 - 	((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] 	= epsi1 - 	((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  
  };
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x 		= state[0];
  double y 		= state[1];
  double psi 	= state[2];
  double v 		= state[3];
  double cte 	= state[4];
  double epsi 	= state[5];

  /// Set the number of model variables (includes both states and inputs).
  /// N timesteps -> N - 1 actuations
  size_t n_vars = 6 * N + 2 * (N-1);
  size_t n_constraints = 6 * N;

  /// Initial value of the independent variables.
  /// SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  
  /// Set the initial variable values
  vars[x_start] 	= x;
  vars[y_start] 	= y;
  vars[psi_start] 	= psi;
  vars[v_start] 	= v;
  vars[cte_start] 	= cte;
  vars[epsi_start] 	= epsi;
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
    
  /// Set all non-actuators upper and lowerlimits
  /// to the max negative and positive values.
  for (unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  /// The upper and lower limits of delta are set to -25 and 25
  /// degrees (values in radians).
  /// NOTE: Feel free to change this to something else.
  for (unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  /// Acceleration/decceleration upper and lower limits.
  /// NOTE: Feel free to change this to something else.
  for (unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  /// Lower and upper limits for the constraints
  /// Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
 
   for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  /// init state
  
  constraints_lowerbound[x_start] 		= x;
  constraints_lowerbound[y_start] 		= y;
  constraints_lowerbound[psi_start]		= psi;
  constraints_lowerbound[v_start] 		= v;
  constraints_lowerbound[cte_start] 	= cte;
  constraints_lowerbound[epsi_start] 	= epsi;

  constraints_upperbound[x_start] 		= x;
  constraints_upperbound[y_start] 		= y;
  constraints_upperbound[psi_start] 	= psi;
  constraints_upperbound[v_start] 		= v;
  constraints_upperbound[cte_start] 	= cte;
  constraints_upperbound[epsi_start] 	= epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.05\n";

  /// place holder for solution
  CppAD::ipopt::solve_result<Dvector> solution;

  /// solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  /// Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  /// Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  
  /// Return the first actuator values.
  vector<double> result;

  result.push_back(solution.x[a_start]);
  result.push_back(solution.x[delta_start]);
  result.push_back((double) N);
  
  for (unsigned int i=0; i < N; i++) {
	result.push_back(solution.x[x_start + i]);
	result.push_back(solution.x[y_start + i]);
  }
  
  return result;
}
