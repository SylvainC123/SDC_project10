#ifndef MPC_H
#define MPC_H

  
/// init target speed
#define ref_v 		80
/// hyperparameters for the cost function
#define w_cte 		5		// cross track error
#define w_epsi 		1		// direction error
#define w_dv 		1		// gap to target speed
#define w_a 		1		// throtle gap
#define w_delta 	1e2     //5000		// steering gap
#define w_d_a 		0		// throtle continuity factor
#define w_d_delta 	1e6		// steering continuity factor

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
