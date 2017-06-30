#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() == 0) {
      return rmse;
  }
  if (estimations.size() != ground_truth.size()){
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd residual = estimations[i]-ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;

  }

  //calculate the mean
  // ... your code here
    rmse = rmse/estimations.size();
  //calculate the squared root
  // ... your code here
    rmse = rmse.array().square();
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float div = px*px+py*py;
  float div2 = sqrt(div);
  float div32 = div*div2;

	if (fabs(div) > 0.0001) {
    Hj(0,0) = px/div2;
    Hj(0,1) = py/div2;
    Hj(1,0) = -py/div;
    Hj(1,1) = px/div;
    Hj(2,0) = py*(vx*py-vy*px)/div32;
    Hj(2,1) = px*(vy*px-vx*py)/div32;
    Hj(2,2) = px/div2;
    Hj(2,3) = py/div2;
	} else {
	    cout<<"Jacobian Zero Division Error\n";
	}

	return Hj;
}
