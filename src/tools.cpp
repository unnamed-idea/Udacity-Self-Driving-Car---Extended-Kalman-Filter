#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools()  {}
Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

//std::cout<<"toolscalculatermsegBEGIN";
VectorXd rmse(4);
rmse << 0, 0, 0, 0;

if (estimations.size() ==0){

// do not process
return rmse;
}

unsigned int i = 0;

for (i =0; i<estimations.size(); ++i){
	VectorXd resid = estimations[i] - ground_truth[i];
	resid = resid.array() * resid.array();
	rmse += resid;
}

rmse = rmse / estimations.size();

rmse = rmse.array().sqrt();

//std::cout<<"toolscalculatermsegEND";
return rmse;



}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */


//std::cout<<"toolscalculatejacobianBEG";
MatrixXd Hj(3,4);

double px = x_state(0);

double py = x_state(1);
double vx = x_state(2);
double vy = x_state(3);


double square_sum = px*px + py*py;

if (square_sum <0.00001){

	px+=0.001;
	py+=0.001;
	square_sum= px*px + py*py;
}

double sqrt_sum = sqrt(square_sum);
double product = square_sum*sqrt_sum;

Hj << (px/sqrt_sum), (py/sqrt_sum), 0, 0,
	-(py/square_sum), (px/square_sum), 0, 0,
	py*(vx*py - vy*px)/product, px*(px*vy - py*vx)/product, px/sqrt_sum, py/sqrt_sum;

//std::cout<<"toolscalculatejacobianEND";
return Hj;


}
