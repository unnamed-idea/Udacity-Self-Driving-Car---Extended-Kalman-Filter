#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  
//std::cout<<"kalmaninitbeg";
x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
std::cout<<"kalman initilized";
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */


//std::cout<<"kalmanpredictbeg";
x_ = F_ * x_;
P_ = F_*P_*F_.transpose() + Q_;


//std::cout<<"kalmanpredictEND";
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
//std::cout<<"kalmanupdatebeg";
VectorXd z_predict = H_ * x_;
VectorXd y = z - z_predict;
MatrixXd H_trans = H_.transpose();
MatrixXd S = H_*P_*H_trans+R_;
MatrixXd S_inverse = S.inverse();
MatrixXd K = (P_*H_trans)*S_inverse;

x_ = x_ + K*y;

MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

P_ = (I - K*H_)*P_;
//std::cout<<"kalmanupdateend";

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

//std::cout<<"kalmanUpdateEKFbeg";
double px = x_[0];
double py = x_[1];
double vx = x_[2];
double vy = x_[3];


//std::cout<<"check1";

if (fabs(px) < 0.0001){
px = 0.0001;
}

if (fabs(py) < 0.0001){
py = 0.0001;
}

double rho = sqrt(px*px +py*py);

//prevent rho approaching to zero
if (fabs(rho) < 0.0001){
rho = 0.0001;
}

double phi = atan2(py,px);


double rho_dot = (px*vx + py*vy)/rho;

//std::cout<<"check2";
VectorXd z_pred(3);

z_pred << rho, phi, rho_dot;

//std::cout<<"check3";
VectorXd y = z- z_pred;

y[1]=atan2(sin(y[1]),cos(y[1]));//implemented after mentors' suggestion of angle normalization

MatrixXd H_trans = H_.transpose();
MatrixXd S = H_ * P_ * H_trans + R_;
MatrixXd S_inverse = S.inverse();
MatrixXd K = (P_ * H_trans) * S_inverse;




//std::cout<<"check4";
x_ = x_ + (K * y);

//std::cout<<"check5";
MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

//std::cout<<"check6";
P_ = (I - K*H_)*P_;



//std::cout<<"kalmanUpdateEKFEND";



}
