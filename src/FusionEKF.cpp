#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
//std::cout<<"fusion1";
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

H_laser_ << 1, 0, 0, 0,
	0, 1, 0, 0;

 
//std::cout<<"fusion2";
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */



}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
 //   std::cout << "fusion3EKF: ";
//std::cout<<"fusion3.0";
 
	ekf_.x_ = VectorXd(4);
	ekf_.x_ << 1, 1, 1, 1;
	
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1000, 0,
			0, 0, 0, 1000;
//std::cout<<"fusion3.2";

	ekf_.F_ = MatrixXd(4,4);
	ekf_.F_ << 1, 0, 1, 0,
			0, 1, 0, 1,
			0, 0, 1, 0,
			0, 0, 0, 1;
//std::cout<<"fusion4";
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

//std::cout<<"radarFusionbeg";

      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      float px = rho*cos(phi);
      float py = rho*sin(phi);
	ekf_.x_ << px, py, rho_dot*cos(phi), rho_dot*sin(phi);
//std::cout<<"radarFusionend";

   }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	float x_meas = measurement_pack.raw_measurements_[0];
	float y_meas = measurement_pack.raw_measurements_[1];
	ekf_.x_ << x_meas, y_meas, 0., 0.;

//std::cout<<"laserFusionend";
    }

//std::cout<<"initializeDDD";
previous_timestamp_  = measurement_pack.timestamp_;
   // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

double d_time = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.;
previous_timestamp_ = measurement_pack.timestamp_;
//std::cout<<"fusion5";
if (d_time > 0.){

	ekf_.F_(0,2) = d_time;// since our motion model is constant velocity
	ekf_.F_(1,3) = d_time;
float noise_ax = 9.0;
float noise_ay = 9.0;


float d_time_2 = d_time * d_time;
float d_time_3 = d_time_2 * d_time;
float d_time_4 = d_time_3 * d_time;

ekf_.Q_ = MatrixXd(4,4);
ekf_.Q_ << ((d_time_4)*noise_ax)/4,	0,	((d_time_3)*noise_ax)/2,	0,
	   0			,	((d_time_4)*noise_ay)/4, 0, ((d_time_3)*noise_ay)/2,
	   ((d_time_3)*noise_ax)/2, 0, (d_time_2)*noise_ax, 0,
	0, ((d_time_3)*noise_ay)/2, 0, (d_time_2)*noise_ay;
//	std::cout<<"fusion6";
  ekf_.Predict();
}
/*
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	Tools tools;
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF( measurement_pack.raw_measurements_ ); 

 } else {
    // Laser updates
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
 	ekf_.Update( measurement_pack.raw_measurements_ );
 }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << endl;
  std::cout << "P_ = " << ekf_.P_ << endl;
}
