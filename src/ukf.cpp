#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <stdio.h>


#define DEBUG_PRINT 1

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  cout << "UKF Instatiation" << endl;

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;
  n_aug_ = n_x_ + 2;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);


  // TODO: change std_a_ and std_yawdd_
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4.0 / 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI / 16;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  lambda_ = 3 - n_aug_;

  //create vector for weights_
  weights_ = VectorXd(2*n_aug_+1);

  //set weights_
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // cout << "ProcessMeasurement" << endl;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = meas_package.raw_measurements_[0],
            phi = meas_package.raw_measurements_[1];

      // we do not have enough information to init vx, vt from ro rate
      x_ << ro * cos(phi), ro * sin(phi), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float px = meas_package.raw_measurements_[0],
            py = meas_package.raw_measurements_[1];
      x_ << px, py, 0, 0, 0;
    }

    previous_timestamp_ = meas_package.timestamp_;

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

   //compute the time elapsed between the current and previous measurements
   float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   previous_timestamp_ = meas_package.timestamp_;

   Prediction(dt);


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    cout << "UKF: update radar" << endl;

    // Radar updates
    UpdateRadar(meas_package);
    last_used_sensor_ = 1;
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    cout << "UKF: update laser" << endl;

    // Laser updates
    UpdateLidar(meas_package);
    last_used_sensor_ = 0;
  }

  // cout << "Update | result " << endl;

  // print the output
  // cout << "x_ = " << x_ << endl;
  // cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t){
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // cout << "..:: Prediction ::.." << endl;


  /*******************************************************************************
   * Create augmented sigma points
   ******************************************************************************/

  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;

  double aux = sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; i++)
  {
    VectorXd right_part = aux * A_aug.col(i);
    Xsig_aug.col(i+1)        = x_aug + right_part;
    Xsig_aug.col(i+1+n_aug_) = x_aug - right_part;
  }

  /*******************************************************************************
   * Sigma point prediction
   ******************************************************************************/

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  VectorXd Xaux = VectorXd(n_x_);
  VectorXd Xnoise = VectorXd(n_x_);

  //predict sigma points
  for (int i=0; i<2 * n_aug_ + 1;i++){
      double px = Xsig_aug(0, i),
             py = Xsig_aug(1, i),
             v = Xsig_aug(2, i),
             yaw = Xsig_aug(3, i),
             yaw_d = Xsig_aug(4, i),
             noise_a = Xsig_aug(5, i),
             noise_yaw = Xsig_aug(6, i);

      Xnoise << 0.5 * delta_t * delta_t * cos(yaw) * noise_a,
                0.5 * delta_t * delta_t * sin(yaw) * noise_a,
                delta_t * noise_a,
                0.5 * delta_t * delta_t * noise_yaw,
                delta_t * noise_yaw;

      if (fabs(yaw_d) <= 0.001){
          Xaux << v * cos(yaw) * delta_t,
                  v * sin(yaw) * delta_t,
                  0,
                  0,
                  0;
      }
      else {
          Xaux << (v / yaw_d) * ( sin(yaw + yaw_d * delta_t) - sin(yaw)),
                  (v / yaw_d) * ( -cos(yaw + yaw_d * delta_t) + cos(yaw)),
                  0,
                  yaw_d * delta_t,
                  0;
      }

      Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) + Xaux + Xnoise;
  }
    // cout << "Prediction | Xsig_pred_=" << Xsig_pred_ << endl;


  /*******************************************************************************
   * Predicted mean and covariance
   ******************************************************************************/

  x_ = weights_(0) * Xsig_pred_.col(0);

  //predict state mean
  for(int i=1; i<2 * n_aug_ + 1; i++)
      x_ += weights_(i) * Xsig_pred_.col(i);

  //predict state covariance matrix
  P_.fill(0);
  for(int i=0; i<2 * n_aug_ + 1; i++){
      // cout << "Prediction | i=" << i << "/" << 2 * n_aug_ + 1 << endl;
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      // cout << "Prediction | after x_diff=" << x_diff <<" i=" << i << "/" << 2 * n_aug_ + 1 << endl;
      // cout << "Prediction | i=" << M_PI * 2 + 2.15 << endl;
      // cout << "Prediction | i=" << floor((M_PI * 2 + 2.15) / M_PI) << " angle > M_PI" << endl;

      while (x_diff(3) > M_PI)  x_diff(3) -= 2.*M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
      // cout << "Prediction | angle=" << x_diff(3) << " i=" << i << "/" << 2 * n_aug_ + 1 << endl;

      P_ += weights_(i) * (x_diff * x_diff.transpose());
      // cout << "Prediction | P_ updated" <<" i=" << i << "/" << 2 * n_aug_ + 1 << endl;
  }
  // cout << "Prediction | finish" << endl;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // cout << "LidarUpdate | update" << endl;

  /*******************************************************************************
   * Predict into radar measurement sace
   ******************************************************************************/
  int n_z = 2;  //number of measurement variables

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  MatrixXd R = MatrixXd::Zero(n_z, n_z);

  //transform sigma points into measurement space
  for(int i=0; i<2 * n_aug_ + 1; i++){
        float px = Xsig_pred_(0, i),
              py = Xsig_pred_(1, i);

        Zsig(0, i) = px;
        Zsig(1, i) = py;
  }

  //calculate mean predicted measurement
  z_pred = weights_(0) * Zsig.col(0);
  for(int i=1; i<2 * n_aug_ + 1; i++)
    z_pred += weights_(i) * Zsig.col(i);

  //calculate measurement covariance matrix S
  for(int i=0; i<2 * n_aug_ + 1; i++){
      VectorXd z_diff = Zsig.col(i) - z_pred;
      // angle normalization
      while (z_diff(1) >  M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1) < -M_PI) z_diff(1)+=2.*M_PI;
      S += weights_(i) * z_diff * z_diff.transpose();
  }

  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;
  S += R;


  /*******************************************************************************
  * Update
  ******************************************************************************/

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //calculate cross correlation matrix
  for(int i=0; i< 2 * n_aug_ + 1; i++){
      VectorXd Xaux = Xsig_pred_.col(i) - x_,
               Zaux = Zsig.col(i) - z_pred;
      Tc += weights_(i) * Xaux * Zaux.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // cout << "S = " << S << endl;
  // cout << "R = " << R << endl;
  // cout << "Tc = " << Tc << endl;
  // cout << "K = " << K << endl;


  //update state mean and covariance matrix
  x_ = x_ + K * (meas_package.raw_measurements_ - z_pred);
  P_ = P_ - K * S * K.transpose();

  // compute NIS
  VectorXd Zdiff = z_pred - meas_package.raw_measurements_;
  NIS_ = Zdiff.transpose() * S.inverse() * Zdiff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  You'll also need to calculate the radar NIS.
  */

  // cout << "RadarUpdate | update" << x_ << endl;

  /*******************************************************************************
   * Predict into radar measurement sace
   ******************************************************************************/
  int n_z = 3;  //number of measurement variables

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);
  MatrixXd R = MatrixXd::Zero(3, 3);

  //transform sigma points into measurement space
  for(int i=0; i<2 * n_aug_ + 1; i++){
        float px = Xsig_pred_(0, i),
              py = Xsig_pred_(1, i),
              v = Xsig_pred_(2, i),
              teta = Xsig_pred_(3, i),
              teta_d = Xsig_pred_(4, i);

        Zsig(0, i) = sqrt(px*px + py*py);
        Zsig(1, i) = atan2(py, px);
        Zsig(2, i) = v * (px*cos(teta) + py*sin(teta)) / Zsig(0, i);
  }

  //calculate mean predicted measurement
  z_pred = weights_(0) * Zsig.col(0);
  for(int i=1; i<2 * n_aug_ + 1; i++)
    z_pred += weights_(i) * Zsig.col(i);

  //calculate measurement covariance matrix S
  for(int i=0; i<2 * n_aug_ + 1; i++){
      VectorXd z_diff = Zsig.col(i) - z_pred;
      // angle normalization
      while (z_diff(1) >  M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1) < -M_PI) z_diff(1)+=2.*M_PI;
      // compute measurement covariance
      S += weights_(i) * z_diff * z_diff.transpose();
  }

  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  S += R;

  /*******************************************************************************
  * Update
  ******************************************************************************/

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //calculate cross correlation matrix
  for(int i=0; i< 2 * n_aug_ + 1; i++){
      VectorXd Xaux = Xsig_pred_.col(i) - x_,
               Zaux = Zsig.col(i) - z_pred;

     // angle normalization for measurement diff
     while (Zaux(1) >  M_PI) Zaux(1) -= 2.*M_PI;
     while (Zaux(1) < -M_PI) Zaux(1) += 2.*M_PI;

     // angle normalization for measurement diff
     while (Xaux(3) >  M_PI) Xaux(3) -= 2.*M_PI;
     while (Xaux(3) < -M_PI) Xaux(3) += 2.*M_PI;

      Tc += weights_(i) * Xaux * Zaux.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // cout << "S = " << S << endl;
  // cout << "R = " << R << endl;
  // cout << "Tc = " << Tc << endl;
  // cout << "K = " << K << endl;

  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  VectorXd Zdiff = z_pred - meas_package.raw_measurements_;
  NIS_ = Zdiff.transpose() * S.inverse() * Zdiff;
}
