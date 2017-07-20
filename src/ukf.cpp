#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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
  is_initialized_ = false;
  n_x_ = 5;
  lambda_ = 3;
  x_ << 0, 0, 0, 0, 0;
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 0.1;
  n_aug_ = 7;
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) weights_(i) = 0.5/(n_aug_ + lambda_);
   Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package, Tools *tools) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
      x_(0) = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      x_(1) = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
    } else  {
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
    }
    tools->px = 0;
    tools->py = 0;
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && !use_radar_) return;
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && !use_laser_) return;

  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);
  tools->px = x_(0);
  tools->py = x_(1);

  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  } else  {
    if (use_radar_) UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // generate augmented sigma points
  //create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_*std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;
  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int j=0; j < n_aug_; j++) {
    Xsig_aug.col(j+1)       = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(j);
    Xsig_aug.col(j+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(j);
  }

  // predicte sigma points
  for (int i=0; i < 2*n_aug_+1; i++) {
    double dt = delta_t;
    double dt2 = dt*dt;
    double vk = Xsig_aug(2,i);
    double pk = Xsig_aug(3,i);
    double pdk = Xsig_aug(4,i);
    double muak = Xsig_aug(5,i);
    double mupddk = Xsig_aug(6,i);
    if (fabs(pdk) > 0) {
      Xsig_pred_(0,i) = Xsig_aug(0,i) + vk*(sin(pk + pdk*dt) - sin(pk))/pdk;
      Xsig_pred_(1,i) = Xsig_aug(1,i) + vk*(-cos(pk + pdk*dt) + cos(pk))/pdk;
      Xsig_pred_(2,i) = Xsig_aug(2,i) + 0;
      Xsig_pred_(3,i) = Xsig_aug(3,i) + pdk*dt;
      Xsig_pred_(4,i) = Xsig_aug(4,i) + 0;
    } else {
      Xsig_pred_(0,i) = Xsig_aug(0,i) + vk*cos(pk)*dt;
      Xsig_pred_(1,i) = Xsig_aug(1,i) + vk*sin(pk)*dt;
      Xsig_pred_(2,i) = Xsig_aug(2,i) + 0;
      Xsig_pred_(3,i) = Xsig_aug(3,i) + pdk*dt;
      Xsig_pred_(4,i) = Xsig_aug(4,i) + 0;
    }
    Xsig_pred_(0,i) += dt2*cos(pk)*muak/2;
    Xsig_pred_(1,i) += dt2*sin(pk)*muak/2;
    Xsig_pred_(2,i) += dt*muak;
    Xsig_pred_(3,i) += dt2*mupddk/2;
    Xsig_pred_(4,i) += dt*mupddk;
  }

  //predict state mean
  //predict state covariance matrix
  x_.fill(0.0);
  P_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    x_ += weights_(i)*Xsig_pred_.col(i);
  }
  for (int i=0; i < 2*n_aug_+1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ +=  weights_(i) * x_diff * x_diff.transpose() ;
  }

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
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  VectorXd z = VectorXd(n_z);
  z <<
      meas_package.raw_measurements_[0],   //rho in m
      meas_package.raw_measurements_[1],   //phi in rad
      meas_package.raw_measurements_[2];   //rho_dot in m/s
  // predict measurement
  //transform sigma points into measurement space
  //calculate mean predicted measurement
  //calculate measurement covariance matrix S
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z); //measurement covariance matrix S
  for (int i=0; i < 2*n_aug_+1; i++) {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double psy = Xsig_pred_(3,i);
    double sqrtpx2py2 = sqrt(px*px + py*py);
    Zsig(0,i) = sqrtpx2py2;
    Zsig(1,i) = atan2(py,px);
    Zsig(2,i) = (px*cos(psy)*v + py*sin(psy)*v)/sqrtpx2py2;
  }

  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred += weights_(i)*Zsig.col(i);
  }

  S.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      VectorXd x_diff = Zsig.col(i) - z_pred;
      //angle normalization
      while (x_diff(1)> M_PI) x_diff(1)-=2.*M_PI;
      while (x_diff(1)<-M_PI) x_diff(1)+=2.*M_PI;
      S += weights_(i) * x_diff*x_diff.transpose();
  }
  S(0,0) += std_radr_*std_radr_;
  S(1,1) += std_radphi_*std_radphi_;
  S(2,2) += std_radrd_*std_radrd_;

  // update state
  //calculate cross correlation matrix
  //calculate Kalman gain K;
  //update state mean and covariance matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  MatrixXd K = Tc*S.inverse();
  //residual
  VectorXd z_diff = z - z_pred;
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ += K * z_diff;
  P_ -= K*S*K.transpose();
}
