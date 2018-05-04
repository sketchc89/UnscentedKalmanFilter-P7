#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "spdlog/spdlog.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  // ******************************************************************************************
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  // ******************************************************************************************
  
  //TODO Hint: one or more values initialized above might be wildly off...

  // Time when state is true in microseconds
  is_initialized_ = false;

  // State Dimension using CTRV Model
  n_x_ = 5;
  // Augmented State Dimension with uncertainty added
  n_aug_ = 7;

  n_sig_ = 2*n_aug_ + 1;
  n_rad_ = 3;
  n_las_ = 2;

  // Spreading factor
  lambda_ = 3 - n_aug_; 

  // Initial predicted sigma points
  Xsig_ = MatrixXd(n_aug_, n_sig_);
  Xsig_.setZero();

  MatrixXd R_rad_ = MatrixXd(n_rad_, n_rad_);
  R_rad_ << std_radr_*std_radr_,0,0,
            0,std_radphi_*std_radphi_, 0,
            0,0,std_radrd_*std_radrd;
  MatrixXd R_las_ = MatrixXd(n_las_, n_las_);
  R_las_ << std_laspx_*std_laspx_, 0.0,
            0.0, std_laspy_*std_laspy_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} measurement_pack The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  time_us_ = measurement_pack.timestamp_;
  if (!is_initialized_){
    auto console = spdlog::stdout_logger_mt("console");
    console->info("First measurement");
    console->info("Initializing State");
    InitializeState(measurement_pack);
    console->info("Initializing Sigma Points");
    InitializeSigmaPoints();
    is_initialized_ = true;
    return;
  }
  
  double dt;
  dt  = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
  Prediction(dt);
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    console->debug("Updating RADAR");
    UpdateRadar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    console->debug("Updating LIDAR");
    UpdateLidar(measurement_pack);
  }
  std::string x_name="x", P_name="P";
  calc.PrintMatrix(x_name, x_);
  calc.PrintMatrix(P_name, P_);
}

/*
 * Initializes state from first measurement 
 */
void UKF::InitializeState(MeasurementPackage measurement_pack) {
  
  time_us_ = measurement_pack.timestamp_;
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //console->debug("Initializing with RADAR");
    double rho = measurement_pack.raw_measurements_(0);
    double phi = calc.NormalizePhi(measurement_pack.raw_measurements_(1));
    double rho_dot = measurement_pack.raw_measurements_(2);
    
      x_ << rho*cos(phi),
          rho*sin(phi),
          0,
          0,
          0;
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    x_ << measurement_pack.raw_measurements_(0),
          measurement_pack.raw_measurements_(1),
          0,
          0,
          0;
  }
}

/*
 * Initialize sigma points
 */
void UKF::InitializeSigmaPoints() {

  //Calculate sqrt of P_
  MatrixXd A = P_.llt().matrixL();
  //First sigma point is mean
  Xsig_.col(0) = x_;
  //Calculate remaining sigma points from mean and spreading parameter 
  for (int i=0; i<n_x_; ++i) {
    Xsig_.col(i+1)       = x_ + sqrt(lambda_+n_x_) + A.col(i); Xsig_.col(i+1+n_x_)  = x_ - sqrt(lambda_+n_x_) + A.col(i);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  PredictSigmaPoints(delta_t);
  VectorXd weights = SetWeights();
  VectorXd x_pred = VectorXd(n_sig_);
  VectorXd P_pred = VectorXd(n_sig_);
  x_pred.setZero();
  P_pred.setZero();

  //predicted state mean
  for (int i=0; i<n_sig_; ++i) {
   x_pred = x_pred + weights(i) * Xsig_.col(i);
  }

  //predicted state covariance matrix
  for (int i=0; i<n_sig_; ++i) {
    VectorXd x_diff = Xsig_.col(i) - x_pred;
    x_diff(3) = calc.NormalizePhi(x_diff(3));
    P_pred = P_pred + weights(i) * x_diff * x_diff.transpose();
  }

  x_ = x_pred;
  P_ = P_pred;
}

VectorXd SetWeights(){
  VectorXd weights = VectorXd(n_sig_);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<n_sig_; ++i) {
    weights(i) = 0.5/(n_aug_+lambda);
  }
  return weights;
}

/**
 * Predicts sigma points given the state and time since the last measurement
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::PredictSigmaPoints(double delta_t) {
  for (int i=0; i<n_sig_; ++i) {
    double p_x  =     Xsig_(0, i);
    double p_y  =     Xsig_(1, i);
    double v    =     Xsig_(2, i);
    double yaw  =     Xsig_(3, i);
    double yawd =     Xsig_(4, i);
    double nu_a =     Xsig_(5, i);
    double nu_yawdd = Xsig_(6, i);

    //predict state values with CTRV
    double px_p, py_p;
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    double v_p    = v;
    double yaw_p  = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p    = px_p  + 0.5*nu_a*delta_t*delta_t*cos(yaw);
    py_p    = py_p  + 0.5*nu_a*delta_t*delta_t*sin(yaw);
    v_p     = v_p   + nu_a*delta_t;
    yaw_p   = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p  = yawd_p+ nu_yawdd*delta_t;

    //write predicted sigma points
    Xsig_(0, i) = px_p;
    Xsig_(1, i) = py_p;
    Xsig_(2, i) = v_p;
    Xsig_(3, i) = yaw_p;
    Xsig_(4, i) = yawd_p;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
  MatrixXd Zsig = MatrixXd(n_las_, n_sig_);
  weights = SetWeights();
  for (int i=0; i<n_sig_; ++i) {
    //extract values for readability
    double p_x  =     Xsig_(0, i);
    double p_y  =     Xsig_(1, i);
    double v    =     Xsig_(2, i);
    double yaw  =     Xsig_(3, i);
    double v_x  =     v*cos(yaw);
    double v_y  =     v*sin(yaw);

    //measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_las_);
  z_pred.setZero();
  for (int i=0; i<n_sig_; ++i) {
    z_pred = z_pred + weights(i)*Zsig.col(i);
  }

  //innovation coavariance matrix S
  MatrixXd S = MatrixXd(n_las_, n_las_);
  S.setZero();
  for (int i=0; i<n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = calc.NormalizePhi(z_diff(1));
    S = S + weights(i)*z_diff*z_diff.transpose();
  }

  //add measurement noise covariance matrix
  
  S = S+R_las_;

  // cross correlation matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.setZero();
  for (int i=0; i<n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_.col(i) - x;

    z_diff(1) = calc.NormalizePhi(z_diff(1));
    x_diff(3) = calc.NormalizePhi(x_diff(3));
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc*S.inverse();
  VectorXd z = measurement_pack.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  z_diff(1) = calc.NormalizePhi(z_diff(1));

  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();

  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  MatrixXd Zsig = MatrixXd(n_rad_, n_sig_);
  weights = SetWeights();
  for (int i=0; i<n_sig_; ++i) {
    //extract values for readability
    double p_x  =     Xsig_(0, i);
    double p_y  =     Xsig_(1, i);
    double v    =     Xsig_(2, i);
    double yaw  =     Xsig_(3, i);
    double v_x  =     v*cos(yaw);
    double v_y  =     v*sin(yaw);

    //measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x*v_x + p_y*v_y) / Zsig(0, i);
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_rad_);
  z_pred.setZero();
  for (int i=0; i<n_sig_; ++i) {
    z_pred = z_pred + weights(i)*Zsig.col(i);
  }

  //innovation coavariance matrix S
  MatrixXd S = MatrixXd(n_rad_, n_rad_);
  S.setZero();
  for (int i=0; i<n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = calc.NormalizePhi(z_diff(1));
    S = S + weights(i)*z_diff*z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S+R_rad_;

  // cross correlation matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.setZero();
  for (int i=0; i<n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_.col(i) - x;

    z_diff(1) = calc.NormalizePhi(z_diff(1));
    x_diff(3) = calc.NormalizePhi(x_diff(3));
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc*S.inverse();
  VectorXd z = measurement_pack.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  z_diff(1) = calc.NormalizePhi(z_diff(1));

  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
}
