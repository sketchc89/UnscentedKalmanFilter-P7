#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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

  // TODO: Design choice: Spreading factor
  lambda_ = 3 - n_aug_; 

  //auto console_ = spdlog::stdout_logger_mt("console");
  // Initial predicted sigma points
  Xsig_pred_ = MatrixXd(n_aug_, 2*n_aug_ + 1);
  Xsig_pred_.setZero();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} measurement_pack The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  time_us_ = measurement_pack.timestamp_;
  if (!is_initialized_){
    //console->info("First measurement");
    InitializeState(measurement_pack);
    InitializeSigmaPoints();
    is_initialized_ = true;
    return;
  }
  
  double dt;
  dt  = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
  Prediction(dt);
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //console->debug("Updating RADAR");
    //UpdateRadar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    //console->debug("Updating LIDAR");
    //UpdateLidar(measurement_pack);
  }
}

/*
 * Initializes state and sigma points from first measurement 
 */
void UKF::InitializeState(MeasurementPackage measurement_pack) {
  
  time_us_ = measurement_pack.timestamp_;
  //console->info("Time: {}", time_us_);
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

  //console->debug("Initializing sigma points");
  //Calculate sqrt of P_
  MatrixXd A = P_.llt().matrixL();
  //First sigma point is mean
  Xsig_pred_.col(0) = x_;
  //Calculate remaining sigma points from mean and spreading parameter 
  for (int i=0; i<n_x_; ++i) {
    Xsig_pred_.col(i+1)       = x_ + sqrt(lambda_+n_x_) + A.col(i);
    Xsig_pred_.col(i+1+n_x_)  = x_ - sqrt(lambda_+n_x_) + A.col(i);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  PredictSigmaPoints(delta_t);
  VectorXd weights = VectorXd(2*n_aug_ + 1);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; ++i) {
    weights(i) = 0.5/(n_aug_+lambda);
  }
  
  VectorXd x_pred = VectorXd(2*n_aug_ + 1);
  VectorXd P_pred = VectorXd(2*n_aug_ + 1);
  x_pred.fill(0.0);
  P_pred.fill(0.0);

  //predicted state mean
  for (int i=0; i<2*n_aug_+1; ++i) {
   x_pred = x_pred + weights(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  for (int i=0; i< 2*n_aug_+1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
    x_diff(3) = calc.NormalizePhi(x_diff(3));
    P_pred = P_pred + weights(i) * x_diff * x_diff.transpose();
  }

  x_ = x_pred;
  P_ = P_pred;
}

/**
 * Predicts sigma points given the state and time since the last measurement
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::PredictSigmaPoints(double delta_t) {
  for (int i=0; i<2*n_aug_+1; ++i) {
    double p_x = x_(
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
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
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
