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
  P_ = MatrixXd(5, 5);

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
  lambda_ = 3 - n_aug; 

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
  if (!is_initialized){
    auto console = spdlog::stdout_logger_mt("console");
    console->info("First measurement");

    time_us_ = measurement_pack.timestamp_;
    console->info("Time: {}", time_us_);
    is_initialized = true;
    time_us_ = measurement_pack.timestamp_;
    P_ << 1,  0,  0,  0,
          0,  1,  0,  0,
          0,  0,  1000, 0,
          0,  0,  0,  1000;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      console->debug("Initializing with RADAR");
      double rho = measurement_pack.raw_measurements_(0);
      double phi = calc.NormalizePhi(measurement_pack.raw_measurements_(1));
      double rho_dot = measurment_pack.raw_measurements_(2);
      
        x_ << rho*cos(phi),
            rho*sin(phi),
            0,
            0,
            0;
    }
    if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR) {
      x_ << measurement_pack.raw_measurements_(0),
            measurement.pack.raw_measurements_(1),
            0,
            0,
            0;
    }

    return;
  }
  double dt;
  dt  = (measurement_pack.timestamp_ - times_us_) / 1000000.0;
  Prediction();
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
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
