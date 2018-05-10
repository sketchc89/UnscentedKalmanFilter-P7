#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Number of sigma points
  int n_sig_;

  //Measurement process dimension
  int n_rad_;
  int n_las_;

  //Process noise matrix
  MatrixXd Q_;
  //Measurement noise matrix
  MatrixXd R_las_;
  MatrixXd R_rad_;

  ///* Sigma point spreading parameter
  double lambda_;

  Tools calc;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param measurement_pack The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage measurement_pack);

  /*
   * Initializes state from first measurement
   * @param measurement_pack The latest measurement data of either radar or laser
   */
  void InitializeState(MeasurementPackage measurement_pack);

  /*
   * Return x_ vector with additional rows
   */
  VectorXd AugmentX();

  /*
   * Return P_ vector with process noise
   */
  MatrixXd AugmentP();

  /*
   * Generate sigma points for prediction
   */
  MatrixXd GenerateSigmaPoints(VectorXd x_aug, MatrixXd P_aug);

  /*
   * Returns a vector of weights for use with sigma points
   */
  //VectorXd SetWeights();

  /**
   * Predicts sigma points given the state and time since the last measurement
   * @param {double} delta_t the change in time (in seconds) between the last
   * measurement and this one
   */
  void PredictSigmaPoints(MatrixXd Xsig_aug, double delta_t);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param measurement_pack The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage measurement_pack);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param measurement_pack The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage measurement_pack);
};

#endif /* UKF_H */
