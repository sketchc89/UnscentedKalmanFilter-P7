#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  int est_sz = estimations.size();
  int grt_sz = ground_truth.size();

  VectorXd rmse(4);
  rmse.setZero();

  if (est_sz == 0 || est_sz != grt_sz) {
    return rmse;
  }
  

  for (int i = 0; i < est_sz; ++i) {
    VectorXd accum = estimations[i] - ground_truth[i];
    accum = accum.array()*accum.array();
    rmse += accum;
  }
  rmse /= est_sz;
  return rmse.array().sqrt();
}

double Tools::NormalizePhi(double phi) {
    return phi - 2*M_PI*std::floor((phi + M_PI) / (2*M_PI));
}
