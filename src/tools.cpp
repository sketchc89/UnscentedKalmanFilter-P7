#include <iostream>
#include "tools.h"
#include "spdlog/spdlog.h"

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


void Tools::PrintMatrix(std::string name, const MatrixXd &m) {
  auto console = spdlog::get("console");
  console->info("Printing  {}x{} matrix {}", m.rows(), m.cols(), name);
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) {
      console->info("\t{}\t{},{}\t{}", name, i, j, m(i, j));
    }
  }
  console->info("----------------------------");
}

void Tools::PrintVector(std::string name, const VectorXd &v) {
  auto console = spdlog::get("console");
  console->info("Printing vector {}", name);
  for (int i = 0; i < v.size(); ++i) {
    console->info("\t{}\t{}\t{}", name, i, v(i));
  }
  console->info("----------------------------");
}
