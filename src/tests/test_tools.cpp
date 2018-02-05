#ifndef M_PI
  #define M_PI 3.1415926535897;
#endif
const double M_EPS = 0.000001;
#include "catch.hpp"
#include "../tools.cpp"

TEST_CASE("RMSE of invalid length is 0", "[rmse]") {
  Tools calc;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  Eigen::VectorXd e(4);
  Eigen::VectorXd g(4);
  Eigen::VectorXd z(4);
  z.setZero();

  SECTION("RMSE of empty vector is 0") {
    REQUIRE(calc.CalculateRMSE(estimations, ground_truth) == z);
  }
  SECTION("RMSE of vectors of different length is 0") {
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(g);
    g << 1.1, 1.1, 0.3, 0.2;
    REQUIRE(calc.CalculateRMSE(estimations, ground_truth) == z);
  }
}

TEST_CASE("RMSE of two vectors calculated", "[rmse]") {
  Tools calc;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  Eigen::VectorXd e(4);
  Eigen::VectorXd g(4);
  Eigen::VectorXd z(4);
  z.setZero();
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, 
                               ", ", ", ", "", "", " << ", ";");
  SECTION("RMSE of equal vectors is zero") {
    e << 1, 1, 1, 1;
    estimations.push_back(e);
    g << 1, 1, 1, 1;
    ground_truth.push_back(g);
    REQUIRE(calc.CalculateRMSE(estimations, ground_truth) == z);
  }

  SECTION("RMSE calculated") {
    Eigen::VectorXd res(4);
    Approx target = Approx(0.1).epsilon(0.01);
    
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    res = calc.CalculateRMSE(estimations, ground_truth);
    for(int i = 0; i < res.size(); ++i) {
      REQUIRE(res[i] == target);
    }
  }
}

/*
TEST_CASE("Normalize phi calculated" "[normalize-phi]") {
  Tools calc;
  
  REQUIRE(calc.NormalizePhi(0.25*M_PI) == Approx(0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-0.25*M_PI) == Approx(-0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(0.75*M_PI) == Approx(0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-0.75*M_PI) == Approx(-0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(1.25*M_PI) == Approx(-0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-1.25*M_PI) == Approx(0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(1.75*M_PI) == Approx(-0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-1.75*M_PI) == Approx(0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(2.25*M_PI) == Approx(0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-2.25*M_PI) == Approx(-0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(2.75*M_PI) == Approx(0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-2.75*M_PI) == Approx(-0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(3.25*M_PI) == Approx(-0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-3.25*M_PI) == Approx(0.75*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(3.75*M_PI) == Approx(-0.25*M_PI).epsilon(M_EPS));
  REQUIRE(calc.NormalizePhi(-3.75*M_PI) == Approx(0.25*M_PI).epsilon(M_EPS));
}

TEST_CASE("Convert cartesian to polar" "[cartesian-polar]") {
  Tools calc;
  Eigen::VectorXd cart(4), polar(3), calc_polar;
  double sq_2 = std::pow(2, 0.5);
  
  cart << 1, 0, 0, 0;  
  polar << 1, 0, 0;
  calc_polar = calc.CartesianToPolar(cart);
  REQUIRE(calc_polar(0) == Approx(polar(0)).epsilon(M_EPS));
  REQUIRE(calc_polar(1) == Approx(polar(1)).epsilon(M_EPS));
  REQUIRE(calc_polar(2) == Approx(polar(2)).epsilon(M_EPS));

  cart << 10, 10, 0, 0;  
  polar << 10*sq_2, 0.25*M_PI, 0;
  calc_polar = calc.CartesianToPolar(cart);
  REQUIRE(calc_polar(0) == Approx(polar(0)).epsilon(M_EPS));
  REQUIRE(calc_polar(1) == Approx(polar(1)).epsilon(M_EPS));
  REQUIRE(calc_polar(2) == Approx(polar(2)).epsilon(M_EPS));
  
  cart << 1, 0, 1, 0;  
  polar << 1, 0, 1;
  calc_polar = calc.CartesianToPolar(cart);
  REQUIRE(calc_polar(0) == Approx(polar(0)).epsilon(M_EPS));
  REQUIRE(calc_polar(1) == Approx(polar(1)).epsilon(M_EPS));
  REQUIRE(calc_polar(2) == Approx(polar(2)).epsilon(M_EPS));
  
  cart << 1, 0, 0, 1;  
  polar << 1, 0, 0;
  calc_polar = calc.CartesianToPolar(cart);
  REQUIRE(calc_polar(0) == Approx(polar(0)).epsilon(M_EPS));
  REQUIRE(calc_polar(1) == Approx(polar(1)).epsilon(M_EPS));
  REQUIRE(calc_polar(2) == Approx(polar(2)).epsilon(M_EPS));
  
  cart << 1, 1, 1, 1;  
  polar << sq_2, 0.25*M_PI, sq_2;
  calc_polar = calc.CartesianToPolar(cart);
  REQUIRE(calc_polar(0) == Approx(polar(0)).epsilon(M_EPS));
  REQUIRE(calc_polar(1) == Approx(polar(1)).epsilon(M_EPS));
  REQUIRE(calc_polar(2) == Approx(polar(2)).epsilon(M_EPS));
} 

TEST_CASE("Targets too close return 0" "[cartesian-polar]") {
  Tools calc;
  Eigen::VectorXd cart(4), polar(3);
  cart << 0, 0, 0, 0;
  polar << 0, 0, 0;
  REQUIRE(calc.CartesianToPolar(cart) == polar);  
  cart << 0.01, 0.01, 10, 10;
  REQUIRE(calc.CartesianToPolar(cart) == polar);
  cart << 0, 0.0319, 10, 10;
  REQUIRE(calc.CartesianToPolar(cart) == polar);
  cart << 0.0319, 0, 10, 10;
  REQUIRE(calc.CartesianToPolar(cart) == polar);
} 
*/
