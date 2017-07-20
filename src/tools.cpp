#include <iostream>
#include <iomanip>      // std::setprecision
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() == 0) return rmse;
  if (estimations.size() != ground_truth.size()) return rmse;

  for(int i=0; i < estimations.size(); ++i){
      VectorXd diff = ground_truth[i] - estimations[i];
      VectorXd diff2 = diff.array() * diff.array();
      rmse += diff2;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

void Tools::OutputPrecision(int precision) {
  cout << fixed << setprecision(precision);
}

void Tools::ShowInput(MeasurementPackage m, VectorXd &gt,
                      VectorXd e, VectorXd RMSE) {
  if (loglevels_.size() == 0) return;
  cout << m.timestamp_ << " ";
  if (m.sensor_type_ == MeasurementPackage::LASER) {
    cout << "L "
      << m.raw_measurements_(0) << " "
      << m.raw_measurements_(1) << " "
      << "0" << " ";
  } else {
    cout << "R "
      << m.raw_measurements_[0] * cos(m.raw_measurements_[1]) << " "
      << m.raw_measurements_[0] * sin(m.raw_measurements_[1]) << " "
      << m.raw_measurements_(2) << " ";
  }
  cout << gt(0) << " " << gt(1) << " " << gt(2) << " " << gt(3) << " ";
  cout << this->px << " " << this->py << " ";
  cout << e(0) << " " << e(1) << " " << e(2) << " " << e(3) << " ";
  cout << RMSE(0) << " " << RMSE(1) << " " << RMSE(2) << " " << RMSE(3) << endl;
}
