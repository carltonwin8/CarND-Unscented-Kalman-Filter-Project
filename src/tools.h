#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <set>
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  ///* Logleve
  set<int> loglevels_;

  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth);
  /**
  * A helper added by Carlton.
  */
  void OutputPrecision(int precision);
  double px, py;
  void ShowInput(MeasurementPackage m, VectorXd &gt,
    VectorXd estimate, VectorXd RMSE);

};

#endif /* TOOLS_H_ */
