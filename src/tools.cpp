#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
	VectorXd rsme(4);
	rsme.fill(0);

	long n = estimations.size();
	if ( n < 3 || n != ground_truth.size())
	{
		return rsme;
	}

	for (int i = 0; i < n; i++)
	{
		VectorXd xest = estimations[i];
		VectorXd xlabel = ground_truth[i];

		VectorXd dif = xest - xlabel;
		dif = dif.array() * dif.array();

		rsme += dif;
	}

	// Get average of squared differences
	rsme /= (double) n;

	rsme = rsme.array().sqrt();
	return rsme;
}

bool Tools::CalculateJacobian(const VectorXd& x_state, MatrixXd& Hj)
{
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	double c1 = px * px + py * py;
	double c2 = sqrt(c1);
	double c3 = (c1*c2);

	//check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;

		Hj = MatrixXd::Identity(3, 4);
		double signx = px < 0.0 ? -1 : 1;
		double signy = py < 0.0 ? -1 : 1;
		double signx2 = py * (vx*py - vy * px) < 0.0 ? -1 : 1;
		double signy2 = px * (px*vy - py * vx)  < 0.0 ? -1 : 1;
		Hj << (px / c2), (py / c2), 0, 0,
			-signy* numeric_limits<double>::max(), signx * numeric_limits<double>::min(), 0, 0,
			signx2*numeric_limits<double>::max(), 
			signy2*numeric_limits<double>::max(), 
			signx * numeric_limits<double>::min(), 
			signy* numeric_limits<double>::max();
		return true;
	}

	//compute the Jacobian matrix (3,4)
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy * px) / c3, px*(px*vy - py * vx) / c3, px / c2, py / c2;

	return true;
}
