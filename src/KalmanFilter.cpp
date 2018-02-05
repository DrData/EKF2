#include "KalmanFilter.h"
#include "KalmanFilter.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

KalmanFilter::KalmanFilter(StateVector *pSV) : is_initialized_(false), noise_ax(9.0), noise_ay(9.0),
I_(MatrixXd::Identity(pSV->x.size(), pSV->x.size()))
{
}

void KalmanFilter::ProcessMeasurement(const MeasurementPackage & measurement_pack)
{
	//compute the time elapsed between the current and previous measurements
	float dt = (float)((measurement_pack.timestamp_ - _pSV->previous_timestamp_) / 1000000.0);	//dt - expressed in seconds
	_pSV->previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// Modify the F matrix so that the time is integrated
	F_(0, 2) = dt;
	F_(1, 3) = dt;

	// Set the process covariance matrix Q
	Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

}

// Use default prediction
void KalmanFilter::Predict()
{
	*x_ = F_ * *x_;
	*P_ = F_ * *P_ * F_.transpose() + Q_;
}

// Use default prediction
void KalmanFilter::Update(const Eigen::VectorXd &z)
{
	VectorXd z_pred = H_ * *x_;  // (3,4) (4) -> (3)

	VectorXd y = z - z_pred; //(3) - (3)

	MatrixXd S = H_ * *P_ * Ht_ + R_;  // (3,4) (4,4), (4,3) + (3,3)
	MatrixXd Si = S.inverse(); //(3,3)
	MatrixXd PHt = *P_ * Ht_; // (4,4) (4,3) => (4,3)
	MatrixXd K = PHt * Si; // (4,3) (3,3) => (4,3)

						   //new estimate
	*x_ = *x_ + (K * y); // (4) + (4,3)(3)
	*P_ = (I_ - K * H_) * *P_; //( (4,4) - (4,3) (3,4) )  (4,4)
}


LaserKalmanFilter::LaserKalmanFilter(StateVector * pSV) : KalmanFilter(pSV)
{
	// Construct and Initialize the filter
	// for laser sensor
	// Store state vector
	_pSV = pSV;

	// Setup State Vector
	x_ = &(pSV->x);

	//state covariance matrix P
	P_ = &(pSV->P);

	int ndim = pSV->x.size();
	int npos = ndim / 2;


	// Measurement covariance
	R_ = MatrixXd(npos, npos);
	VectorXd laserDiag = VectorXd::Constant(npos,0.0225);
	R_ = laserDiag.asDiagonal();

	// Motion matrix
	H_ = MatrixXd(npos, ndim);
	H_.fill(0);
	for (int i = 0; i < npos; i++)
	{
		H_(i, i) = 1.0;
	}

	// Save transpose
	Ht_ = H_.transpose();

	// the initial transition matrix F_
	// Will incorporate time later
	F_ = MatrixXd::Identity(ndim, ndim);
	for (int i = 0; i < npos; i++)
	{
		F_(i, i + npos) = 1.0;
	}

	//the initial transition matrix Q_
	Q_ = MatrixXd::Identity(ndim, ndim);
}

LaserKalmanFilter::~LaserKalmanFilter()
{
}

void LaserKalmanFilter::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
	if (!_pSV->is_initialized_) {
		//cout << "Laser Kalman Filter Initialization " << endl;
		//set the state with the initial location and zero velocity
		*x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

		_pSV->previous_timestamp_ = measurement_pack.timestamp_;
		_pSV->is_initialized_ = true;
		return;
	}

	// Do the standard processing to prepare for predict and update
	KalmanFilter::ProcessMeasurement(measurement_pack);

	// Predict
	Predict();

	// Measurement update
	Update(measurement_pack.raw_measurements_);

}

RadarKalmanFilter::RadarKalmanFilter(StateVector *pSV) : KalmanFilter(pSV)
{
	// Store the state vector of car
	_pSV = pSV;

	// Setup State Vector in the Kalman
	x_ = &(pSV->x);

	//state covariance matrix P
	P_ = &(pSV->P);

	int ndim = pSV->x.size();

	// Assumes the position vector is in the first half of vector
	int npos = ndim / 2;

	// Dimension of Radar variables
	int ndf = 3;

	// Initialize Parameters
	// Measurement covariance
	R_ = MatrixXd(ndf, ndf);
	R_.fill(0);
	VectorXd diag = VectorXd(ndf);
	diag << 0.09, 0.0009, 0.09;
	R_ = diag.asDiagonal();

	// Measurement matrix
	H_ = MatrixXd(ndf, ndim);
	H_.fill(0);
	for (int i = 0; i < ndf; i++)
	{
		H_(i, i) = 1.0;
	}
	Ht_ = H_.transpose();

	// The initial transition matrix F_
	F_ = MatrixXd::Identity(ndim, ndim);
	for (int i = 0; i < npos; i++)
	{
		F_(i, i + npos) = 1.0;
	}

	// The initial convariance noise matrix Q_
	Q_ = MatrixXd::Identity(ndim, ndim);
}

RadarKalmanFilter::~RadarKalmanFilter()
{
}

void RadarKalmanFilter::Update(const Eigen::VectorXd &z)
{
	VectorXd &xstate = *x_;
	MatrixXd &Pstate = *P_;

	// Compute h(x')
	// Convert to polar coordinates
	double px = xstate(0);
	double py = xstate(1);
	double vx = xstate(2);
	double vy = xstate(3);

	double r = sqrt(px*px + py * py);
	if (r > 0.0001)
	{
		double phi = atan2(py, px);
		double phodot = (px*vx + py * vy) / r;

		VectorXd z_pred(3);
		z_pred << r, phi, phodot;

		VectorXd y = z - z_pred; //(3) - (3)

		// Normalize angle
		while (y(1)> M_PI) y(1) -= 2.*M_PI;
		while (y(1)<-M_PI) y(1) += 2.*M_PI;

		// Radar Use EKF. Get Jacobian
		if (!Tools::CalculateJacobian(xstate, H_))
		{
			cout << "Invalid Jacobian " << endl;
			return; // don't update
		}
		Ht_ = H_.transpose();

		MatrixXd S = H_ * Pstate * Ht_ + R_;  // (3,4) (4,4), (4,3) + (3,3)
		MatrixXd Si = S.inverse(); //(3,3)
		MatrixXd PHt = Pstate * Ht_; // (4,4) (4,3) => (4,3)
		MatrixXd K = PHt * Si; // (4,3) (3,3) => (4,3)

		//new estimate
		xstate = xstate + (K * y); // (4) + (4,3)(3)
		Pstate = (I_ - K * H_) * Pstate; //( (4,4) - (4,3) (3,4) )  (4,4)
	}
}


void RadarKalmanFilter::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
	if (!_pSV->is_initialized_) {
		//cout << "Kalman Filter Initialization " << endl;
		//set the state with the initial location and zero velocity
		// Convert the first measurement data to initialize the state vector
		// Must convert from polar to Cartesian coordinates
		double r = measurement_pack.raw_measurements_(0);
		double phi = measurement_pack.raw_measurements_(1);
		double rdot = measurement_pack.raw_measurements_(2);

		// Populate state vector
		*x_ << r * cos(phi), r*sin(phi), rdot*cos(phi), rdot*sin(phi);

		// Update timestamp
		_pSV->previous_timestamp_ = measurement_pack.timestamp_;
		_pSV->is_initialized_ = true;
		return;
	}

	// Prep for predict and update
	KalmanFilter::ProcessMeasurement(measurement_pack);

	// Predict
	Predict();

	// Measurement update
	Update(measurement_pack.raw_measurements_);
}



KalmanFilter* KalmanFilterFactory::MakeKalmanFilter(SensorType type)
{
	KalmanFilter * kfs=NULL;
	switch (type)
	{
	case eLaser:
		kfs = new LaserKalmanFilter(_pSV);
		break;
	case eRadar:
		kfs = new RadarKalmanFilter(_pSV);
		break;
	}
	return kfs;
}

