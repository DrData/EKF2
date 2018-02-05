#ifndef KTSensor_H_
#define KTSensor_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

struct StateVector
{
	StateVector(int ndim) : is_initialized_(false), previous_timestamp_(0)
	{
		x = VectorXd(ndim);
		x.fill(0);

		P = MatrixXd::Identity(ndim, ndim);
	}
	
	VectorXd x;
	MatrixXd P;
	long size() const { return x.size(); }

	bool is_initialized_;
	long long previous_timestamp_;
};


// Base class for Kalman Filter 
class KalmanFilter
{
public:
	KalmanFilter(StateVector *pSV);
	
	virtual ~KalmanFilter() {};

	virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	virtual void Update(const Eigen::VectorXd &z);
	virtual void Predict();


protected:
	KalmanFilter() {};

	// state vector
	Eigen::VectorXd *x_;

	// state covariance matrix
	Eigen::MatrixXd *P_;

	// state transition matrix
	Eigen::MatrixXd F_;

	// process covariance matrix
	Eigen::MatrixXd Q_;

	// measurement matrix
	Eigen::MatrixXd H_;

	// measurement covariance matrix
	Eigen::MatrixXd R_;

	// KalmanFilter
	//KalmanFilter kf_;

	// previous timestamp
	long long previous_timestamp_;
	
	// Flag to user first measurement to initialilze state.
	bool is_initialized_;

	// Contains the state vector of the car
	StateVector* _pSV;

	// Noise in process/measurement
	double noise_ax;
	double noise_ay;

	MatrixXd I_;
	MatrixXd Ht_; // Transpose of H

};

// Define sensor type enumeration
enum SensorType
{
	eLaser,
	eRadar,
};

// Laser Sensor Kalman Filter
// Handles laser measurement related parameters
class LaserKalmanFilter : public KalmanFilter
{
public:
	LaserKalmanFilter(StateVector* pSV);

	virtual ~LaserKalmanFilter();

	virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack);

protected:
	LaserKalmanFilter() {};
private:

};

// Radar Sensor is derived from KalmanFilter
// Handles Radar measurement related parameters
class RadarKalmanFilter : public KalmanFilter
{
public:
	RadarKalmanFilter(StateVector* pSV);

	virtual ~RadarKalmanFilter();

	virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	virtual void Update(const Eigen::VectorXd &z);

protected:
	RadarKalmanFilter() {};

};

// Creates the requested KTSensor class (laser or radar)
class KalmanFilterFactory
{
public:
	KalmanFilterFactory(StateVector *pSV) 
	{
		_pSV = pSV;
	};

	KalmanFilter * MakeKalmanFilter(SensorType type);

private:
	StateVector *_pSV;
};

#endif 
