#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "KalmanFilter.h"
#include "tools.h"
#include <string>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

const static int N_STATE_VECTOR_DIMENSION = 4;

void ProcessToFile();

int main()
{
   uWS::Hub h;

  bool bFromFile = false;
  if (bFromFile)
  {
	  ProcessToFile();
  }
  else
  {

	  Tools tools;
	  vector<VectorXd> estimations;
	  vector<VectorXd> ground_truth;

	  map<SensorType, KalmanFilter *> mapSensorsType2KF;

	  StateVector theState(N_STATE_VECTOR_DIMENSION);
	  StateVector *pTheState = &theState;

	  KalmanFilterFactory KTsensorFactory(pTheState);
	  mapSensorsType2KF.insert(pair<SensorType, KalmanFilter *>(eLaser, KTsensorFactory.MakeKalmanFilter(eLaser)));
	  mapSensorsType2KF.insert(pair<SensorType, KalmanFilter *>(eRadar, KTsensorFactory.MakeKalmanFilter(eRadar)));

	  h.onMessage([&theState, &tools, &estimations, &ground_truth, &mapSensorsType2KF](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		  // "42" at the start of the message means there's a websocket message event.
		  // The 4 signifies a websocket message
		  // The 2 signifies a websocket event

		  if (length && length > 2 && data[0] == '4' && data[1] == '2')
		  {

			  auto s = hasData(std::string(data));
			  if (s != "") {

				  auto j = json::parse(s);

				  std::string event = j[0].get<std::string>();

				  if (event == "telemetry") {
					  // j[1] is the data JSON object

					  string sensor_measurment = j[1]["sensor_measurement"];

					  MeasurementPackage meas_package;
					  istringstream iss(sensor_measurment);
					  long long timestamp;

					  // reads first element from the current line
					  string sensor_type;
					  iss >> sensor_type;

					  SensorType etype;
					  if (sensor_type.compare("L") == 0) {
						  etype = eLaser;
						  meas_package.sensor_type_ = MeasurementPackage::LASER;
						  meas_package.raw_measurements_ = VectorXd(2);
						  float px;
						  float py;
						  iss >> px;
						  iss >> py;
						  meas_package.raw_measurements_ << px, py;
						  iss >> timestamp;
						  meas_package.timestamp_ = timestamp;
					  }
					  else if (sensor_type.compare("R") == 0) {
						  etype = eRadar;

						  meas_package.sensor_type_ = MeasurementPackage::RADAR;
						  meas_package.raw_measurements_ = VectorXd(3);
						  float ro;
						  float theta;
						  float ro_dot;
						  iss >> ro;
						  iss >> theta;
						  iss >> ro_dot;
						  meas_package.raw_measurements_ << ro, theta, ro_dot;
						  iss >> timestamp;
						  meas_package.timestamp_ = timestamp;
					  }

					  // Get Ground Truth
					  float x_gt;
					  float y_gt;
					  float vx_gt;
					  float vy_gt;
					  iss >> x_gt;
					  iss >> y_gt;
					  iss >> vx_gt;
					  iss >> vy_gt;
					  VectorXd gt_values(4);
					  gt_values(0) = x_gt;
					  gt_values(1) = y_gt;
					  gt_values(2) = vx_gt;
					  gt_values(3) = vy_gt;
					  ground_truth.push_back(gt_values);

					  //Call ProcessMeasurment(meas_package) for Kalman filter
					  mapSensorsType2KF[etype]->ProcessMeasurement(meas_package);

					  //Push the current estimated x,y positon from the Kalman filter's state vector
					  VectorXd estimate(4);

					  double p_x = theState.x(0);
					  double p_y = theState.x(1);
					  double v1 = theState.x(2);
					  double v2 = theState.x(3);

					  estimate(0) = p_x;
					  estimate(1) = p_y;
					  estimate(2) = v1;
					  estimate(3) = v2;

					  estimations.push_back(estimate);

					  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

					  json msgJson;
					  msgJson["estimate_x"] = p_x;
					  msgJson["estimate_y"] = p_y;
					  msgJson["rmse_x"] = RMSE(0);
					  msgJson["rmse_y"] = RMSE(1);
					  msgJson["rmse_vx"] = RMSE(2);
					  msgJson["rmse_vy"] = RMSE(3);
					  //cout << "et: " << p_x << ", " << p_y << ", " << v1 << ", " << v2 << endl;
					  //cout << "gt: " << x_gt << ", " << y_gt << ", " << vx_gt << ", " << vy_gt << ", " << endl;
					  //cout << "rmse: " << RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << endl;

					  auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
					  // std::cout << msg << std::endl;
					  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				  }
			  }
			  else {

				  std::string msg = "42[\"manual\",{}]";
				  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			  }
		  }

	  });

	  // We don't need this since we're not using HTTP but if it's removed the program
	  // doesn't compile :-(
	  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		  const std::string s = "<h1>Hello world!</h1>";
		  if (req.getUrl().valueLength == 1)
		  {
			  res->end(s.data(), s.length());
		  }
		  else
		  {
			  // i guess this should be done more gracefully?
			  res->end(nullptr, 0);
		  }
	  });

	  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		  std::cout << "Connected!!!" << std::endl;
	  });

	  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		  ws.close();
		  std::cout << "Disconnected" << std::endl;
	  });

	  int port = 4567;
	  //if (h.listen(port))
	  if (h.listen("127.0.0.1", port))
	  {
		  std::cout << "Listening to port " << port << std::endl;
	  }
	  else
	  {
		  std::cerr << "Failed to listen to port" << std::endl;
		  return -1;
	  }
	  h.run();
  }
}


void ProcessToFile()
{
	Tools tools;
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	map<SensorType, KalmanFilter *> mapSensorsType2KF;

	StateVector theState(N_STATE_VECTOR_DIMENSION);
	StateVector *pTheState = &theState;

	KalmanFilterFactory KTsensorFactory(pTheState);
	mapSensorsType2KF.insert(pair<SensorType, KalmanFilter *>(eLaser, KTsensorFactory.MakeKalmanFilter(eLaser)));
	mapSensorsType2KF.insert(pair<SensorType, KalmanFilter *>(eRadar, KTsensorFactory.MakeKalmanFilter(eRadar)));

	MeasurementPackage meas_package;

	std::string fnameData = "D:\\Google\\GDrive443\\SDCT2\\EKF\\data\\obj_pose-laser-radar-synthetic-input.txt";
	std::string foutput = "D:\\Google\\GDrive443\\SDCT2\\EKF\\rsme_set1.txt";
	ofstream sOutput;
	ifstream sensorDataFile;

	sOutput.open(foutput);
	sensorDataFile.open(fnameData);
	string sMeasure;
	if (sensorDataFile.is_open()) {
		while (std::getline(sensorDataFile, sMeasure))
		{
			istringstream iss(sMeasure);
			long long timestamp;

			// reads first element from the current line
			string sensor_type;
			iss >> sensor_type;

			SensorType etype;
			if (sensor_type.compare("L") == 0) {
				etype = eLaser;
				meas_package.sensor_type_ = MeasurementPackage::LASER;
				meas_package.raw_measurements_ = VectorXd(2);
				float px;
				float py;
				iss >> px;
				iss >> py;
				meas_package.raw_measurements_ << px, py;
				iss >> timestamp;
				meas_package.timestamp_ = timestamp;
			}
			else if (sensor_type.compare("R") == 0) {
				etype = eRadar;

				meas_package.sensor_type_ = MeasurementPackage::RADAR;
				meas_package.raw_measurements_ = VectorXd(3);
				float ro;
				float theta;
				float ro_dot;
				iss >> ro;
				iss >> theta;
				iss >> ro_dot;
				meas_package.raw_measurements_ << ro, theta, ro_dot;
				iss >> timestamp;
				meas_package.timestamp_ = timestamp;
			}

			// Get Ground Truth
			float x_gt;
			float y_gt;
			float vx_gt;
			float vy_gt;
			iss >> x_gt;
			iss >> y_gt;
			iss >> vx_gt;
			iss >> vy_gt;
			VectorXd gt_values(4);
			gt_values(0) = x_gt;
			gt_values(1) = y_gt;
			gt_values(2) = vx_gt;
			gt_values(3) = vy_gt;
			ground_truth.push_back(gt_values);

			//Call ProcessMeasurment(meas_package) for Kalman filter
			mapSensorsType2KF[etype]->ProcessMeasurement(meas_package);

			//Push the current estimated x,y positon from the Kalman filter's state vector
			VectorXd estimate(4);

			double p_x = theState.x(0);
			double p_y = theState.x(1);
			double v1 = theState.x(2);
			double v2 = theState.x(3);

			estimate(0) = p_x;
			estimate(1) = p_y;
			estimate(2) = v1;
			estimate(3) = v2;

			estimations.push_back(estimate);

			VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

			json msgJson;
			msgJson["estimate_x"] = p_x;
			msgJson["estimate_y"] = p_y;
			msgJson["rmse_x"] = RMSE(0);
			msgJson["rmse_y"] = RMSE(1);
			msgJson["rmse_vx"] = RMSE(2);
			msgJson["rmse_vy"] = RMSE(3);
			//sOutput << "et: " << p_x << ", " << p_y << ", " << v1 << ", " << v2 << endl;
			//sOutput << "gt: " << x_gt << ", " << y_gt << ", " << vx_gt << ", " << vy_gt << ", " << endl;
			sOutput << "rmse: " << RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << endl;

			long long nest = estimations.size();
		}
	}

}
