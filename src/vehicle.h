#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
//#include "helper_functions.h"
#include "cost.h"

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = { { "PLCL", -1 },{ "LCL", -1 },{ "LCR", 1 },{ "PLCR", 1 } };

  // calculate current lane number (last waypoint of the residual
  // of the previous path
  int lane;

  // target lane number
  int target_lane;

  // flag to indicate lane change
  bool change_lane = false;

  // number of waypoints in trajectory
  int num_wpts = 30;

  // safety clearance to vehicles in front and behind
  double safety_clearance = 20.; 

  // horizon for prediction (max. distance to be considered)
  double horizon = 100.;

  // vehicles localization data
  // vehicles x-position
  double x;
  // vehicles y-position
  double y;

  // vehicles previous x-position in global c/s
  double prev_x;
  // vehicles previous y-position in global c/s
  double prev_y;

  // vehicles previous x-position in vehicle c/s
  double prev_x_car;
  // vehicles previous y-position in vehicle c/s
  double prev_y_car;

  // Frenet coordinates
  // longitudinal distance
  double s;
  // transversal distance
  double d;

  // yaw angle
  double yaw;
  double sin_yaw;
  double cos_yaw;

  // vehicles velocity
  double v;

  // vehicles last velocity
  double prev_speed = 0;

  // vehicles target velocity
  double target_speed = MAX_SPEED;

  // vehicles acceleration
  double a;

  // number of remaining waypoints from previous path
  int residual_path_size;

  // x-values of remaining waypoints from previous path
  vector<double> residual_path_x;
  // y-values of remaining waypoints from previous path
  vector<double> residual_path_y;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> &maps_s;
  vector<double> &maps_x;
  vector<double> &maps_y;
  vector<double> &maps_dx;
  vector<double> &maps_dy;

  // data from sensor fusion
  vector<vector<double>> sensor_fusion;

  // current state in finite state machine
  string state = "KL";
  
  /**
  * Constructor
  */
  Vehicle(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y,
          vector<double> &maps_dx, vector<double> &maps_dy);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<vector<double>> choose_next_state();

  vector<string> successor_states();

  states generate_trajectory(string &state_name, const vector<map<int, predict>> &predictions);

  vector<vector<double>> generate_waypoints(int lane);

  double get_lane_speed(int lane, double ref_speed, const vector<map<int, predict>> &predictions);

  double get_lane_speed_behind(int lane, const vector<map<int, predict>> &predictions);

  states keep_lane_trajectory(const vector<map<int, predict>> &predictions);
  
  states prep_lane_change_trajectory(string &state_name, const vector<map<int, predict>> &predictions);

  states lane_change_trajectory(string &state_name, const vector<map<int, predict>> &predictions);

  vector<map<int, predict>> generate_predictions(double s, int timesteps);

  bool residual_feasible(double s, double d, double end_path_s);

  void update(double x, double y, double s, double d, double yaw, double v,
              vector<double> residual_path_x, vector<double> residual_path_y,
              double end_path_s, double end_path_d, 
              vector<vector<double>> sensor_fusion);
};

#endif