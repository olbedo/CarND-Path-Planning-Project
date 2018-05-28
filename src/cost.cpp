#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "cost.h"


// weights for cost functions
const double STATE = 1.0;
const double LANE = 0.1;
const double SPEED = 5.0;
const double DISTANCE = 1.0;
const double JAM = 10.;


double state_cost(const vector<map<int, predict>>& predictions, const states & state) {
  /*
  Cost function for the intended state
  */
  // No extra costs for keep lane
  if (state.name == "KL") return 0.;

  // if in state PLCL / PLCR and it is safe to change lanes, prefere lane change over
  // all other choices (negative cost)
  if (state.name == "LCL" || state.name == "LCR") return -10000.;

  // extra cost for preparation of lane change to avoid unnecessary lane changes
  return 100.;
}


double lane_cost(const vector<map<int, predict>> &predictions, const states & state) {
  /*
  Cost function for the intended lane
  How attractive is the intended lane and how far is it from the best lane
  Avoid being stuck in a semi-optimal lane
  */
  // find best lane to be in
  map<int, predict> cars_ahead = predictions[0];
  map<int, predict> cars_behind = predictions[1];

  map<int, double> lane_costs;
  double min_cost = 1e9;
  int best_lane = -1;
  for (int lane = 0; lane < LANES_AVAILABLE; lane++) {
    double cost = 0.;
    // costs for vehicles in front of the ego vehicle
    cost -= cars_ahead[lane].speed * cars_ahead[lane].speed * SPEED;
    cost -= cars_ahead[lane].distance * cars_ahead[lane].distance * DISTANCE;
    cost += cars_ahead[lane].count * JAM;
    // costs for vehicles behind
    cost += cars_behind[lane].speed * SPEED;
    cost -= cars_behind[lane].distance * DISTANCE;

    // avoid being stuck in the outer lanes
    if (lane == 0 || lane == LANES_AVAILABLE - 1) {
      cost += 10;
    }
    lane_costs[lane] = cost;
    // update min costs and best lane if necessary
    if (cost < min_cost) {
      min_cost = cost;
      best_lane = lane;
    }
  }

  // get intended lane
  int intended_lane = state.intended_lane;

  return fabs( (intended_lane - best_lane) * (lane_costs[intended_lane] - lane_costs[best_lane]) );
}


double lane_speed_cost(const vector<map<int, predict>>& predictions, const states & state) {
  /*
  Cost function for the lane speed
  The higher the better
  */
  // get intended speed for the state
  return -(state.intended_speed * state.intended_speed);
}


double vehicle_distance_cost(const vector<map<int, predict>> &predictions, const states & state) {
  /*
  Cost function for close vehicles in the inteded lane
  The farther the vehicles the lower the costs
  */
  // get predictions of close vehicles
  map<int, predict> cars_ahead = predictions[0];
  map<int, predict> cars_behind = predictions[1];

  // get intended lane
  int lane = state.intended_lane;

  // distance to vehicles in front is more important than distance to vehicles behind
  return -(cars_ahead[lane].distance * cars_ahead[lane].distance + cars_behind[lane].distance);
}


double lane_jam_cost(const vector<map<int, predict>>& predictions, const states & state) {
  /*
  Costs for traffic jam in the inteded lane
  Prefer lanes with fewer vehicles in front
  */
  // get number of vehicles in front in the intended lane
  map<int, predict> cars_ahead = predictions[0];

  return cars_ahead[state.intended_lane].count;
}


double calculate_cost(const vector<map<int, predict>> & predictions, const states & state) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  double cost = 0.0;

  //Add additional cost functions here.
  vector< function< double (const vector<map<int, predict>> & predictions, const states & state) >> 
    cf_list = { lane_speed_cost, vehicle_distance_cost , state_cost , lane_jam_cost , lane_cost };
  vector<double> weight_list = { SPEED , DISTANCE , STATE , JAM , LANE };
    
  for (int i = 0; i < cf_list.size(); i++) {
      double new_cost = weight_list[i] * cf_list[i](predictions, state);
      cost += new_cost;
  }

  return cost;
}

