#ifndef COST_H
#define COST_H
#include "helper_functions.h"

using namespace std;

// Cost function for the intended state
double state_cost(const vector<map<int, predict>> &predictions, const states & state);

// Cost function for the intended lane
double lane_cost(const vector<map<int, predict>> &predictions, const states & state);

// Cost function for the lane speed
double lane_speed_cost(const vector<map<int, predict>> &predictions, const states & state);

// Cost function for close vehicles in the inteded lane
double vehicle_distance_cost(const vector<map<int, predict>> &predictions, const states & state);

// Costs for traffic jam in the inteded lane
double lane_jam_cost(const vector<map<int, predict>>& predictions, const states & state);

// Calculation of total costs
double calculate_cost(const vector<map<int, predict>> &predictions, const states & state);

#endif