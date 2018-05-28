#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"


/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y,
                 vector<double> &maps_dx, vector<double> &maps_dy) : maps_s(maps_s),
                 maps_x(maps_x), maps_y(maps_y), maps_dx(maps_dx), maps_dy(maps_dy) {};

Vehicle::~Vehicle() {}


vector<vector<double>> Vehicle::choose_next_state() {
  /*
  Chose next state, calculate and return its trajectory
  */
  // get predictions for near vehicles from sensor fusion data
  vector<map<int, predict>> predictions = this->generate_predictions(this->s, this->residual_path_size);

  // get states which can be reached from current FSM state.
  vector<string> possible_successor_states = this->successor_states();

  vector<double> costs;
  vector<states> feasible_states;

	for (int i = 0; i < possible_successor_states.size(); i++) {
    // generate a rough idea of what trajectory we would
    // follow IF we chose this state
    string state_name = possible_successor_states[i];
    states state = this->generate_trajectory(state_name, predictions);
    // consider feasible states only
    if (state.feasible) {
      feasible_states.push_back(state);
      // calculate the "cost" associated with that state
      double cost = calculate_cost(predictions, state);
      costs.push_back(cost);
    }
  }
    
  // find the best state (with minimum cost)
  int best_id = 0;
  double min_cost = costs[0];
	for (int i = 1; i < costs.size(); i++) {
	  if (costs[i] < min_cost) {
	      best_id = i;
	  }
  }

  // realize best state
  states best_state = feasible_states[best_id];
  this->state = best_state.name;
  // set change_lane flag according to the state chosen
  if (this->state == "LCL" || this->state == "LCR") {
    this->change_lane = true;
  }
  else {
    this->change_lane = false;
  }

  // set target lane (current lane in case of prepare lane change)
  this->target_lane = best_state.wpts_lane;

  // set target speed for this lane
  this->target_speed = best_state.target_speed;
  // don't go faster than allowed
  if (this->target_speed > MAX_SPEED) {
    this->target_speed = MAX_SPEED;
  }

  // create a spline for the waypoints
  tk::spline spl;
  spl.set_points(best_state.wpts_x, best_state.wpts_y);

  // generate trajectory
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // reuse remaining points of previous path
  if (this->residual_path_size > 1) {
    for (int i = 0; i < this->residual_path_size; i++) {
      next_x_vals.push_back(this->residual_path_x[i]);
      next_y_vals.push_back(this->residual_path_y[i]);
    }
  }

  // generate new waypoints using spline
  double current_speed = this->v;
  double x_add_on = 0;

  // get vehicles in front
  map<int, predict> cars_ahead = predictions[0];

  // calculate acceleration / deceleration based on speed and the distance of the vehicle in front,
  // absolute value of the speed increment per time step (20 ms)
  // acc = 0.1 m/s / 0.02 s = 5 m/s^2
  double acc;
  double speed_diff = this->target_speed - current_speed;
  if (speed_diff < 0) {
    double brake_dist = speed_diff * speed_diff / 
      ( 2 * (cars_ahead[this->target_lane].distance - 4.) ) * 0.02;
    acc = max({ acc , 0.1 });
    acc = min({ acc , MAX_ACC * 0.02 });
  }
  else {
    acc = 0.1;
  }

  // fill up the trajectory with new points 
  for (int i = 0; i < this->num_wpts - next_x_vals.size(); i++) {
    // accelerate / decelerate if target speed is not reached yet
    double speed_diff = this->target_speed - current_speed;
    if (speed_diff >= acc) {
      current_speed += acc;
    } else if (speed_diff <= -acc) {
      current_speed -= acc;
    }

    // update of car position every 20 ms
    double x_point = x_add_on + 0.02 * current_speed;
    double y_point = spl(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to global c/s
    x_point = cos_yaw * x_ref - sin_yaw * y_ref;
    y_point = sin_yaw * x_ref + cos_yaw * y_ref;

    x_point += this->x;
    y_point += this->y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return { next_x_vals, next_y_vals };
}


vector<string> Vehicle::successor_states() {
  /*
  Provides the possible next states given the current state for the FSM 
  */
  vector<string> possible_successor_states;
  possible_successor_states.push_back("KL");

  // If state is "LCL" or "LCR" ...
  if (this->change_lane == true) {
    // ... and d is not well in the target lane yet ...
    if (this->state.compare("LCL") == 0 && this->d > (2 + 4 * this->target_lane + 1.) || 
        this->state.compare("LCR") == 0 && this->d < (2 + 4 * this->target_lane - 1.)) {
      // ... keep on changeing lane ...
      possible_successor_states.push_back(this->state);
    }
  } else {
    // if not in left lane ...
    if (this->lane != 0) {
      // ... and not in preparing-for-a-lane-change-right state ...
      if (this->state.compare("PLCR") != 0) {
        // add preparing-for-a-lane-change-left state
        possible_successor_states.push_back("PLCL");
      }
      // if in preparing-for-a-lane-change-left state
      if (this->state.compare("PLCL") == 0) {
        // add lane-change-left state
        possible_successor_states.push_back("LCL");
      }
    }
    // if not in right-most lane ...
    if (this->lane != LANES_AVAILABLE - 1) {
      // ... ditto
      if (this->state.compare("PLCL") != 0) {
        possible_successor_states.push_back("PLCR");
      }
      if (this->state.compare("PLCR") == 0) {
        possible_successor_states.push_back("LCR");
      }
    }
  }

  return possible_successor_states;
}


states Vehicle::generate_trajectory(string &state_name, const vector<map<int, predict>> &predictions) {
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */
  if (state_name.compare("KL") == 0) return keep_lane_trajectory(predictions);
  if (state_name.compare("LCL") == 0 || state_name.compare("LCR") == 0)
    return lane_change_trajectory(state_name, predictions);
  if (state_name.compare("PLCL") == 0 || state_name.compare("PLCR") == 0)
    return prep_lane_change_trajectory(state_name, predictions);
}


vector<vector<double>> Vehicle::generate_waypoints(int lane) {
  /*
  Generate waypoints (based on the vehicle c/s) for a rough trajectory (control points for spline).
  */

  vector<double> ptsx_car;
  vector<double> ptsy_car;

  // insert last and current vehicle location in vehicle c/s
  ptsx_car.push_back(this->prev_x_car);
  ptsx_car.push_back(0.);

  ptsy_car.push_back(this->prev_y_car);
  ptsy_car.push_back(0.);

  // define step size based on current velocity
  double step_size = max({ this->v * 1.35 , 20. });

  // add 3 new waypoints each in step_size distance
  for (int i = 0; i < 3; i++)
  {
    vector<double> next_wpt = getXY(this->s + (double)(i + 1) * step_size, (2 + 4 * lane),
      this->maps_s, this->maps_x, this->maps_y);

    // apply homogeneous transformation from global c/s to vehicle c/s
    double shift_x = next_wpt[0] - this->x;
    double shift_y = next_wpt[1] - this->y;

    double next_x = this->cos_yaw * shift_x + this->sin_yaw * shift_y;
    double next_y = -this->sin_yaw * shift_x + this->cos_yaw * shift_y;

    ptsx_car.push_back(next_x);
    ptsy_car.push_back(next_y);
  }

  return { ptsx_car, ptsy_car };
}


double Vehicle::get_lane_speed(int lane, double ref_speed, const vector<map<int, predict>> &predictions) {
  /*
  Calculate target speed for the given lane based on the given reference velocity 
  as well as on the distance and speed of the vehicle in front
  */
  double lane_speed;

  // calculate safe distance 1.35 * v  -> ca. 30 m for 50 mph
  double safe_dist = ref_speed * 1.35;
  // but min twice the safety clearance
  safe_dist = max({ safe_dist , this->safety_clearance });

  // check for vehicles in front
  map<int, predict> cars_ahead = predictions[0];

  if (cars_ahead[lane].count == 0) {
    // if there is no vehicle in front in this lane
    lane_speed = MAX_SPEED;
  }
  else {
    double car_dist = cars_ahead[lane].distance;
    double car_speed = min({ cars_ahead[lane].speed, MAX_SPEED });
    double dist_to_max_speed = this->horizon / 2;
    if (car_dist >= dist_to_max_speed) {
      // if vehicle in front is more than 50 m away
      lane_speed = MAX_SPEED;
    }
    else if (car_dist >= safe_dist) {
      // if vehicle is more than safe distance away
      lane_speed = car_speed + (MAX_SPEED - car_speed) / (dist_to_max_speed - safe_dist)
        * (car_dist - safe_dist);
    }
    else if (car_dist >= this->safety_clearance) {
      // if vehicle is more than safety clearance away
      lane_speed = car_speed * (1. - 0.9 / (safe_dist - this->safety_clearance)
                                 * (safe_dist - car_dist));
    }
    else {
      // if distance is smaller than safety clearance -> reduce speed
      lane_speed = car_speed / this->safety_clearance * car_dist;
    }
  }

  return lane_speed;
}


double Vehicle::get_lane_speed_behind(int lane, const vector<map<int, predict>> &predictions) {
  /*
  Calculate target speed for the given lane based on the distance and speed of the vehicle behind
  */
  // get target speed for this lane
  double lane_speed;

  // check for vehicles behind
  map<int, predict> cars_behind = predictions[1];

  if (cars_behind[lane].count == 0) {
    // if there is no vehicle behind in this lane
    lane_speed = MAX_SPEED;
  }
  else {
    double dist_to_max_speed = 2. * this->safety_clearance;
    double car_dist = cars_behind[lane].distance;
    double car_speed = min({ cars_behind[lane].speed, MAX_SPEED });
    if (car_dist >= dist_to_max_speed) {
      // if vehicle is more than safe distance away
      lane_speed = MAX_SPEED;
    }
    else {
      // if distance is smaller than safe distance -> reduce speed
      lane_speed = MAX_SPEED / dist_to_max_speed * car_dist;
    }
  }

  return lane_speed;
}


states Vehicle::keep_lane_trajectory(const vector<map<int, predict>> &predictions) {
  /*
  Generate a keep lane trajectory
  */
  // get waypoints for current lane
  vector<vector<double>> waypoints = this->generate_waypoints(this->lane);

  // get target speed for this lane
  double target_speed = this->get_lane_speed(this->lane, this->v, predictions);

  states state;
  state.name = "KL";
  state.feasible = true;
  state.intended_lane = this->lane;
  state.intended_speed = target_speed;
  state.wpts_lane = this->lane;
  state.target_speed = target_speed;
  state.wpts_x = waypoints[0];
  state.wpts_y = waypoints[1];

  return state;
}


states Vehicle::prep_lane_change_trajectory(string &state_name, const vector<map<int, predict>> &predictions) {
  /*
  Generate a trajectory preparing for a lane change.
  */
  // get trajectory for current lane
  states state = this->keep_lane_trajectory(predictions);
  state.name = state_name;

  // get intended lane (target lane after lane change)
  int intended_lane = this->lane + this->lane_direction[state_name];
  state.intended_lane = intended_lane;

  // get target speed considering vehicles in front in the intended lane
  double target_speed = this->get_lane_speed(intended_lane, this->v, predictions);

  // also consider vehicles behind
  double target_speed_behind = this->get_lane_speed_behind(intended_lane, predictions);

  state.intended_speed = min({ target_speed , target_speed_behind });

  return state;
}


states Vehicle::lane_change_trajectory(string &state_name, const vector<map<int, predict>> &predictions) {
  /*
  Generate a lane change trajectory.
  */
  // get target lane
  int target_lane;
  if (this->change_lane == true) {
    // if already in change lane mode, don't change target lane
    target_lane = this->target_lane;
  }
  else {
    target_lane = this->lane + this->lane_direction[state_name];
  }

  states state;
  state.name = state_name;
  state.feasible = false;
  state.intended_lane = target_lane;
  state.wpts_lane = target_lane;

  // get target speed considering vehicles in front in the intended lane
  double target_speed;

  // check for vehicles in front
  map<int, predict> cars_ahead = predictions[0];

  if (cars_ahead[target_lane].count == 0) {
    // if there is no vehicle in front in this lane
    target_speed = MAX_SPEED;
  }
  else {
    // calculate safe distance 1.35 * v  -> ca. 30 m for 50 mph
    double safe_dist = this->v * 1.35;
    // but min twice the safety clearance
    safe_dist = max({ safe_dist , this->safety_clearance });
    double dist_to_max_speed = this->horizon / 2;

    double car_dist = cars_ahead[target_lane].distance;
    double car_speed = min({ cars_ahead[target_lane].speed, MAX_SPEED });
    if (car_dist >= dist_to_max_speed) {
      // if vehicle in front is more than 50 m away
      target_speed = MAX_SPEED;
    }
    else if (car_dist >= safe_dist) {
      // if vehicle is more than safe distance away
      target_speed = car_speed + (MAX_SPEED - car_speed) / (dist_to_max_speed - safe_dist)
        * (car_dist - safe_dist);
    }
    else if (car_dist > this->safety_clearance) {
      // if vehicle is more than safety clearance away
      target_speed = car_speed * (1. - 0.9 / (safe_dist - this->safety_clearance)
        * (safe_dist - car_dist));
    }
    else {
      // if distance is smaller than safe distance -> state is not feasible
      return state;
    }
  }

  // also consider vehicles behind
  double target_speed_behind;

  // check for vehicles behind
  map<int, predict> cars_behind = predictions[1];

  if (cars_behind[target_lane].count == 0) {
    // if there is no vehicle behind in this lane
    target_speed_behind = MAX_SPEED;
  }
  else {
    // calculate safe distance 1.35 * v  -> ca. 30 m for 50 mph
    double safe_dist = cars_behind[target_lane].speed * 1.35;
    // but min twice the safety clearance
    safe_dist = max({ safe_dist , this->safety_clearance });
    
    double car_dist = cars_behind[target_lane].distance;

    if (car_dist >= safe_dist) {
      // if vehicle is more than safe distance away
      target_speed_behind = MAX_SPEED;
    }
    else {
      // if distance is smaller than safe distance -> state is not feasible
      return state;
    }
  }

  state.feasible = true;
  state.target_speed = target_speed;
  state.intended_speed = min({ target_speed , target_speed_behind });

  // get waypoints for target lane
  vector<vector<double>> waypoints = this->generate_waypoints(target_lane);
  state.wpts_x = waypoints[0];
  state.wpts_y = waypoints[1];

  return state;
}


vector<map<int, predict>> Vehicle::generate_predictions(double s, int timesteps) {
  /*
  Generate predictions from sensor fusion results
  */
  map<int, predict> cars_ahead;
  map<int, predict> cars_behind;

  // initialize distance with search horizon and with max speed
  for (int lane = 0; lane < LANES_AVAILABLE; lane++) {
    cars_ahead[lane].distance = this->horizon;
    cars_ahead[lane].speed = MAX_SPEED;
    cars_behind[lane].distance = this->horizon / 2.;
    cars_behind[lane].speed = MAX_SPEED;
  }

  // check position and velocity of other vehicles
  for (int i = 0; i < this->sensor_fusion.size(); i++) {
    // check if car is in my lane
    int car_id = (int)this->sensor_fusion[i][0];
    double car_x = this->sensor_fusion[i][1];
    double car_y = this->sensor_fusion[i][2];
    double car_vx = this->sensor_fusion[i][3];
    double car_vy = this->sensor_fusion[i][4];
    double car_s = this->sensor_fusion[i][5];
    double car_d = this->sensor_fusion[i][6];
    double car_speed = sqrt(car_vx*car_vx + car_vy*car_vy);	// m/s

    // check vehicles position after applying residual of previous path
    car_s += (double)timesteps * 0.02 * car_speed;
    // take reset of s at the end of the track into account
    car_s = fmod(car_s, MAX_S);
    double dist = car_s - s;
    if (dist > MAX_S / 2.) {
      dist -= MAX_S;
    } else if (dist < -MAX_S / 2.) {
      dist += MAX_S;
    }

    // if car is close ...
    if (-this->horizon / 2 <= dist && dist <= this->horizon) {

      // get absolute distance
      double abs_dist = fabs(dist);

      // check all lanes ... 
      for (int lane = 0; lane < LANES_AVAILABLE; lane++) {
        // ... if vehicle is occupying it
        if (car_d > (2 + 4 * lane - 3) && car_d < (2 + 4 * lane + 3)) {
          // if the vehicle is in front of us
          if (dist >= 0.) {
            // check if there is already an entry for this lane which is closer
            if (cars_ahead[lane].count == 0 || abs_dist < cars_ahead[lane].distance) {
              // if not insert car data
              cars_ahead[lane].distance = abs_dist;
              cars_ahead[lane].speed = car_speed;
            }
            // if car is close ...
             cars_ahead[lane].count += 1;
          }
          // if the vehicle is behind of us
          else {
            // check if there is already an entry for this lane which is closer
            if (cars_behind[lane].count == 0 || abs_dist < cars_behind[lane].distance) {
              // if not insert car data
              cars_behind[lane].distance = abs_dist;
              cars_behind[lane].speed = car_speed;
            }
            // if car is close ...
            cars_behind[lane].count += 1;
          }
        }
      }
    }
  }

  return { cars_ahead, cars_behind };
}


bool Vehicle::residual_feasible(double s, double d, double end_path_s) {
  /*
  Check if the residual of the previous path is still free (i.e. safe to realize)
  */
  int lane = (int)d / 4;

  vector<map<int, predict>> predictions = generate_predictions(s, 0);

  // check for vehicles
  map<int, predict> cars_ahead = predictions[0];

  double len_residual = end_path_s - s;
  if (len_residual < 0) {
    len_residual += MAX_S;
  }
  if (cars_ahead[lane].distance <= len_residual) {
    return false;
  }

  return true;
}


void Vehicle::update(double x, double y, double s, double d, double yaw, double v,
                     vector<double> residual_path_x, vector<double> residual_path_y,
                     double end_path_s, double end_path_d, 
                     vector<vector<double>> sensor_fusion) {
  /*
  Update vehicle data with current data from simulator
  */
  this->sensor_fusion = sensor_fusion;

  // get number of remaining points of previous path
  this->residual_path_size = residual_path_x.size();

  // check if the residual of the previous path is still free (i.e. safe to realize)
  if (this->residual_path_size > 0 && this->residual_feasible(s, d, end_path_s) == false) {
    this->residual_path_size = 0;
    residual_path_x.clear();
    residual_path_y.clear();
    end_path_s = s;
    end_path_d = d;
  }

  // Get last two points of the previous path (or extrapolate back form current car position)
  if (this->residual_path_size < 2) {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = deg2rad(yaw);
    this->v = v;
    this->a = (v - this->prev_speed) / 0.02;
    this->prev_speed = v;
    this->prev_x = x - cos(this->yaw);
    this->prev_y = y - sin(this->yaw);
  } else {
    // set current position to the last point in the remainder of the previous path
    this->x = residual_path_x[this->residual_path_size - 1];
    this->y = residual_path_y[this->residual_path_size - 1];
    this->prev_x = residual_path_x[this->residual_path_size - 2];
    this->prev_y = residual_path_y[this->residual_path_size - 2];
    this->s = end_path_s;
    this->d = end_path_d;
    this->yaw = atan2(this->y - this->prev_y, this->x - this->prev_x);
    this->v = distance(this->prev_x, this->prev_y, this->x, this->y) / 0.02;
    this->prev_speed = distance(residual_path_x[this->residual_path_size - 3],
                                residual_path_y[this->residual_path_size - 3],
                                this->prev_x, this->prev_y) / 0.02;
    this->a = (v - this->prev_speed) / 0.02;
    this->prev_speed = v;
  }
   
  // convert previous location into vehicle coordinate system
  // (current position in vehicle c/s is (0,0) )
  this->sin_yaw = sin(this->yaw);
  this->cos_yaw = cos(this->yaw);

  // pre-calculations for homogeneous transformations
  double shift_x = this->prev_x - this->x;
  double shift_y = this->prev_y - this->y;

  // apply homogeneous transformation from global c/s to vehicle c/s
  this->prev_x_car =  this->cos_yaw * shift_x + this->sin_yaw * shift_y;
  this->prev_y_car = -this->sin_yaw * shift_x + this->cos_yaw * shift_y;

  // calculate current lane number (last waypoint of the residual
  // of the previous path)
  this->lane = (int)this->d / 4;

  // store remainder of previous path
  this->residual_path_x = residual_path_x;
  this->residual_path_y = residual_path_y;
}

