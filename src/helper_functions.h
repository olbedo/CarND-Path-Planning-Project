#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <vector>

using namespace std;

struct states {
  string name;            // name of the state
  bool feasible;          // is state feasible (considering current traffic)
  int intended_lane;      // intended lane (after lane change)
  double intended_speed;  // target speed for waypoints lane
  int wpts_lane;          // lane used for waypoints (final lane of current state)
  double target_speed;    // target speed for waypoints lane
  vector<double> wpts_x;  // x-values of waypoints
  vector<double> wpts_y;  // y-values of waypoints
};

struct predict {
  int count = 0;          // total number of vehicles within horizon
  double distance;        // distance to ego vehicles position
  double speed;           // measured speed 
};

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);


// total number of lanes
const int LANES_AVAILABLE = 3;

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

// max velocity 50 mph -> 22.35 m/s
const double MAX_SPEED = 49.5 * 0.447;

// max acceleration 10 m/s^2
const double MAX_ACC = 10.;

// max jerk 10 m/s^3
const double MAX_JERK = 10.;


// distance between 2 points
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


#endif /* HELPER_FUNCTIONS_H_ */