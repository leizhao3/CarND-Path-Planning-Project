#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * Calculate closest waypoint to current x, y position
 * @param x, @param y 
 * @param maps_x, @param maps_y
 * @return 
 */
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

/**
 * Returns next waypoint of the closest waypoint. For example, if there is closest
 * waypoint right behind you, you don't want to go to that waypoint. Instead, you
 * go to NextWaypoint farther out.
 * @param x, @param y 
 * @param theta 
 * @param maps_x, @param maps_y
 * @return 
 */
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) { //pi()/2 is used because that is where the car is looking (ahead of car)
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

/**
 * Transform from Cartesian x,y coordinates to Frenet s,d coordinates
 * @param x, @param y 
 * @param theta 
 * @param maps_x, @param maps_y
 * @return 
 */
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

/**
 * Transform from Frenet s,d coordinates to Cartesian x,y
 * @param s, @param d 
 * @param maps_s
 * @param maps_x, @param maps_y
 * @return 
 */
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


/**
 * convert MAP coord to vehicle coord
 * @param 
 * @return 
 */
void map2car(
  vector<double> &coord_map_x, vector<double> &coord_map_y,
  double ref_x, double ref_y, double ref_yaw) {
    for(int i=0; i<coord_map_x.size(); i++) {
      double shift_x = coord_map_x[i] - ref_x;
      double shift_y = coord_map_y[i] - ref_y;

      coord_map_x[i] =   shift_x*cos(ref_yaw) + shift_y*sin(ref_yaw);
      coord_map_y[i] = - shift_x*sin(ref_yaw) + shift_y*cos(ref_yaw);

    }
    return;
  }



/**
 * convert car coord to MAP coord
 * @param 
 * @return 
 */
void car2map(
  vector<double> &coord_car_x, vector<double> &coord_car_y,
  double ref_x, double ref_y, double ref_yaw) {
    for(int i=0; i<coord_car_x.size(); i++) {
      double x_point = ref_x + coord_car_x[i]*cos(ref_yaw) - coord_car_y[i]*sin(ref_yaw);
      double y_point = ref_y + coord_car_x[i]*sin(ref_yaw) + coord_car_y[i]*cos(ref_yaw);

      coord_car_x[i] = x_point;
      coord_car_y[i] = y_point;


    }
    return;
  }

/**define a path made up of (x,y) points that the car will visit
 * sequentially every .02 seconds
 * Method: 
 * 1. Generate spline from previous_path(last 2 points) & next_path(s+30, s+60, s+90)
 * 2. Generate points on the next_path from the spline up to s+30 to make total 50 points. 
 */
void generate_traj(vector<double> &next_path_x_ref, vector<double> &next_path_y_ref, 
  vector<double> &previous_path_x, vector<double> &previous_path_y, double car_x, double car_y, double car_yaw, double car_s, double ref_vel, 
  int target_lane, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {

  vector<double> spline_x; //point x for spline generation
  vector<double> spline_y; //point x for spline generation

  double ref_x, ref_y, ref_yaw;
  double ref_x_prev, ref_y_prev;

  int prev_size = previous_path_x.size();

  //if previous size is almost empty, use the car as starting reference
  if(prev_size < 2) {
    ref_x = car_x; 
    ref_y = car_y;
    ref_yaw = deg2rad(car_yaw);

    //use two points that make the path tangent to the car. dist=1
    ref_x_prev = car_x - cos(car_yaw); 
    ref_y_prev = car_y - sin(car_yaw); 

  } else {
    //use last two points from previous path to create future path
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    ref_x_prev = previous_path_x[prev_size-2];
    ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

  }

  spline_x.push_back(ref_x_prev);
  spline_x.push_back(ref_x);

  spline_y.push_back(ref_y_prev);
  spline_y.push_back(ref_y);

  //In Frenet add evenly 30m space point ahead of the car reference
  vector<double> next_wp0 = getXY(car_s+30,2+4*target_lane,maps_s, maps_x, maps_y);
  vector<double> next_wp1 = getXY(car_s+60,2+4*target_lane,maps_s, maps_x, maps_y);
  vector<double> next_wp2 = getXY(car_s+90,2+4*target_lane,maps_s, maps_x, maps_y);

  spline_x.push_back(next_wp0[0]);
  spline_x.push_back(next_wp1[0]);
  spline_x.push_back(next_wp2[0]);

  spline_y.push_back(next_wp0[1]);
  spline_y.push_back(next_wp1[1]);
  spline_y.push_back(next_wp2[1]);

  // convert map coordiante to vehicle coordinate
  map2car(spline_x, spline_y, ref_x, ref_y, ref_yaw);

  // create a spline
  tk::spline traj;
  traj.set_points(spline_x, spline_y);
  
  double target_x = 30.0;
  double target_y = traj(target_x);
  double target_dist = sqrt(target_x*target_x+target_y*target_y);
  int next_size = 50 - prev_size; //How many point we want to generate in next path beside the point in previous path
  double N = target_dist/(0.02*ref_vel/2.24);
    //Number of point within target_x.
    //0.02 is from the car updates every 0.02 second
    //2.24 convert MPH to m/s
  double x_add_on = target_x/N; //evenly spacing within target_x

  double x_point, y_point;
  x_point = 0;
  
  for(int i=0; i<next_size; i++) {
    x_point += x_add_on;
    y_point = traj(x_point);

    next_path_x_ref.push_back(x_point);
    next_path_y_ref.push_back(y_point);

  }

  // convert CAR coordiante to MAP coordinate
  car2map(next_path_x_ref, next_path_y_ref, ref_x, ref_y, ref_yaw);

  return;
}


//find all potential lane
vector<int> find_lane(int lane) {
    vector<int> potential_lane;

    if(lane == 0) {
        potential_lane.push_back(lane+1);
    } 
    else if (lane == 1) {
        potential_lane.push_back(lane-1);
        potential_lane.push_back(lane+1);
    } 
    else if (lane == 2) {
        potential_lane.push_back(lane-1);
    }
    potential_lane.push_back(lane);

    return potential_lane;
}

/**
 * A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
 */
double logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

#endif  // HELPERS_H