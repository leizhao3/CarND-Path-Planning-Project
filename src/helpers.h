#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include <iomanip>
#include <iostream>

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;



struct Vehicle {
  double car_x_; //[meter] The car's x position in MAP coordinates
  double car_y_; //[meter] The car's y position in MAP coordinates
  double car_s_; //[meter] The car's s position in frenet coordinates
  double car_d_; //[meter] The car's d position in frenet coordinates
  double car_yaw_; //[deg] The car's yaw angle in the map
  double car_speed_; //[MPH] The car's speed
};

struct Trajectory {
  vector<double> x_;
  vector<double> y_;
};

struct TrajectorySD {
  vector<double> s_;
  vector<double> d_;
};

struct MapWaypoints {
  vector<double> x_;
  vector<double> y_;
  vector<double> s_;
  vector<double> dx_;
  vector<double> dy_;
};


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
 * get Frenet for a trajectory
 * @param trajectory_xy trajectory in MAP coordinate
 * @param map_waypoints
 * @return trajectory_sd trajectory in FRENET coordinate
 */
TrajectorySD getFrenet_traj(Trajectory &trajectory_xy, MapWaypoints &map_waypoints, Vehicle &ego_car, vector<double> &end_path) {

  TrajectorySD trajectory_sd;

  const vector<double> maps_x = map_waypoints.x_;
  const vector<double> maps_y = map_waypoints.y_;

  double x, x_prev, y, y_prev, theta;


  for(int i=0; i<trajectory_xy.x_.size(); i++) {
    if(i == 0) {
      trajectory_sd.s_.push_back(end_path[0]);
      trajectory_sd.d_.push_back(end_path[1]);

    } else {
      x = trajectory_xy.x_[i];
      x_prev = trajectory_xy.x_[i-1];
      y = trajectory_xy.y_[i];
      y_prev = trajectory_xy.y_[i-1];
      theta = atan2(y-y_prev, x-x_prev);

      vector<double> sd_temp = getFrenet(x,y,theta,maps_x,maps_y);

      trajectory_sd.s_.push_back(sd_temp[0]);
      trajectory_sd.d_.push_back(sd_temp[1]);
    }
  }

  return trajectory_sd;
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

/**
 * define a path made up of (x,y) points that the car will visit
 * sequentially every .02 seconds
 * Method: 
 * 1. Generate spline from previous_path(last 2 points) & next_path(s+30, s+60, s+90)
 * 2. Generate points on the next_path from the spline up to s+30 to make total 50 points. 
 * @return next_path_x_ref & next_path_y_ref in MAP coordinate
 */
void generateTraj(vector<double> &next_path_x_ref, vector<double> &next_path_y_ref, 
  vector<double> &previous_path_x, vector<double> &previous_path_y, 
  double car_x, double car_y, double car_yaw, double car_s, double ref_vel, 
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
  double N = target_dist/(0.02*ref_vel/2.24);
    //Number of point within target_x.
    //0.02 is from the car updates every 0.02 second
    //2.24 convert MPH to m/s
  double x_add_on = target_x/N; //evenly spacing within target_x

  double x_point, y_point;
  x_point = 0;

  int next_size = 50 - prev_size; 
    //How many point we want to generate in next path beside the point in previous path
  
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
vector<int> findLane(int lane) {
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
 * A function that returns a value between 0 and 1 for x in the range [0, infinity] and 
 *                                        -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
 */
double logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

/**
 * Prediction other vehicles [s, s_dot, s_double_dot, d, d_dot, d_double_dot] based on sensor fusion data.
 * Assumption:
 *    s_double_dot = 0
 *    d_dot = 0
 *    d_double_dot = 0
 * @param sensor_fusion Sensor Fusion Data, a 2d vector of all other cars on the same side of the road.
 *                      Accessed via [ID][@param]
  * @param ID car's unique ID
  * @param x [meter] car's x position in map coordinates
  * @param y [meter] car's y position in map coordinates
  * @param vx [m/s] car's x velocity
  * @param vy [m/s] car's y velocity
  * @param s [meter] car's s position in frenet coordinates
  * @param d [meter] car's d position in frenet coordinates. 
 * @return [ID][s, s_dot, s_double_dot, d, d_dot, d_double_dot]
       
 */
vector<vector<double>> predictionCars(vector<vector<double>> sensor_fusion) {
  vector<vector<double>> prediction;

  for(int i=0; i<sensor_fusion.size(); i++) {
    
    double s = sensor_fusion[i][4];
    double vx = sensor_fusion[i][2];
    double vy = sensor_fusion[i][3];
    double s_dot = sqrt(vx*vx + vy*vy);
    double s_double_dot = 0;

    double d = sensor_fusion[i][5];
    double d_dot = 0;
    double d_double_dot = 0;
    



    prediction.push_back({s,s_dot,s_double_dot, d,d_dot,d_double_dot});
    
  }

  return prediction;
}


/**
  * @param ID car's unique ID
  * @param x [meter] car's x position in map coordinates
  * @param y [meter] car's y position in map coordinates
  * @param vx [m/s] car's x velocity
  * @param vy [m/s] car's y velocity
  * @param s [meter] car's s position in frenet coordinates
  * @param d [meter] car's d position in frenet coordinates. 
 */
double nearestDist2SingleCar(
  vector<double> &next_path_x, vector<double> &next_path_y, 
  vector<double> &sensor_fusion_single, double ref_vel) {
    
    double closest = 999999;

    double x = sensor_fusion_single[1];
    double x_dot = sensor_fusion_single[3];
    double x_double_dot = 0;

    double y = sensor_fusion_single[2];
    double y_dot = sensor_fusion_single[4];
    double y_double_dot = 0;

    for(int i=0; i<(next_path_x.size()-1); i++) {
      double dist = distance(next_path_x[i], next_path_y[i],next_path_x[i+1], next_path_y[i+1]);
        //get distance from the the current point of analysis to the next point of analysis in next_path.
      double dT = dist/(ref_vel/2.24); 
        //the time needed for ego vehicle to get to the next point
        //ref_vel/2.24 convert MPH to m/s
      x += x_dot*dT + x_double_dot*dT*dT;
      y += y_dot*dT + y_double_dot*dT*dT;
      double dist2ego = distance(x, y, next_path_x[i], next_path_y[i]);

      if(dist2ego < closest) {
        closest = dist2ego;
        //cout << "Getting closer distance " << endl;
        //cout << "dT = " << dT << endl;
        //cout << "x\ty\tnext_path_x[i]\tnext_path_y[i]" << endl;
        //cout << x << "\t" << y << "\t" << next_path_x[i] << "\t" << next_path_y[i] << endl;
      } 
    }

    return closest;
}


/**
 * Calculates the closest distance to any vehicle during a trajectory.
 * 
* Sensor Fusion Data, a 2d vector of all other cars on the same side 
* of the road.Accessed via [ID][@param]
* @param ID car's unique ID
* @param x [meter] car's x position in map coordinates
* @param y [meter] car's y position in map coordinates
* @param vx [m/s] car's x velocity
* @param vy [m/s] car's y velocity
* @param s [meter] car's s position in frenet coordinates
* @param d [meter] car's d position in frenet coordinates. 
           
 */
double nearestDist2Cars(
  vector<double> &next_path_x, vector<double> &next_path_y, 
  vector<vector<double>> &sensor_fusion, double ref_vel) {
    
    double closest = 999999;
    for(int i=0; i<sensor_fusion.size(); i++) {
      double dist = nearestDist2SingleCar(next_path_x, next_path_y, sensor_fusion[i], ref_vel);
      //cout << "Vehicle ID = " << i << endl;
      //cout << "dist = " << dist << endl;

      if(dist<closest) {
        closest = dist;
      }
    }
    return closest;
    
  }


/**
  * @param ID car's unique ID
  * @param x [meter] car's x position in map coordinates
  * @param y [meter] car's y position in map coordinates
  * @param vx [m/s] car's x velocity
  * @param vy [m/s] car's y velocity
  * @param s [meter] car's s position in frenet coordinates
  * @param d [meter] car's d position in frenet coordinates. 
 */
double nearestDist2SingleCar_front(
  vector<double> &next_path_s, vector<double> &next_path_d, 
  vector<double> &sensor_fusion_single, double ref_vel) {
    
    double closest = 999999;

    double s = sensor_fusion_single[5];
    double vx = sensor_fusion_single[3];
    double vy = sensor_fusion_single[4];
    double s_dot = sqrt(vx*vx+vy*vy); //assume the worst: all velocity is in s
    double s_double_dot = 0;

    double d = sensor_fusion_single[6];

    for(int i=0; i<(next_path_s.size()-1); i++) {
      double dist = distance(next_path_s[i], next_path_d[i],next_path_s[i+1], next_path_d[i+1]);
        //get distance from the the current point of analysis to the next point of analysis in next_path.
      double dT = dist/(ref_vel/2.24); 
        //the time needed for ego vehicle to get to the next point
        //ref_vel/2.24 convert MPH to m/s
      s += s_dot*dT + s_double_dot*dT*dT; //the s of the other vehicle in lane

      double dist2ego_front = fabs(s-next_path_s[i]);

      //Only check the vehicle showing up in the range of d of ego vehicle
      if((d>next_path_d[i]-1.5) && (d<next_path_d[i]+1.5)) {
        if(dist2ego_front < closest) {
          closest = dist2ego_front;
          //cout << "Getting closer distance " << endl;
          //cout << "dT = " << dT << endl;
          //cout << "x\ty\tnext_path_x[i]\tnext_path_y[i]" << endl;
          //cout << x << "\t" << y << "\t" << next_path_x[i] << "\t" << next_path_y[i] << endl;
        } 
      }
    }

    return closest;
}


/**
 * Calculates the closest FRONT distance to any vehicle during a trajectory.
 * 
 * Sensor Fusion Data, a 2d vector of all other cars on the same side 
 * of the road.Accessed via [ID][@param]
 * @param ID car's unique ID
 * @param x [meter] car's x position in map coordinates
 * @param y [meter] car's y position in map coordinates
 * @param vx [m/s] car's x velocity
 * @param vy [m/s] car's y velocity
 * @param s [meter] car's s position in frenet coordinates
 * @param d [meter] car's d position in frenet coordinates.         
 */
double nearestDist2Cars_front(
  vector<double> &next_path_s, vector<double> &next_path_d, 
  vector<vector<double>> &sensor_fusion, double ref_vel) {
    
    double closest = 999999;
    for(int i=0; i<sensor_fusion.size(); i++) {

      double dist = nearestDist2SingleCar_front(next_path_s, next_path_d, sensor_fusion[i], ref_vel);

      if(dist<closest) {
        closest = dist;
      }
    }
    return closest;
  }

#endif  // HELPERS_H