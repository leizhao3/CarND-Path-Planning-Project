#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H


#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"
#include "cost_functions.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;


class PathPlanning {
    public: 
        // Constructors
        //PathPlanning();
        
        PathPlanning(
            Vehicle &ego_vehicle, Trajectory &previous_path, 
            vector<vector<double>> &sensor_fusion, MapWaypoints &map_waypoints);

        // Destructor
        ~PathPlanning();

        Trajectory chooseNextState();
        vector<string> successorStates();
        Trajectory generateTrajectory(string state);
        double findRefVel(int lane);
    
    private:
        Vehicle ego_vehicle_;
        Trajectory previous_path_;
        MapWaypoints map_waypoints_;
        vector<vector<double>> sensor_fusion_;
            /**
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
   
};


PathPlanning::PathPlanning(
    Vehicle &ego_vehicle, Trajectory &previous_path, 
    vector<vector<double>> &sensor_fusion, MapWaypoints &map_waypoints) {

    ego_vehicle_ = ego_vehicle;
    previous_path_ = previous_path;
    sensor_fusion_ = sensor_fusion;
    map_waypoints_ = map_waypoints;

}

PathPlanning::~PathPlanning() {};


Trajectory PathPlanning::chooseNextState() {
  /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param predictions predictions map, a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @return The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * vector<string> Vehicle::successor_states()
   * 
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * vector<Vehicle> Vehicle::generate_trajectory(string state, 
   *                                         map<int, vector<Vehicle>> &predictions)
   * 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   * float calculate_cost(const Vehicle &vehicle, 
   *                  const map<int, vector<Vehicle>> &predictions, 
   *                  const vector<Vehicle> &trajectory) {
   * // Sum weighted cost functions to get total cost for trajectory.
   *
   * TODO: Your solution here.
   */

  vector<string> states = successorStates();
  vector<Trajectory> final_trajectories;
  Cost cost = Cost(previous_path_, ego_vehicle_, sensor_fusion_);
  vector<double> costs = {};

  for(int i=0; i<states.size(); i++) {
    Trajectory trajectory = generateTrajectory(states[i]);
    double cost_temp = cost.calculateCost(0);
    costs.push_back(cost_temp);
    final_trajectories.push_back(trajectory);
  }

  vector<double>::iterator best_cost = std::min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  return final_trajectories[best_idx];
}

vector<string> PathPlanning::successorStates() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  /**
   * @return only return viable states
   */
  vector<string> states;
  states.push_back("KL");
  int lane = ego_vehicle_.lane_;
  string state = ego_vehicle_.state_;

  //state = the current state of the vehicle. defined in the vehicle class.
  if(state.compare("KL") == 0) {
      if(lane == 0) {
          states.push_back("PLCR");
      } 
      else if (lane == 1) {
          states.push_back("PLCL");
          states.push_back("PLCR");
      }
      else if (lane == 2) {
          states.push_back("PLCL");
      }
  } else if (state.compare("PLCL") == 0) {
    if (lane == 1 || lane == 2) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane == 0 || lane == 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

/**
 * find the target vehicle to follow & match ego vehicle to that speed. 
 * Follow the last vehicle on the lane specified. If there is no vehicle, no change on ref_vel.
 */
double PathPlanning::findRefVel(int ref_lane) {

    int target_vehicle_ID = -1;
    double check_s_min = 100000;
    double ref_vel;

    //find the last vehicle on that line in sensor fusion
    for(int i=0; i<sensor_fusion_.size(); i++) {
        double check_d = sensor_fusion_[i][6];

        //Check only the vehicle in that lane
        if(check_d<(2+4*ref_lane+2) && (check_d>(2+4*ref_lane-2))) {

            double check_s = sensor_fusion_[i][5];
            //make sure the target vehicle is the last vehicle on that lane
            if(check_s < check_s_min) {

                target_vehicle_ID = sensor_fusion_[i][0];
                double target_vehicle_vx = sensor_fusion_[i][3];
                double target_vehicle_vy = sensor_fusion_[i][4];
                double target_vehicle_speed = sqrt(target_vehicle_vx*target_vehicle_vx + target_vehicle_vy*target_vehicle_vy);
                ref_vel = target_vehicle_speed;
            }
        }
    }

    //if there is no vehicle on that lane, no change on the ref_vel
    if(target_vehicle_ID == -1) {
        ref_vel = ego_vehicle_.ref_vel_;
    }

    return ref_vel;
}


/**
 * define a path made up of (x,y) points that the car will visit
 * sequentially every .02 seconds
 * Method: 
 * 1. Generate spline from previous_path(last 2 points) & next_path(s+30, s+60, s+90)
 * 2. Generate points on the next_path from the spline up to s+30 to make total 50 points. 
 * @return next_path_x_ref & next_path_y_ref in MAP coordinate
 */
Trajectory PathPlanning::generateTrajectory(string state) {

    vector<double> previous_path_x = previous_path_.x_;
    vector<double> previous_path_y = previous_path_.y_;
    double car_x = ego_vehicle_.car_x_;
    double car_y = ego_vehicle_.car_y_;
    double car_yaw = ego_vehicle_.car_yaw_;
    double car_s = ego_vehicle_.car_s_;
    double ref_vel = ego_vehicle_.ref_vel_;
    const vector<double> maps_s = map_waypoints_.s_;
    const vector<double> maps_x = map_waypoints_.x_;
    const vector<double> maps_y = map_waypoints_.y_;
  
    Trajectory next_path_ref; //output trajectory based on current state
  
    //decision making on ref_vel & ref_lane;
    int ref_lane; //referance lane for cost evaluation
    int target_vehicle_ID;
  
    if (state.compare("KL") == 0) {
        //ref_vel = ref_vel; //no change on speed
        ref_lane = ego_vehicle_.lane_;
    }
    else if (state.compare("LCL") == 0) {
        ref_lane -= 1; //minus 1 lane
        ref_vel = findRefVel(ref_lane);
    }
    else if (state.compare("LCR") == 0) {
        ref_lane += 1; // add 1 lane
        ref_vel = findRefVel(ref_lane);
    }
    else if (state.compare("PLCL") == 0) {
        ref_lane = ego_vehicle_.lane_; //no change lane
        ref_vel = findRefVel((ref_lane-1)); //set the speed to the vehicle on the left lane
    }
    else if (state.compare("PLCR") == 0) {
        ref_lane = ego_vehicle_.lane_; //no change lane
        ref_vel = findRefVel((ref_lane+1)); //set the speed to the vehicle on the left lane
    }

  
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
    vector<double> next_wp0 = getXY(car_s+30,2+4*ref_lane,maps_s, maps_x, maps_y);
    vector<double> next_wp1 = getXY(car_s+60,2+4*ref_lane,maps_s, maps_x, maps_y);
    vector<double> next_wp2 = getXY(car_s+90,2+4*ref_lane,maps_s, maps_x, maps_y);

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

        next_path_ref.x_.push_back(x_point);
        next_path_ref.y_.push_back(y_point);

  }

  // convert CAR coordiante to MAP coordinate
  car2map(next_path_ref.x_, next_path_ref.y_, ref_x, ref_y, ref_yaw);

  return next_path_ref;
}





#endif  // PATH_PLANNING_H