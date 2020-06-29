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
        PathPlanning(
            Vehicle &ego_vehicle, Trajectory &previous_path, 
            vector<vector<double>> &sensor_fusion, MapWaypoints &map_waypoints,
            int lane, double ref_vel, string state, vector<double> &end_path);

        // Destructor
        ~PathPlanning();

        Trajectory chooseNextState();
        vector<string> successorStates();
        Trajectory generateTrajectory(int ref_lane, double ref_vel);
        double findRefVel(int lane);

        void statusUpdate(int &lane, double &ref_vel, string &state); //pass updated value to upper func
    
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
        int lane_;
        double ref_vel_;
        string state_;
        vector<double> end_path_;
   
};


PathPlanning::PathPlanning(
    Vehicle &ego_vehicle, Trajectory &previous_path, 
    vector<vector<double>> &sensor_fusion, MapWaypoints &map_waypoints, 
    int lane, double ref_vel, string state, vector<double> &end_path) {

    ego_vehicle_ = ego_vehicle;
    previous_path_ = previous_path;
    sensor_fusion_ = sensor_fusion;
    map_waypoints_ = map_waypoints;
    lane_ = lane;
    ref_vel_ = ref_vel;
    state_ = state;
    end_path_ = end_path;
   

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
    vector<double> costs = {};
    vector<int> ref_lane_out = {};
    vector<double> ref_vel_out = {};

    cout << "=====================================PATH PLANNING=====================================" << endl;

    for(int i=0; i<states.size(); i++) {
        int ref_lane = lane_;
        double ref_vel = ref_vel_;

        if (states[i].compare("KL") == 0) {
            //no change on the ref_lane
            //no change on the ref_vel
        }
        else if (states[i].compare("LCL") == 0) {
            ref_lane -= 1; //minus 1 lane
            ref_vel = findRefVel(ref_lane);
        }
        else if (states[i].compare("LCR") == 0) {
            ref_lane += 1; // add 1 lane
            ref_vel = findRefVel(ref_lane);
        }
        else if (states[i].compare("PLCL") == 0) {
            //no change on the ref_lane
            ref_vel = findRefVel((ref_lane-1)); //set the speed to the vehicle on the left lane
        }
        else if (states[i].compare("PLCR") == 0) {
            //no change on the ref_lane
            ref_vel = findRefVel((ref_lane+1)); //set the speed to the vehicle on the right lane
        }

        Trajectory trajectory = generateTrajectory(ref_lane, ref_vel);
        
        int width = 15; 
        /*
        cout << "trajectory" << endl;
        cout << setw(width) << "x" << setw(width) << "y" << endl;
        for(int i=0; i<trajectory.x_.size(); i++) {
            cout << setw(width) << trajectory.x_[i] << setw(width) << trajectory.y_[i] << endl;
        }
        cout << endl;*/

        TrajectorySD trajectory_sd = getFrenet_traj(trajectory, map_waypoints_, ego_vehicle_, end_path_);
        /*
        cout << "trajectory_sd" << endl;
        cout << setw(width) << "s" << setw(width) << "d" << endl;
        for(int i=0; i<trajectory_sd.s_.size(); i++) {
            cout << setw(width) << trajectory_sd.s_[i] << setw(width) << trajectory_sd.d_[i] << endl;
        }
        cout << endl;*/

        Cost cost = Cost(trajectory, trajectory_sd, ego_vehicle_, sensor_fusion_, ref_lane, ref_vel, states[i]);
        cout << "---------" << states[i] << "---------" << endl;
        double cost_temp = cost.calculateCost(1);

        costs.push_back(cost_temp);
        ref_lane_out.push_back(ref_lane);
        ref_vel_out.push_back(ref_vel);
        final_trajectories.push_back(trajectory);

    }

    //debug
    int width = 15;
    cout << "------------------------SUMMARY------------------------" << endl;
    cout << setw(width) << "states is ";
    for(int i=0; i<states.size(); i++) {
        cout << setw(width) << states[i];
    }
    cout << endl;
    cout << setw(width) << "costs is ";
    for(int i=0; i<costs.size(); i++) {
        cout << setw(width) << costs[i];
    }
    cout << "\n\n" << endl;

    vector<double>::iterator best_cost = std::min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    //update state_ in this class so that it would pass to upper function
    state_ = states[best_idx];
    lane_ = ref_lane_out[best_idx];
    ref_vel_ = ref_vel_out[best_idx];


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

  //state = the current state of the vehicle. defined in the vehicle class.
  if(state_.compare("KL") == 0) {
      if(lane_ == 0) {
          states.push_back("PLCR");
      } 
      else if (lane_ == 1) {
          states.push_back("PLCL");
          states.push_back("PLCR");
      }
      else if (lane_ == 2) {
          states.push_back("PLCL");
      }
  } else if (state_.compare("PLCL") == 0) {
    if (lane_ == 1 || lane_ == 2) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state_.compare("PLCR") == 0) {
    if (lane_ == 0 || lane_ == 1) {
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

    double ref_vel_temp = ref_vel_;
    double check_s_min = 100000;

    //find the last vehicle on target line in sensor fusion
    for(int i=0; i<sensor_fusion_.size(); i++) {
        double check_d = sensor_fusion_[i][6];

        //Check only the vehicle in that lane
        if(check_d<(2+4*ref_lane+2) && (check_d>(2+4*ref_lane-2))) {

            double check_s = sensor_fusion_[i][5];
            //make sure the target vehicle is the last vehicle on that lane
            if(check_s < check_s_min) {

                double target_vehicle_vx = sensor_fusion_[i][3];
                double target_vehicle_vy = sensor_fusion_[i][4];
                double target_vehicle_vel = sqrt(target_vehicle_vx*target_vehicle_vx + target_vehicle_vy*target_vehicle_vy);
                
                //match the speed to the tail vehicle
                if(target_vehicle_vel < ref_vel_temp) {
                    ref_vel_temp -= .224;
                }
            }
        }
    }

    //there is no vehicle on target lane
    if(check_s_min == 100000) {
        ref_vel_temp += .224;
    }

    return ref_vel_temp;
}


/**
 * define a path made up of (x,y) points that the car will visit
 * sequentially every .02 seconds
 * Method: 
 * 1. Generate spline from previous_path(last 2 points) & next_path(s+30, s+60, s+90)
 * 2. Generate points on the next_path from the spline up to s+30 to make total 50 points. 
 * @return next_path_x_ref & next_path_y_ref in MAP coordinate
 */
Trajectory PathPlanning::generateTrajectory(int ref_lane, double ref_vel) {

    vector<double> previous_path_x = previous_path_.x_;
    vector<double> previous_path_y = previous_path_.y_;
    double car_x = ego_vehicle_.car_x_;
    double car_y = ego_vehicle_.car_y_;
    double car_yaw = ego_vehicle_.car_yaw_;
    double car_s = ego_vehicle_.car_s_;
    const vector<double> maps_s = map_waypoints_.s_;
    const vector<double> maps_x = map_waypoints_.x_;
    const vector<double> maps_y = map_waypoints_.y_;
  
    Trajectory next_path_ref; //output trajectory based on current state
  
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

    /*
    int width = 15;
    cout << setw(width) << "spline_x" << setw(width) << "spline_y" << endl;
    for(int i=0; i<spline_x.size(); i++) {
        cout << setw(width) << spline_x[i] << setw(width) << spline_y[i] << endl;
    }*/

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


//Pass lane, ref_vel, state to upper function. 
void PathPlanning::statusUpdate(int &lane, double &ref_vel, string &state) {
    lane = lane_;
    ref_vel = ref_vel_;
    state = state_;
}



#endif  // PATH_PLANNING_H