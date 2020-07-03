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
#include <stdio.h> 
#include <stdlib.h> 

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
            int lane, double ref_vel, string state, vector<double> &end_path, int LC_idx);

        // Destructor
        ~PathPlanning();

        Trajectory chooseNextState();
        vector<string> successorStates();
        Trajectory generateTrajectory(int ref_lane, double ref_vel, string ref_state);
        double findRefVel(int lane);

        void statusUpdate(int &lane, double &ref_vel, string &state); //pass updated value to upper func

        double getMaxAccel_T(int ref_lane, double ref_vel, string ref_state);
    
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
        int LC_idx_;
        double s_gap_min_;

        const double MAX_ACCEL = 10; //[m/s^2], the maximum acceleration
        const double MAX_JERK = 10; //[m/s^3], the maximum jerk
        const double SAFE_DIST = 30; //[meter]
        //const int LC_TIMEGAP = 5; //[second], the min time gap between the lane change
   
};


PathPlanning::PathPlanning(
    Vehicle &ego_vehicle, Trajectory &previous_path, 
    vector<vector<double>> &sensor_fusion, MapWaypoints &map_waypoints, 
    int lane, double ref_vel, string state, vector<double> &end_path, int LC_idx) {

    ego_vehicle_ = ego_vehicle;
    previous_path_ = previous_path;
    sensor_fusion_ = sensor_fusion;
    map_waypoints_ = map_waypoints;
    lane_ = lane;
    ref_vel_ = ref_vel;
    state_ = state;
    end_path_ = end_path;
    LC_idx_ = LC_idx;


    //find the gap to the sensed car in front of us.
    double check_s_min = 99999;
    for(int i=0; i<sensor_fusion_.size(); i++) {

        double check_d = sensor_fusion_[i][6];

        //Check only the vehicle in that lane
        if(check_d<(2+4*lane_+2) && (check_d>(2+4*lane_-2))) {

            double check_s = sensor_fusion_[i][5];

            double dist_s = fabs(check_s - ego_vehicle_.car_s_);

            if(dist_s < check_s_min) {
                check_s_min = dist_s;
            }
        }
    }
    
    s_gap_min_ = check_s_min;
}

PathPlanning::~PathPlanning() {};

Trajectory PathPlanning::chooseNextState() {
    /**
     * @return The best (lowest cost) trajectory corresponding to the next ego 
     *   vehicle state.
     */

    Trajectory next_path; //output parameter
 
    vector<string> states = successorStates();
    vector<Trajectory> final_trajectories;
    vector<double> costs = {};
    vector<int> ref_lane_out = {};
    vector<double> ref_vel_out = {};

    int prev_size = previous_path_.x_.size();
    int next_size = 50 - prev_size; 
        //How many point we want to generate in next path beside the point in previous path

    cout << "=====================================PATH PLANNING=====================================" << endl;
    cout << "s_gap_min_ = " << s_gap_min_ << endl;

    for(int i=0; i<states.size(); i++) {
        int ref_lane = lane_;
        double ref_vel = ref_vel_;

        if (states[i].compare("KL") == 0) {
            //no change on the ref_lane
            ref_vel = findRefVel(ref_lane); //set the speed to the vehicle in front of ego car
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

        Trajectory trajectory = generateTrajectory(ref_lane, ref_vel, states[i]);

        TrajectorySD trajectory_sd = getFrenet_traj(trajectory, map_waypoints_, ego_vehicle_, end_path_);

        Cost cost = Cost(trajectory, trajectory_sd, ego_vehicle_, 
                        sensor_fusion_, ref_lane, ref_vel, states[i], prev_size, LC_idx_);
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
    Trajectory final_trajectory = final_trajectories[best_idx];
    
    for(int i=0; i<next_size; i++) {

        next_path.x_.push_back(final_trajectory.x_[i]);
        next_path.y_.push_back(final_trajectory.y_[i]);

    }

    return next_path;
}

vector<string> PathPlanning::successorStates() {    
    /**
    * Provides the possible next states given the current state for the Finite State Machine
    * @return viable states
    */
    vector<string> states;
    states.push_back("KL");

    //ONLY "KL" when the ego car is at the beginning
    //NOT allow for "PLCL" and "PLCR" when s_gap_min_ is smaller than SAFE_DIST-10 --> Only "KL"
    if ((ego_vehicle_.car_s_<200) || (s_gap_min_<(SAFE_DIST-10))) {
        return states;
    } else {
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
        } 
        else if (state_.compare("PLCL") == 0) {
            if (lane_ == 1 || lane_ == 2) {
                states.push_back("PLCL");
                states.push_back("LCL");
            }
        } 
        else if (state_.compare("PLCR") == 0) {
            if (lane_ == 0 || lane_ == 1) {
                states.push_back("PLCR");
                states.push_back("LCR");
            }
        }
    }
    return states;
}


double PathPlanning::findRefVel(int ref_lane) {
    /**
     * KL: find the target vehicle to follow & match ego vehicle to that speed. 
     */

    double ref_vel_temp = ref_vel_;
    double check_s_min = 100000;
    double dist_s = 0;
    bool find_target_vehicle = false;
    double target_vehicle_vel = 0; //[m/s]
    double target_vehicle_s = 1000000;
    double s_gap; 
    string type;

    //find the vehicle on target line having the cloest s-value as ego vehicle
    for(int i=0; i<sensor_fusion_.size(); i++) {

        double check_d = sensor_fusion_[i][6];

        //Check only the vehicle in that lane
        if(check_d<(2+4*ref_lane+2) && (check_d>(2+4*ref_lane-2))) {

            double check_s = sensor_fusion_[i][5];

            dist_s = fabs(check_s - ego_vehicle_.car_s_);

            if(dist_s < check_s_min) {

                check_s_min = dist_s;
                find_target_vehicle = true;
                target_vehicle_s = check_s;
                s_gap = dist_s;

                if(dist_s > SAFE_DIST) {
                    target_vehicle_vel = 49.5; //[MPH]
                } else { //dist_s <= SAFE_DIST
                    double target_vehicle_vx = sensor_fusion_[i][3];
                    double target_vehicle_vy = sensor_fusion_[i][4];
                    target_vehicle_vel = sqrt(target_vehicle_vx*target_vehicle_vx + target_vehicle_vy*target_vehicle_vy);
                }
            }
        }
    }

    //debug
    /*
    if(!find_target_vehicle) {
        cout << "\nNO TARGET VEHICLE FOUND in 30m in fornt of ego car" << endl;
        cout << "s_gap = " << s_gap << endl;
    } else {
        cout << "\nFINAL: target_vehicle_vel = " << target_vehicle_vel << "[MPH]" << endl;
        cout << "s_gap = " << s_gap << endl;
    }*/

    
    if(!find_target_vehicle) {
        if(ref_vel_temp < 49.5) {
            ref_vel_temp += .224; //if there is no vehicle on that lane, accelerate faster
        }
    } else {
        //"The target vehicle is in front of ego vehicle"
        if (target_vehicle_s > ego_vehicle_.car_s_) {
            //cout << "The target vehicle is in front of ego vehicle" << endl;
            if (s_gap > SAFE_DIST) {
                if(ref_vel_temp < 49.5) {
                    ref_vel_temp += .224;
                }
            } 
            else { // s_gap <= 30
                if(ref_vel_temp > target_vehicle_vel-3) {
                    ref_vel_temp -= .224*4*logistic(SAFE_DIST/s_gap);
                    ref_vel_temp = std::max(ref_vel_temp, 0.001); 
                } else {
                    ref_vel_temp += .224*logistic(target_vehicle_vel/ref_vel_temp); 
                        //reward higher target_vehicle_vel --> differentiate PLCL & PLCR
                }
            }
        } 
        //The target vehicle is behind ego vehicle
        else {
            //cout << "The target vehicle is behind ego vehicle" << endl;
            if(ref_vel_temp < 49.5) {
                ref_vel_temp += .224;
            }
        }
    }

    //cout << "final ref_vel_temp = " << ref_vel_temp << "MPH" << endl;

    return ref_vel_temp;
}


Trajectory PathPlanning::generateTrajectory(int ref_lane, double ref_vel, string ref_state) {
    /**
    * define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    * Method: 
    * 1. Generate spline from previous_path(last 2 points) & next_path(s+30, s+60, s+90)
    * 2. Generate points on the next_path from the spline up to s+30 to make total 50 points. 
    * @return next_path_x_ref & next_path_y_ref in MAP coordinate
    */

    Trajectory next_path_ref; 
        //output trajectory based on current stateTrajectory next_path_ref; //output trajectory based on current state

    vector<double> previous_path_x = previous_path_.x_;
    vector<double> previous_path_y = previous_path_.y_;
    double car_x = ego_vehicle_.car_x_;
    double car_y = ego_vehicle_.car_y_;
    double car_yaw = ego_vehicle_.car_yaw_;
    double car_s = ego_vehicle_.car_s_;
    const vector<double> maps_s = map_waypoints_.s_;
    const vector<double> maps_x = map_waypoints_.x_;
    const vector<double> maps_y = map_waypoints_.y_;
  
    vector<double> spline_x; //point x for spline generation
    vector<double> spline_y; //point x for spline generation
  
    double ref_x, ref_y, ref_yaw;
    double ref_x_prev, ref_y_prev;
  
    //if previous size is almost empty, use the car as starting reference
    int prev_size = previous_path_x.size();
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

    if(prev_size > 0) {
        car_s = end_path_[0];
    }

    vector<double> next_wp0,next_wp1,next_wp2;

    if ((ref_state.compare("LCL") == 0) || (ref_state.compare("LCR") == 0)) {
        // "LCR", "LCL": In Frenet add evenly 30m space point ahead of the car reference with spread out d
        next_wp0 = getXY(car_s+30,4*ref_lane,maps_s, maps_x, maps_y);
        next_wp1 = getXY(car_s+60,4*ref_lane+2,maps_s, maps_x, maps_y);
        next_wp2 = getXY(car_s+90,4*ref_lane+2,maps_s, maps_x, maps_y);
        
    } 
    else if (ref_state.compare("PLCL") == 0) {
        next_wp0 = getXY(car_s+30,4*ref_lane+2-0.1,maps_s, maps_x, maps_y);
        next_wp1 = getXY(car_s+60,4*ref_lane+2-0.1,maps_s, maps_x, maps_y);
        next_wp2 = getXY(car_s+90,4*ref_lane+2-0.1,maps_s, maps_x, maps_y);
    }
    else if (ref_state.compare("PLCR") == 0) {
        next_wp0 = getXY(car_s+30,4*ref_lane+2+0.1,maps_s, maps_x, maps_y);
        next_wp1 = getXY(car_s+60,4*ref_lane+2+0.1,maps_s, maps_x, maps_y);
        next_wp2 = getXY(car_s+90,4*ref_lane+2+0.1,maps_s, maps_x, maps_y);
    }
    else if (ref_state.compare("KL") == 0) {
        next_wp0 = getXY(car_s+30,4*ref_lane+2,maps_s, maps_x, maps_y);
        next_wp1 = getXY(car_s+60,4*ref_lane+2,maps_s, maps_x, maps_y);
        next_wp2 = getXY(car_s+90,4*ref_lane+2,maps_s, maps_x, maps_y);
    }

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
    
    for(int i=0; i<N; i++) {
        x_point += x_add_on;
        y_point = traj(x_point);

        next_path_ref.x_.push_back(x_point);
        next_path_ref.y_.push_back(y_point);
    }

  // convert CAR coordiante to MAP coordinate
  car2map(next_path_ref.x_, next_path_ref.y_, ref_x, ref_y, ref_yaw);

  return next_path_ref;
}


void PathPlanning::statusUpdate(int &lane, double &ref_vel, string &state) {
    //Pass lane, ref_vel, state to upper function. 
    lane = lane_;
    ref_vel = ref_vel_;
    state = state_;
}

double PathPlanning::getMaxAccel_T(int ref_lane, double ref_vel, string ref_state) {
    //get maximum tangential acceleration

    double max_accel; //output parameter
    vector<double> time;
    vector<double> velocity; 
    vector<double> acceleration; 

    Trajectory trajectory = generateTrajectory(ref_lane, ref_vel,ref_state);
    TrajectorySD trajectory_sd = getFrenet_traj(trajectory, map_waypoints_, ego_vehicle_, end_path_);

    
    for(int i=0; i<(trajectory_sd.d_.size()-1); i++) {
        double dist = distance(trajectory_sd.s_[i],     trajectory_sd.d_[i],
                               trajectory_sd.s_[i+1],   trajectory_sd.d_[i+1]);
        double dT = dist/ref_vel;
        time.push_back(dT);

        double vel_temp = (trajectory_sd.d_[i+1]-trajectory_sd.d_[i])/dT;
        velocity.push_back(vel_temp);

    }

    for(int i=0; i<(velocity.size()-1); i++) {

        double accel_temp = (velocity[i]-velocity[i+1])/time[i];
        acceleration.push_back(accel_temp);

    }

    max_accel = *std::max_element(begin(acceleration), end(acceleration));


    return max_accel;
}


#endif  // PATH_PLANNING_H