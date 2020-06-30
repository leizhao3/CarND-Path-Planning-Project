#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"
#include <iostream>
#include <iomanip>

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;



//WEIGHTED_COST_FUNCTIONS
struct WEIGHTED_COST_FUNCTIONS {
    double max_speed_cost = 10000;
    double slow_speed_cost = 500;
    double time_diff_cost = 100;
    double max_LC_time_cost = 100;
    //double max_jerk_cost = 10000;
    //double total_jerk_cost = 1000;
    double side_collision_cost = 10000;
    double side_buffer_cost = 750;
    double front_collision_cost = 10000;
    double front_buffer_cost = 1500;
    //double max_accel_cost = 10000;
    //double max_accel_cost_d = 10000;
    //double total_accel_cost = 50;
    //double total_accel_cost_d = 1000;
    double change_lane_cost = 1;
};



/**
 * @param sensor_fusion
 * @param lane_change
 * @param lane
 * @param car_s [meter] The car's s position in frenet coordinates
 * @param car_d [meter] The car's d position in frenet coordinates
 * @param car_speed [MPH] The car's speed
 * if need to change lane, change within 30 meter of s
 * sensor fusion need to calculate s,s_dot,s_double_dot & d,d_dot,d_double_dot
 * @return 
 * car_s+5  d@car_s+5
 * car_s+15 d@car_s+15
 * car_s+25 d@car_s+25
 */

class Cost {
    public:
        //contructor 
        Cost(Trajectory &trajectory, TrajectorySD &trajectory_sd, Vehicle &vehicle, vector<vector<double>> &sensor_fusion, 
             int lane, double ref_vel, string state, int prev_size) {
            next_path_x_ = trajectory.x_;//in MAP coordinate
            next_path_y_ = trajectory.y_;//in MAP coordinate
            next_path_s_ = trajectory_sd.s_;//in FRENET coordinate
            next_path_d_ = trajectory_sd.d_;//in FRENET coordinate
            sensor_fusion_ = sensor_fusion;
            ref_vel_ = ref_vel;
            lane_ = lane;
            state_ = state;
            prev_size_ = prev_size;
            //potential_lane_ = potential_lane;
        }

        //destructor
        ~Cost() {}

        //void init(vector<double> &next_path_x, vector<double> &next_path_y, double ref_vel,
        //          vector<vector<double>> &sensor_fusion, int lane, int potential_lane);
        double calculateCost(int verbose);
        double maxSpeedCost();
        double slowSpeedCost();

        double sideCollisionCost();
        double sideBufferCost();

        double frontCollisionCost();
        double frontBufferCost();

        //double changeLaneCost();

        double timeDiffCost();
        double maxLCTimeCost();


    private:
        WEIGHTED_COST_FUNCTIONS Weight;
        vector<double> next_path_x_; 
        vector<double> next_path_y_;
        vector<double> next_path_s_; 
        vector<double> next_path_d_;
        double ref_vel_;
        vector<vector<double>> sensor_fusion_;
        int lane_;
        string state_;
        //int potential_lane_;
        int prev_size_;


        //CONSTANTS
        const double MAX_LC_TIME = 3; //[s], the maximum allowable time to make lane change
        const double VEHICLE_RADIUS = 1.5; //[meter], model vehicle as circle to simplify buffer cost calculation
        const double VEHICLE_LENTH = 5.0; //[meter], parapmeter used for front collision
        const double VEHICLE_WIDTH = 3.0; //[meter], parapmeter used for side collision
        const double EXPECTED_JERK_IN_ONE_SEC = 2; //[m/s^2]
        const double EXPECTED_ACC_IN_ONE_SEC = 1; //[m/s]
        const double SPEED_LIMIT = 50; //[MPH]
        const double SLOW_SPEED = 45; //[MPH]


};


/**
 * @param verbose 1 - print segregated cost, 0 - NOT print segreated cost
 */
double Cost::calculateCost(int verbose) {
    
    double cost = 0.0;
    vector<double> del_cost;
    vector<string> title;
    double del_cost_temp;

    del_cost_temp = Weight.max_speed_cost * maxSpeedCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("maxSpd");

    del_cost_temp = Weight.slow_speed_cost * slowSpeedCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("slowSpd");

    del_cost_temp = Weight.side_collision_cost * sideCollisionCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Cllsn_side");

    del_cost_temp = Weight.side_buffer_cost * sideBufferCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Bffr_side");

    del_cost_temp = Weight.front_collision_cost * frontCollisionCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Cllsn_FR");

    del_cost_temp = Weight.front_buffer_cost * frontBufferCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Bffr_FR");

    //del_cost_temp = Weight.change_lane_cost * changeLaneCost();
    //del_cost.push_back(del_cost_temp);
    //title.push_back("changeLaneCost");

    if ((state_.compare("LCL") == 0) || (state_.compare("LCR") == 0)) {
        del_cost_temp = Weight.max_LC_time_cost * maxLCTimeCost();
        del_cost.push_back(del_cost_temp);
        title.push_back("maxLCTime");

        del_cost_temp = Weight.time_diff_cost * timeDiffCost();
        del_cost.push_back(del_cost_temp);
        title.push_back("timeDiff");
    }

    for(int i=0; i<del_cost.size(); i++) {
        cost += del_cost[i];
    }

    if(verbose == 1) {
        int width = 11;
        
        for(int i=0; i<title.size(); i++) {
            cout << setw(width) << title[i];
        }
        cout << endl;

        for(int i=0; i<del_cost.size(); i++) {
            cout << setw(width) << del_cost[i];
        }
        cout << endl;
    }


    return cost;
}

double Cost::maxSpeedCost() {

    double cost;
    if(ref_vel_>SPEED_LIMIT) {
        return 1;
    }
    else {
        return 0;
    }

}

double Cost::slowSpeedCost() {

    double cost; 

    if(ref_vel_>SLOW_SPEED) {
        cost = 0;
    }
    else {
        cost = logistic((SLOW_SPEED-ref_vel_) / (SPEED_LIMIT-SLOW_SPEED));
    }

    //cout << "ref_vel_ = " << ref_vel_ << endl;
    //cout << "slowSpeedCost = " << cost << endl;

    return cost;
}

double Cost::sideCollisionCost() {

    double nearest = nearestDist2Cars(next_path_x_, next_path_y_, sensor_fusion_, ref_vel_, prev_size_);

    if(nearest < VEHICLE_WIDTH) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

double Cost::sideBufferCost() {

    double nearest = nearestDist2Cars(next_path_x_, next_path_y_, sensor_fusion_, ref_vel_, prev_size_);
    return logistic(VEHICLE_WIDTH / nearest);

}

double Cost::frontCollisionCost() {

    double nearest_front = nearestDist2Cars_front(next_path_s_, next_path_d_, sensor_fusion_, ref_vel_, prev_size_);

    if(nearest_front < VEHICLE_LENTH) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

double Cost::frontBufferCost() {

    double nearest_front = nearestDist2Cars_front(next_path_s_, next_path_d_, sensor_fusion_, ref_vel_, prev_size_);
    return logistic(VEHICLE_LENTH / nearest_front);
    
}


/*
double Cost::changeLaneCost() {

    double cost = 0;

    if(potential_lane_ != lane_) {
        cost = 1;
    } 
    else {
        cost = 0;
    }

    //cout << "changeLanceCost() = " << cost << endl;

    return cost;
    
}*/

double Cost::timeDiffCost() {
    double T = 0.0;

    for(int i=0; i<(next_path_x_.size()-1); i++) {
      double dist = distance(next_path_x_[i], next_path_y_[i],next_path_x_[i+1], next_path_y_[i+1]);
        //get distance from the the current point of analysis to the next point of analysis in next_path.
      double dT = dist/(ref_vel_/2.24); 
      T += dT;
    }

    return logistic(T / MAX_LC_TIME);

}

double Cost::maxLCTimeCost() {
    double T = 0.0;
    double cost = 0.0;

    for(int i=0; i<(next_path_x_.size()-1); i++) {
      double dist = distance(next_path_x_[i], next_path_y_[i],next_path_x_[i+1], next_path_y_[i+1]);
        //get distance from the the current point of analysis to the next point of analysis in next_path.
      double dT = dist/(ref_vel_/2.24); 
      T += dT;
    }

    if(T>MAX_LC_TIME) {return 1.0;}
    else {return 0.0;}


}




//In Frenet add evenly 30m space point ahead of the car reference
/*
vector<double> next_wp0 = getXY(car_s+30,2+4*lane,map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,2+4*lane,map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,2+4*lane,map_waypoints_s, map_waypoints_x, map_waypoints_y);
*/

#endif  // COST_FUNCTIONS_H