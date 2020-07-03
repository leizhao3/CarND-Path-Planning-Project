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
    const double max_speed_cost = 10000;
    const double slow_speed_cost = 500;
    const double time_diff_cost = 150;
    const double max_LC_time_cost = 10000;
    const double side_collision_cost = 10000;
    const double side_buffer_cost = 250;
    const double front_collision_cost = 10000;
    const double front_buffer_cost = 2000;
    const double rear_collision_cost = 10000;
    const double rear_buffer_cost = 1500;
    const double double_LC_cost = 200;
};

class Cost {
    public:
        //contructor 
        Cost(Trajectory &trajectory, TrajectorySD &trajectory_sd, Vehicle &vehicle, vector<vector<double>> &sensor_fusion, 
             int lane, double ref_vel, string state, int prev_size, int LC_idx) {
            next_path_x_ = trajectory.x_;
            next_path_y_ = trajectory.y_;
            next_path_s_ = trajectory_sd.s_;
            next_path_d_ = trajectory_sd.d_;
            sensor_fusion_ = sensor_fusion;
            ref_vel_ = ref_vel;
            lane_ = lane;
            state_ = state;
            prev_size_ = prev_size;
            LC_idx_ = LC_idx;
        }

        //destructor
        ~Cost() {}

        double calculateCost(int verbose);
        double maxSpeedCost(); //cost for exceeding speed limit
        double slowSpeedCost(); //cost for slow speed

        double sideCollisionCost(); //cost for side collision
        double sideBufferCost(); //cost for how close the min side distance is

        double frontCollisionCost(); //cost for front collision
        double frontBufferCost(); //cost for how close the min front distance is
        double rearCollisionCost(); //cost for rear collision
        double rearBufferCost(); //cost for how close the min rear distance is

        double doubleLaneChangeCost(); //cost for double lane change

        double timeDiffCost(); //cost for how long the lane change execute
        double maxLCTimeCost(); //cost for exceeding maximum lane change


    private:
        WEIGHTED_COST_FUNCTIONS Weight; //weights for different cost function
        vector<double> next_path_x_; //[meter] x-value for next path trajectory in future 90 meter, in MAP coordinate
        vector<double> next_path_y_; //[meter] y-value for next path trajectory in future 90 meter, in MAP coordinate
        vector<double> next_path_s_; //[meter] s-value for next path trajectory in future 90 meter, in FRENET coordinate
        vector<double> next_path_d_; //[meter] d-value for next path trajectory in future 90 meter, in FRENET coordinate
        double ref_vel_; //[MPH] reference velocity
        vector<vector<double>> sensor_fusion_; // sensor fusion data
        int lane_; //current lane of ego vehicle
        string state_; //current state of the ego vehicle
        int prev_size_; //the size of previous path
        int LC_idx_; //the number of 0.02s to last lane change

        //CONSTANTS
        const double MAX_LC_TIME = 3; //[s], the maximum allowable time to make lane change
        const double LC_TIMEGAP = 3; //[second], the min time gap between the lane change to previous lane change
        const double VEHICLE_RADIUS = 1.5; //[meter], model vehicle as circle to simplify buffer cost calculation
        const double VEHICLE_LENTH = 5.0; //[meter], parapmeter used for front collision
        const double VEHICLE_WIDTH = 2.5; //[meter], parapmeter used for side collision
        const double EXPECTED_JERK_IN_ONE_SEC = 2; //[m/s^2]
        const double EXPECTED_ACC_IN_ONE_SEC = 1; //[m/s]
        const double SPEED_LIMIT = 50; //[MPH]
        const double SLOW_SPEED = 45; //[MPH]
        const double SAFE_DIST_SIDE = 6; //[meter]
};



double Cost::calculateCost(int verbose) {
    /**
    * @param verbose 1 - print segregated cost, 0 - NOT print segreated cost
    * @return the cost for the reference trajectory
    */
    
    double cost = 0.0; //return paremeter
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
    title.push_back("Cll_SD");

    del_cost_temp = Weight.side_buffer_cost * sideBufferCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Bffr_SD");

    del_cost_temp = Weight.front_collision_cost * frontCollisionCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Cll_FR");

    del_cost_temp = Weight.front_buffer_cost * frontBufferCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Bffr_FR");

    del_cost_temp = Weight.rear_collision_cost * rearCollisionCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Cll_RR");

    del_cost_temp = Weight.rear_buffer_cost * rearBufferCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("Bffr_RR");


    if ((state_.compare("LCL") == 0) || (state_.compare("LCR") == 0)) {
        del_cost_temp = Weight.max_LC_time_cost * maxLCTimeCost();
        del_cost.push_back(del_cost_temp);
        title.push_back("maxLC");

        del_cost_temp = Weight.time_diff_cost * timeDiffCost();
        del_cost.push_back(del_cost_temp);
        title.push_back("TDiff");

        if (LC_idx_ < 50*LC_TIMEGAP) {//update 50 times per second
            del_cost_temp = Weight.double_LC_cost * doubleLaneChangeCost();
            del_cost.push_back(del_cost_temp);
            title.push_back("2LC");
        }
    }

    for(int i=0; i<del_cost.size(); i++) {
        cost += del_cost[i];
    }

    if(verbose == 1) {
        int width = 8;
        
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
        return 0;
    }
    else {
        return logistic((SLOW_SPEED-ref_vel_) / (SPEED_LIMIT-SLOW_SPEED));
    }
}

double Cost::sideCollisionCost() {

    double nearest_side = nearestDist2Cars(next_path_x_, next_path_y_, sensor_fusion_, ref_vel_, prev_size_);
    //cout << "nearest_side @ sideCollisionCost() = " << nearest_side << endl;

    if(nearest_side < VEHICLE_WIDTH) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

double Cost::sideBufferCost() {

    double nearest = nearestDist2Cars(next_path_x_, next_path_y_, sensor_fusion_, ref_vel_, prev_size_);

    /*
    if(nearest > SAFE_DIST_SIDE) {
        return 0.0;
    } else {
        return logistic(VEHICLE_WIDTH / nearest);
    }*/

    return logistic(VEHICLE_WIDTH / nearest);
}

double Cost::frontCollisionCost() {

    double nearest_front = nearestDist2Cars_s(
        next_path_s_, next_path_d_, sensor_fusion_, ref_vel_, prev_size_, "front");
    //cout << "nearest_front @ frontCollisionCost() = " << nearest_front << endl;

    if(nearest_front < VEHICLE_LENTH) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

double Cost::frontBufferCost() {

    double nearest_front = nearestDist2Cars_s(
        next_path_s_, next_path_d_, sensor_fusion_, ref_vel_, prev_size_,"front");
    /*
    if(nearest_front > SAFE_DIST) {
        return 0.0;
    } else {
        return logistic(VEHICLE_LENTH / nearest_front);
    }
    */
    return logistic(VEHICLE_LENTH / nearest_front);
}

double Cost::rearCollisionCost() {

    double nearest_rear = nearestDist2Cars_s(
        next_path_s_, next_path_d_, sensor_fusion_, ref_vel_, prev_size_, "rear");
    //cout << "nearest_rear @ rearCollisionCost() = " << nearest_rear << endl;

    if(nearest_rear < VEHICLE_LENTH) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

double Cost::rearBufferCost() {

    double nearest_rear = nearestDist2Cars_s(
        next_path_s_, next_path_d_, sensor_fusion_, ref_vel_, prev_size_,"rear");

    /*
    if(nearest_rear > SAFE_DIST) {
        return 0.0;
    } else {
        return logistic(VEHICLE_LENTH / nearest_rear);
    }*/
    return logistic(VEHICLE_LENTH / nearest_rear);
}

double Cost::doubleLaneChangeCost() {
    if (LC_idx_ < 50*LC_TIMEGAP) {//update 50 times per second
        return 1.0;
    } else {
        return 0.0;
    }    
}

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

#endif  // COST_FUNCTIONS_H