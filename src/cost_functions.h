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
    double slow_speed_cost = 1000;
    double time_diff_cost = 1000;
    double max_LC_time_cost = 100;
    double max_jerk_cost = 10000;
    double total_jerk_cost = 1000;
    double collision_cost = 10000;
    double buffer_cost = 100;
    double max_accel_cost = 10000;
    double max_accel_cost_d = 10000;
    double total_accel_cost = 50;
    double total_accel_cost_d = 1000;
    double change_lane_cost = 5;
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


//Change the target car to follow on other lane. The target car has max speed of the rest of the car & has s larger than car_s
int find_target_car(vector<vector<double>> &sensor_fusion, bool lane_change, int lane, double car_s, double car_d, double car_speed) {

    int target_car_id;
    double check_speed_max = 0;

    for(int i=0; i<sensor_fusion.size(); i++) {

        double check_d = sensor_fusion[i][6];
        double check_s = sensor_fusion[i][5];
        double check_vy = sensor_fusion[i][4];
        double check_vx = sensor_fusion[i][3];
        double check_speed = sqrt(check_vx*check_vx + check_vy*check_vy);

        //find the car NOT in my lane
        if(((check_d>(2+4*lane+2)) || (check_d<(2+4*lane-2))) &&
            (check_s>car_s) && 
            (check_speed>car_speed) && (check_speed>check_speed_max)) {
               target_car_id = i;
               check_speed_max = check_speed;
        }
    }

    return target_car_id;
}

class CalculateCost {
    public:
        //contructor 
        CalculateCost() {}

        //destructor
        ~CalculateCost() {}

        void init(vector<double> &next_path_x, vector<double> &next_path_y, double ref_vel,
                  vector<vector<double>> &sensor_fusion, int lane, int potential_lane);
        double calculateCost(int verbose);
        double maxSpeedCost();
        double slowSpeedCost();

        double collisionCost();
        double bufferCost();

        double changeLaneCost();

        double timeDiffCost();
        double maxLCTimeCost();


    private:
        WEIGHTED_COST_FUNCTIONS Weight;
        vector<double> next_path_x_; 
        vector<double> next_path_y_;
        double ref_vel_;
        vector<vector<double>> sensor_fusion_;
        vector<vector<double>> predictions_;
        int lane_;
        int potential_lane_;


        //CONSTANTS
        double MAX_ACCEL = 10; //[m/s^2], the maximum acceleration
        double MAX_JERK = 10; //[m/s^3], the maximum jerk
        double MAX_LC_TIME = 3; //[s], the maximum allowable time to make lane change
        double VEHICLE_RADIUS = 1.5; //[meter], model vehicle as circle to simplify collision detection
        double EXPECTED_JERK_IN_ONE_SEC = 2; //[m/s^2]
        double EXPECTED_ACC_IN_ONE_SEC = 1; //[m/s]
        double SPEED_LIMIT = 50; //[MPH]
        double SLOW_SPEED = 45; //[MPH]


};

void CalculateCost::init(
    vector<double> &next_path_x, vector<double> &next_path_y, double ref_vel,
    vector<vector<double>> &sensor_fusion, int lane, int potential_lane) {
    next_path_x_ = next_path_x;
    next_path_y_ = next_path_y;
    ref_vel_ = ref_vel;
    sensor_fusion_ = sensor_fusion;
    lane_ = lane;
    potential_lane_ = potential_lane;
    
}

/**
 * @param verbose 1 - print segregated cost, 0 - NOT print segreated cost
 */
double CalculateCost::calculateCost(int verbose) {
    
    double cost = 0.0;
    vector<double> del_cost;
    vector<string> title;
    double del_cost_temp;

    del_cost_temp = Weight.max_speed_cost * maxSpeedCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("maxSpeedCost");

    del_cost_temp = Weight.slow_speed_cost * slowSpeedCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("slowSpeedCost");

    del_cost_temp = Weight.collision_cost * collisionCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("collisionCost");

    del_cost_temp = Weight.buffer_cost * bufferCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("bufferCost");

    del_cost_temp = Weight.change_lane_cost * changeLaneCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("changeLaneCost");

    del_cost_temp = Weight.max_LC_time_cost * maxLCTimeCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("maxLCTimeCost");

    del_cost_temp = Weight.time_diff_cost * timeDiffCost();
    del_cost.push_back(del_cost_temp);
    title.push_back("timeDiffCost");

    for(int i=0; i<del_cost.size(); i++) {
        cost += del_cost[i];
    }

    if(verbose == 1) {
        int width = 15;
        
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

double CalculateCost::maxSpeedCost() {

    double cost;

    if(ref_vel_>SPEED_LIMIT) {
        cost = 1;
    }
    else {
        cost = 0;
    }

    //cout << "max_speed_cost = " << cost << endl;

    return cost;
}

double CalculateCost::slowSpeedCost() {

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

double CalculateCost::collisionCost() {

    double nearest = nearestDist2Cars(next_path_x_, next_path_y_, sensor_fusion_, ref_vel_);
    double cost = 0;

    if(nearest < 2*VEHICLE_RADIUS) {
        cost = 1.0;
    }
    else {
        cost = 0.0;
    }

    //cout << "collisionCost() = " << cost << endl;

    return cost;
    
}

double CalculateCost::bufferCost() {

    double nearest = nearestDist2Cars(next_path_x_, next_path_y_, sensor_fusion_, ref_vel_);
    double cost = logistic(2*VEHICLE_RADIUS / nearest);

    //cout << "nearest = " << nearest << endl;
    //cout << "bufferCost() = " << cost << endl;

    return cost;
    
}

double CalculateCost::changeLaneCost() {

    double cost = 0;

    if(potential_lane_ != lane_) {
        cost = 1;
    } 
    else {
        cost = 0;
    }

    //cout << "changeLanceCost() = " << cost << endl;

    return cost;
    
}

double CalculateCost::timeDiffCost() {
    double T = 0.0;

    for(int i=0; i<(next_path_x_.size()-1); i++) {
      double dist = distance(next_path_x_[i], next_path_y_[i],next_path_x_[i+1], next_path_y_[i+1]);
        //get distance from the the current point of analysis to the next point of analysis in next_path.
      double dT = dist/(ref_vel_/2.24); 
      T += dT;
    }

    return logistic(T / MAX_LC_TIME);

}

double CalculateCost::maxLCTimeCost() {
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