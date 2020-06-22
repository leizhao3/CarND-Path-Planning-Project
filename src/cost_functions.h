#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"

// for convenience
using std::string;
using std::vector;


//CONSTANTS
double MAX_ACCEL = 10; //[m/s^2], the maximum acceleration
double MAX_JERK = 10; //[m/s^3], the maximum jerk
double MAX_LC_TIME = 3; //[s], the maximum allowable time to make lane change
double VEHICLE_RADIUS = 1.5; //[meter], model vehicle as circle to simplify collision detection
double EXPECTED_JERK_IN_ONE_SEC = 2; //[m/s^2]
double EXPECTED_ACC_IN_ONE_SEC = 1; //[m/s]

//WEIGHTED_COST_FUNCTIONS
vector<double> WEIGHTED_COST_FUNCTIONS = {
    1, //time_diff_cost
    10000, //max_jerk_cost
    1000, //total_jerk_cost
    10000, //collision_cost
    10, //buffer_cost
    10000, //max_accel_cost
    10000, //max_accel_cost_d
    50, //total_accel_cost
    1000 //total_accel_cost_d
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


double time_diff_cost() {
    double cost = 0.2;
    return cost;
}

double calculate_cost(vector<double> &next_path_x, vector<double> &next_path_y) {
    
    double cost = 0.0;

    for(int i=0; i<WEIGHTED_COST_FUNCTIONS.size(); i++) {
        if(i==0) {
            cost += WEIGHTED_COST_FUNCTIONS[i]*time_diff_cost();
        }
    }

    return cost;
}




//In Frenet add evenly 30m space point ahead of the car reference
/*
vector<double> next_wp0 = getXY(car_s+30,2+4*lane,map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,2+4*lane,map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,2+4*lane,map_waypoints_s, map_waypoints_x, map_waypoints_y);
*/

#endif  // COST_FUNCTIONS_H