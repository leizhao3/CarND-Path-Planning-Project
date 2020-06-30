#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>    // std::max

#include "json.hpp"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "cost_functions.h"
#include "path_planning.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  double max_accel = 10; //[m/s^2], the maximum acceleration
  double max_jerk = 10; //[m/s^3], the maximum jerk

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Create variable for the lane & velocity
  int lane = 1; //start out at the center lane. lane[LEFT,MID,RIGHT] = [0,1,2]
  double ref_vel = 0; // [mph]. 
  double target_vel = 49.5; // [mph]. The initial target speed is set slightly below speed limit.
  string state = "KL"; //"KL" or "LCL" or "LCR" or "PLCL" or "PLCL"

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, 
               &lane,&ref_vel,&target_vel, &state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    
    MapWaypoints map_waypoints = {map_waypoints_x, map_waypoints_y, map_waypoints_s,map_waypoints_dx, map_waypoints_dy};

    double lane_width = 4; //[meter]
    double lane_center_d = 2 + 4*lane;
    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"]; //[meter] The car's x position in MAP coordinates
          double car_y = j[1]["y"]; //[meter] The car's y position in MAP coordinates
          double car_s = j[1]["s"]; //[meter] The car's s position in frenet coordinates
          double car_d = j[1]["d"]; //[meter] The car's d position in frenet coordinates
          double car_yaw = j[1]["yaw"]; //[deg] The car's yaw angle in the map
          double car_speed = j[1]["speed"]; //[MPH] The car's speed
          Vehicle ego_vehicle = {car_x, car_y, car_s, car_d, car_yaw, car_speed};

          // Previous path data given to the Planner. Return the previous list but with 
          // PROCESSED POINTS REMOVED, to show how far along the path has processed
          // since last time.
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          Trajectory previous_path = {previous_path_x, previous_path_y};


          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"]; //The previous list's last point's frenet s value
          double end_path_d = j[1]["end_path_d"]; //The previous list's last point's frenet d value
          vector<double> end_path = {end_path_s, end_path_d};

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
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          
          int width = 10;

          /*
          cout << "Print out ego car ......................." << endl;
          cout << setw(width) <<"ego car"
               << setw(width) <<"car_x"
               << setw(width) <<"car_y"
               << setw(width) <<"car_s"
               << setw(width) <<"car_d"
               << setw(width) <<"car_yaw"
               << setw(width) <<"car_speed" << endl;
          cout << setw(width) <<"ego car"
               << setw(width) <<car_x
               << setw(width) <<car_y
               << setw(width) <<car_s
               << setw(width) <<car_d
               << setw(width) <<car_yaw
               << setw(width) <<car_speed << endl;

          cout << "Print out sensor fusion ......................." << endl;
          cout << setw(width) <<"i"
               << setw(width) <<"ID"
               << setw(width) <<"x"
               << setw(width) <<"y"
               << setw(width) <<"vx"
               << setw(width) <<"vy"
               << setw(width) <<"s"
               << setw(width) <<"d" << endl;
          for(int i=0; i<sensor_fusion.size(); i++) {
            cout << setw(width) << i;
            for(int j=0; j<sensor_fusion[i].size(); j++) {
              cout << setw(width) << sensor_fusion[i][j];
            }
            cout << endl;
          }*/

          bool too_close = false;
          //bool too_close_rear = false;
          //bool too_close_left = false;
          //bool too_close_right = false;
          double deceleration_dist; //the distance to decelerate

          //find ref_v to use
          for(int i=0; i<sensor_fusion.size(); i++) {
            //find the car in my lane
            float check_d = sensor_fusion[i][6];
            if(check_d<(lane_center_d+2) && (check_d>(lane_center_d-2))) {
              double check_vx = sensor_fusion[i][3];
              double check_vy = sensor_fusion[i][4];
              double check_speed = sqrt(check_vx*check_vx + check_vy*check_vy);
              double check_s = sensor_fusion[i][5];
              double s_gap = check_s - car_s; //the gap between ego car to check car

              if((check_s>car_s) && (s_gap<50)) {

                too_close = true;
                target_vel = check_speed; //set the target velocity to the vehicle in front of us
                deceleration_dist = s_gap - 30; //adaptive distance. when deceleration_dist = 0, ref_vel=check_vel

              }
            }
          }

          if(too_close) {
            if(ref_vel > (target_vel-3)) {//slightly below the target_vel
              double deceleration = (ref_vel-target_vel)*(ref_vel-target_vel)/fabs(deceleration_dist); 
              deceleration = std::min(2.0, deceleration); //to avoid larger acceleration
              ref_vel -= deceleration * 0.02;
              ref_vel = std::max(0.0001, ref_vel); //to avoid negative velocity when the velocity is small
            }

            if(deceleration_dist<0) {
              cout << "deceleration_dist<0, larger deceleration is needed" << endl;
            }
          } 
          /*
          else {

            if (ref_vel < 49.5) {
              ref_vel += .224;
            }
          }*/


          /*---------------Use points in previous path to generate smoother path---------------*/
          /**define a path made up of (x,y) points that the car will visit
           * sequentially every .02 seconds
           * Method: 
           * 1. Generate spline from previous_path(last 2 points) & next_path(s+30, s+60, s+90)
           * 2. Generate points on the next_path from the spline up to s+30 to make total 50 points. 
           */

          PathPlanning path_planning(ego_vehicle, previous_path, sensor_fusion, map_waypoints, lane, ref_vel, state, end_path);
          Trajectory next_path;

          next_path = path_planning.chooseNextState();
          path_planning.statusUpdate(lane, ref_vel, state);

          cout << "state after updated = " << state << endl;

          int prev_size = previous_path_x.size();
          int next_size = 50 - prev_size; 
              //How many point we want to generate in next path beside the point in previous path
          
          // define the actual (x,y) points used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          // attach previous path points
          for(int i=0; i<prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            //cout << setw(width) << next_x_vals[i] << setw(width) << next_y_vals[i] << endl;
          }

          // attach next path points
          //cout << "attach the next path" << endl;
          for(int i=0; i<next_size; i++) {
            next_x_vals.push_back(next_path.x_[i]);
            next_y_vals.push_back(next_path.y_[i]);
            //cout << setw(width) << next_x_vals[i+prev_size] << setw(width) << next_y_vals[i+prev_size] << endl;
          }


          
          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}