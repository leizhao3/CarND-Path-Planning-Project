#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>

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
  int LC_idx = 9999; // to avoid double lane change

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, 
               &lane,&ref_vel,&target_vel, &state, &LC_idx]
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
           * @param vx [MPH] car's x velocity
           * @param vy [MPH] car's y velocity
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

        
          /*---------------PATH PLANNING---------------*/

          PathPlanning path_planning(
            ego_vehicle, previous_path, sensor_fusion, map_waypoints, lane, ref_vel, state, end_path, LC_idx);
          Trajectory next_path;

          next_path = path_planning.chooseNextState();
          path_planning.statusUpdate(lane, ref_vel, state);

          //cout << "state after updated = " << state << endl;
          LC_idx += 1;

          //avoid double lane change
          /*
          if((state.compare("LCL")==0) || (state.compare("LCR")==0)) {
            cout << "making " << state;
            cout << "current LC_idx = " << LC_idx << ", setting it to 0" << endl;
            LC_idx = 0;
          }*/

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