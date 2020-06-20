#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// maximum acceleration: 10 [m/s^2]
// maximum jerk: 10 [m/s^3]
// maximum lane change time: 3 [s]
// speed limit: 50 [MPH] or 22.352 [meter/s]

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // Create variable for the lane & velocity
          int lane = 1; //start out at the center lane. lane[LEFT,MID,RIGHT] = [0,1,2]
          double ref_vel = 49.5; // [mph]. The max speed the vehicle would go to. Speed Limit = 50.

          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"]; //[meter] The car's x position in MAP coordinates
          double car_y = j[1]["y"]; //[meter] The car's y position in MAP coordinates
          double car_s = j[1]["s"]; //[meter] The car's s position in frenet coordinates
          double car_d = j[1]["d"]; //[meter] The car's d position in frenet coordinates
          double car_yaw = j[1]["yaw"]; //[deg] The car's yaw angle in the map
          double car_speed = j[1]["speed"]; //[MPH] The car's speed

          // Previous path data given to the Planner. Return the previous list but with 
          // PROCESSED POINTS REMOVED, to show how far along the path has processed
          // since last time.
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"]; //The previous list's last point's frenet s value
          double end_path_d = j[1]["end_path_d"]; //The previous list's last point's frenet d value


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

            

          json msgJson;


          /*-----Use points in previous path to generate smoother path------*/
          /**define a path made up of (x,y) points that the car will visit
           * sequentially every .02 seconds
           * Method: 
           */

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
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
          int next_size = 50 - prev_size; //How many point we want to generate in next path beside the point in previous path
          double N = target_dist/(0.02*ref_vel/2.24);//Number of point within target_x
          double x_add_on = target_x/N; //evenly spacing within target_x

          vector<double> next_path_x;
          vector<double> next_path_y;

          double x_point, y_point;
          x_point = 0;
          
          for(int i=0; i<next_size; i++) {
            x_point += x_add_on;
            y_point = traj(x_point);

            next_path_x.push_back(x_point);
            next_path_y.push_back(y_point);

          }

          // convert CAR coordiante to MAP coordinate
          car2map(next_path_x, next_path_y, ref_x, ref_y, ref_yaw);






          // define the actual (x,y) points used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // attach previous path points
          for(int i=0; i<prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // attach next path points
          for(int i=0; i<next_size; i++) {
            next_x_vals.push_back(next_path_x[i]);
            next_y_vals.push_back(next_path_y[i]);
          }
          
          













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