#ifndef VEHICLES_H
#define VEHICLES_H


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
        PathPlanning();
        PathPlanning(Vehicle &ego_vehicle, Trajectory &previous_path, vector<vector<double>> &sensor_fusion) {
            ego_vehicle_ = ego_vehicle;
            previous_path_ = previous_path;
            sensor_fusion_ = sensor_fusion;
        }

        // Destructor
        ~PathPlanning();

        Trajectory chooseNextState();
        vector<string> successorStates();
        Trajectory generateTrajectory(string state);
    
    private:
        Vehicle ego_vehicle_;
        Trajectory previous_path_;
        vector<vector<double>> sensor_fusion_;
   
};







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
  double cost_temp;
  vector<double> costs = {};



  for(int i=0; i<states.size(); i++) {
    Trajectory trajectory = generateTrajectory(states[i]);
    cost_temp = cost.calculateCost(0);
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

Trajectory PathPlanning::generateTrajectory(string state) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  Trajectory trajectory;
  if (state.compare("KL") == 0) {
      trajectory = keep_lane_trajectory();
    } 
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
      trajectory = lane_change_trajectory(state);
    } 
  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
      trajectory = prep_lane_change_trajectory(state);
    }

  return trajectory;
}

#endif  // VEHICLES_H