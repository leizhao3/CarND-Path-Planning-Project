
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include "helpers.h"
#include "cost_functions.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;


int main() {
    vector<double> next_path_x_ref = {0.0, 3.2, 6.4, 10.0};
    vector<double> next_path_y_ref = {0.0, 3.2, 6.4, 10.0};

    vector<double> car_x = {0};
    vector<double> car_y = {5.09};

    double ref_x = 3.2;
    double ref_y = 3.2;
    double ref_yaw = M_PI/4;

    int lane = 0;
    vector<int> potential_lane;
    potential_lane = find_lane(lane);

    cout << "potential_lane = " << endl;
    for(int i=0; i<potential_lane.size(); i++) {
      cout << potential_lane[i] << " " << endl;
    }


    double cost = calculate_cost(next_path_x_ref, next_path_y_ref);
    cout << "cost = " << cost << endl;
    


}


          