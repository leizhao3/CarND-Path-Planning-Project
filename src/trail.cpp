
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;



/**
 * convert MAP coord to vehicle coord
 * @param 
 * @return 
 */
void map2car(
  vector<double> &coord_map_x, vector<double> &coord_map_y,
  double ref_x, double ref_y, double ref_yaw) {
    for(int i=0; i<coord_map_x.size(); i++) {
      double shift_x = coord_map_x[i] - ref_x;
      double shift_y = coord_map_y[i] - ref_y;

      coord_map_x[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
      coord_map_y[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);

    }
  }



/**
 * convert car coord to MAP coord
 * @param 
 * @return 
 */
void car2map(
  vector<double> &coord_car_x, vector<double> &coord_car_y,
  double ref_x, double ref_y, double ref_yaw) {
    for(int i=0; i<coord_car_x.size(); i++) {
      double x_point = ref_x + coord_car_x[i]*cos(ref_yaw) - coord_car_y[i]*sin(ref_yaw);
      double y_point = ref_y + coord_car_x[i]*sin(ref_yaw) + coord_car_y[i]*cos(ref_yaw);

      coord_car_x[i] = x_point;
      coord_car_y[i] = y_point;


    }
  }


int main() {
    vector<double> map_x = {0.0, 3.2, 6.4, 10.0};
    vector<double> map_y = {0.0, 3.2, 6.4, 10.0};

    vector<double> car_x = {0};
    vector<double> car_y = {5.09};

    double ref_x = 3.2;
    double ref_y = 3.2;
    double ref_yaw = M_PI/4;

    map2car(map_x, map_y, ref_x, ref_y, ref_yaw);

    int width = 10;
    cout << "after map2car" << endl;
    cout << setw(width) << "x" << setw(width) << "y" << endl;
    for(int i=0; i<map_x.size(); i++) {
        cout << setw(width) << map_x[i] << setw(width) << map_y[i] << endl;
    }
    


}