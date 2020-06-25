
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
  vector<vector<double>> pred;
  vector<vector<double>> sensor_fusion{ {20,10,4,5,100,2},
                                        {90,70,8,6,60,6} }; 

  

  for(int i=0; i<sensor_fusion.size(); i++) {
    cout << "ID = " << i << endl;
    for(int j=0; j<sensor_fusion[i].size(); j++) {
      cout << sensor_fusion[i][j] << " ";
    }
    cout << endl;
  }

  pred = prediction(sensor_fusion);

  for(int i=0; i<pred.size(); i++) {
    cout << "ID = " << i << endl;
    cout << "[s, s_dot, s_double_dot, d, d_dot, d_double_dot] = ";
    for(int j=0; j<pred[i].size(); j++) {
      cout << pred[i][j] << " ";
    }
    cout << endl;
  }


  return 0;

}


          