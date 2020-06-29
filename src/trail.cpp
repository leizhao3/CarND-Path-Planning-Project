
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


class Print {    
  public:
    void output(double& trial) {
      cout << "inside class: trail = " << trial << endl;
      trial += 1;
      cout << "inside class: trail+1 = " << trial << endl;
    }
};


int main() {
  Print foo;
  double number = 1.2;

  cout << "outside class before = " << number << endl;

  foo.output(number);

  cout << "outside class after = " << number << endl;

  return 0;

}


          