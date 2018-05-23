// Author: Aliakbar Toghyan

#define _USE_MATH_DEFINES
#include "Car.h"
#include <iostream>
#include <vector>
#include "PathPlanner.h"
#include <math.h>

int main(int argc, char const *argv[])
{
	Car start(0,1,0);
    Car goal(0,3,M_PI);
	PathPlanner plan(start,goal, 1, 1);
    string command = "";
    vector<double> path = plan.find_optimal_path(command);
    cout << command <<endl;
    cout << path[0] << " , " << path[1] << " , " << path[2] << endl;
	return 0;
}
