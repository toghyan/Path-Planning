#ifndef CIRCLE_H
#define CIRCLE_H

#include <iostream>
#include <string>
#include <math.h>
#include "Car.h"

using namespace std;

class Circle
{
public:

	// Default constructor, sets the center of the circle at the origin, assumes a right circle, and turning radius of 1.  
	Circle();	

	//  Constructor using class Car for kinematics of the car, minimum turning radius, and wether if the circle is a right or left circle
	Circle(Car new_car, double new_r_min, bool new_is_right = true);	

	// Deconstructor	
	~Circle();

	// Returns the x coordinate of the center of the circle
	double get_xc() const;

	// Returns the x coordinate of the center of the circle
	double get_yc() const;
    
    // Returns the minimum turning radius
    double get_r_min() const;

	// Returns if the circle is a right or left circle
	bool is_right() const;
    
    // cout operator (print)
	friend ostream& operator <<(ostream& out, const Circle & C1);


private:

	double x_center, y_center, r_min; 
	bool right;
    Car car;


	
};

#endif // CIRCLE_H
