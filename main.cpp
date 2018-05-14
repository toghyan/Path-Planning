// Author: Aliakbar Toghyan

#include "Car.h"
// #include "Circle.h"
#include <iostream>

int main(int argc, char const *argv[])
{
	Car myCar(1,2,0);
	cout << myCar << endl;
    Car yourCar = myCar;
    cout << yourCar.get_x() << endl;
	return 0;
}
