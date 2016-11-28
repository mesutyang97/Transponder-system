//
//  Car.hpp
//  Transponder system
//
//  Created by Xiaocheng Yang on 9/27/15.
//  Copyright (c) 2015 Xiaocheng Yang. All rights reserved.
//

#ifndef Transponder_system_Car_hpp
#define Transponder_system_Car_hpp

#include <utility>      // std::pair
#include <iostream>     // std::cout
#include <math.h>
#include <string>

class Car {
public:
    Car (int carNumber);
    ~Car();
    int i; //Car number
    std::string carName = "Car_" + std::to_string(i);
    double a1 = 3;//!! need to set to private
    double a2 = 2;
    double b1 = 1;
    double b2 = -1;
    
    double theta1 = atan2(b2-a2, b1-a1);
    
    

    
    double angPos; //Angular position of the car facing forward
    
    double d_F = 6;
    double d_B = 8;
    double d_R = 5;
    double d_L = 5.2;
    
    double c1 = d_F;
    double c2 = d_L;
    double d1 = d_F;
    double d2 = -d_R; //Right is negative on y direction
    double e1 = -d_B; //Back is negative on x direction
    double e2 = -d_R;
    double f1 = -d_B;
    double f2 = d_L;
    
    double h1 = 0;
    double h2 = -d_L;
    double g1 = 0;
    double g2 = d_R;
    
    double a1_t1;
    double a2_t1;
    double b1_t1;
    double b2_t1;
    
    double a1_t2;
    double a2_t2;
    double b1_t2;
    double b2_t2;
    

    double turningR;//Turning Radius
    double v_CM;
    std::pair<double, double> CMVelocity;
    
    
private:
};


#endif
