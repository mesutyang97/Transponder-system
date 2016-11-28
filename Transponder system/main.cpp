//
//  main.cpp
//  Transponder system
//
//  Created by Xiaocheng Yang on 8/27/15.
//  Copyright (c) 2015 Xiaocheng Yang. All rights reserved.
//

// this demonstrates returning two values from a function
//

//prefix of length of a segment is "l_"


#include <iostream>
#include <map>
#include <math.h>

#include "Pre-Loop Car Determination.hpp"

using namespace std;



int main()
{
    Car Car_1(1);
    
    PreLoopDataRun(Car_1);
    
    cout << Car_1.carName<< endl;
    return 0;
    
}
