//
//  Pre-Loop Car Determination.hpp
//  Transponder system
//
//  Created by Xiaocheng Yang on 9/25/15.
//  Copyright (c) 2015 Xiaocheng Yang. All rights reserved.
//

#ifndef __Transponder_system__Pre_Loop_Car_Determination__
#define __Transponder_system__Pre_Loop_Car_Determination__

#include <iostream>
#include "Car.hpp"
using namespace std;

double GetTheta(double a1, double a2, double b1, double b2);

double d_Theta(double thetaInitial, double thetaFinal);

pair<double, double> Rotation(double o1, double o2, double x_OZ, double y_OZ, double d_theta);




pair<double, double> GetCarCMPosition(double a1, double a2, double a1_t, double a2_t, double d_theta);

double GetSpeed (double a1, double a2, double a1_t, double a2_t, double diffT);

double GetSlope (double x, double y, double x_t, double y_t);
double GetPerSlope (double x, double y, double x_t, double y_t);

pair<double, double> GetTurningCenterPosition (double a1, double a2, double b1, double b2, double m_a_per, double m_b_per);

double GetAngularSpeed (double speed, double a1, double a2, double p1, double p2);

double GetTurningRadius (double o1, double o2, double p1, double p2);

double GetSpeedOfCM (double l_PO, double Omiga);

pair<double, double> GetVelocityVector (double o1, double o2, double p1, double p2, double l_PO, double v_CM);

void PreLoopDataRun (Car Car_i);

#endif /* defined(__Transponder_system__Pre_Loop_Car_Determination__) */
