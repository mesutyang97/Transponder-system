//
//  Pre-Loop Car Determination.cpp
//  Transponder system
//
//  Created by Xiaocheng Yang on 9/25/15.
//  Copyright (c) 2015 Xiaocheng Yang. All rights reserved.
//

#include "Pre-Loop Car Determination.hpp"
#include "Car.hpp"
#include <math.h>
#include <string>

double GetTheta(double a1, double a2, double b1, double b2)
{
    return atan2(b2-a2, b1-a1);
}

double d_Theta(double thetaInitial, double thetaFinal)
{
    return thetaFinal - thetaInitial;// The difference in angle position CCW
}


// Point Z is a sample point. OZ is the vector pointing from O (center of mass) to Z (the disired point
pair<double, double> Rotation(double o1, double o2, double x_OZ, double y_OZ, double d_theta)
{
    double x_OZ_t = x_OZ * cos(d_theta) - y_OZ * sin(d_theta); // x componant
    double y_OZ_t = x_OZ * sin(d_theta) + y_OZ * cos(d_theta); // y componant
    
    double z1 = o1 + x_OZ_t; // x componant of point Z at time t
    double z2 = o2 + y_OZ_t; // y componant of point Z at time t
    return make_pair(z1, z2);
}


//CM = Center of Mass
pair<double, double> GetCarCMPosition(double a1, double a2, double a1_t, double a2_t, double d_theta)
{
    double x_OA = a1 * cos(d_theta) - a2 * sin(d_theta); // x componant of vector pointing from O to A
    double y_OA = a1 * sin(d_theta) + a2 * cos(d_theta); // y componant of vector pointing from O to A
    double x_O = a1_t - x_OA; // x componant of position of car (at point O)
    double y_O = a2_t - y_OA; // y componant of position of car (at point O)
    
    return make_pair(x_O, y_O);
}

double GetSpeed (double a1, double a2, double a1_t, double a2_t, double diffT)
{
    return sqrt((a1_t - a1)*(a1_t - a1) + (a2_t - a2) * (a2_t - a2))/diffT;
}

double GetSlope (double x, double y, double x_t, double y_t)
{
    return ((y_t-y)/(x_t-x));
}


// Functions to get the slope of a lineperpendicular
double GetPerSlope (double x, double y, double x_t, double y_t)
{
    return ((x-x_t)/(y_t-y));
}

// Functions that return x position of rotation center
pair<double, double> GetTurningCenterPosition (double a1, double a2, double b1, double b2, double m_a_per, double m_b_per)
{
    
    /* The following three lines are just demonstration of how I get the idea. NEVER EXCECUTE THEM!!!
     y_a_per = m_a_per * x - m_a_per * a1 + a2
     y_b_per = m_b_per * x - m_b_per * b1 + b2
     set y_a_per and y_b_per equal to find the intersection point, which is the Turning Center*/
    double p1 = (m_a_per * a1 - a2 - m_b_per * b1 + b2)/(m_a_per - m_b_per);
    double p2 = m_a_per * p1 - m_a_per * a1 + a2;
    return make_pair(p1, p2);
}

double GetAngularSpeed (double speed, double a1, double a2, double p1, double p2)
{
    double l_PA = sqrt((a1 - p1)*(a1 - p1) + (a2 - p2) * (a2 - p2));
    return (speed/l_PA);
}

double GetTurningRadius (double o1, double o2, double p1, double p2)
{
    return sqrt((o1 - p1)*(o1 - p1) + (o2 - p2) * (o2 - p2));
}

double GetSpeedOfCM (double l_PO, double Omiga)
{
    return l_PO * Omiga; // v = r * omiga
}

pair<double, double> GetVelocityVector (double o1, double o2, double p1, double p2, double l_PO, double v_CM)
{
    /*This comment is just a reminder of how I got the result, NEVER EXCECUTE!!!
     m0_per = (o2-p2)/(o1-p1)
     m0 = -1 / (m0_per) = -(o1-p1)/(o2-p2)
     v_y : v_x = (p1-o1) : (o2-p2)*/
    double v2 = v_CM * (p1-o1) / l_PO;
    double v1 = v_CM * (o2-p2) / l_PO;
    return make_pair(v1, v2);
}



void PreLoopDataRun (Car Car_i)
{
    double diff_T = 0.03;
    
    Car_i.a1_t1 = 3;
    Car_i.a2_t1 = 2;
    Car_i.b1_t1 = 1;
    Car_i.b2_t1 = -1;
    
    Car_i.a1_t2 = 3.08;
    Car_i.a2_t2 = 1.95;
    Car_i.b1_t2 = 1.04;
    Car_i.b2_t2 = Car_i.a2_t2 - sqrt(13 - (Car_i.b1_t2-Car_i.a1_t2)*(Car_i.b1_t2-Car_i.a1_t2));
    
    
    
    cout << "Theta1 is: " << Car_i.theta1 << "\n";
    
    double theta2 = GetTheta(Car_i.a1_t2, Car_i.a2_t2, Car_i.b1_t2, Car_i.b2_t2);
    cout << "Theta2 is: " << theta2 << "\n";
    
    Car_i.angPos = d_Theta(Car_i.theta1, theta2);
    cout << "d_Theta is: " << Car_i.angPos << "\n";
    
    pair <double,double> CMPosition = GetCarCMPosition(Car_i.a1_t1, Car_i.a2_t1, Car_i.a1_t2, Car_i.a2_t2, Car_i.angPos);
    cout << "X cordinate of CM is: " << CMPosition.first << "\n";
    cout << "Y cordinate of CM is :" << CMPosition.second << " \n";
    
    double v_a = GetSpeed(Car_i.a1_t1, Car_i.a2_t1, Car_i.a1_t2, Car_i.a2_t2, diff_T);
    cout << "Speed of point A is " << v_a << " unit per second. \n";
    
    double v_b = GetSpeed(Car_i.b1_t1, Car_i.b2_t1, Car_i.b1_t2, Car_i.b2_t2, diff_T);
    cout << "Speed of point B is " << v_b << " unit per second. \n";
    
    double m_a_per = GetPerSlope(Car_i.a1_t1, Car_i.a2_t1, Car_i.a1_t2, Car_i.a2_t2);
    double m_b_per = GetPerSlope(Car_i.b1_t1, Car_i.b2_t1, Car_i.b1_t2, Car_i.b2_t2);
    
    pair<double, double> TurnCPosition = GetTurningCenterPosition (Car_i.a1_t1, Car_i.a2_t1, Car_i.b1_t1, Car_i.b2_t1, m_a_per, m_b_per);// TurnC = Turning Center
    cout << "X cordinate of Turning Center is: " << TurnCPosition.first << "\n";
    cout << "Y cordinate of Turning Center is :" << TurnCPosition.second << " \n";
    
    double OmigaA = GetAngularSpeed (v_a, Car_i.a1_t1, Car_i.a2_t1, TurnCPosition.first, TurnCPosition.second);
    cout << "Angular speed according to A is " << OmigaA << " Radian per seconds. \n";
    
    double OmigaB = GetAngularSpeed (v_b, Car_i.b1_t1, Car_i.b2_t1, TurnCPosition.first, TurnCPosition.second);
    cout << "Angular speed according to B is " << OmigaB << " Radian per seconds. \n";
    
    Car_i.turningR = GetTurningRadius(CMPosition.first, CMPosition.second, TurnCPosition.first, TurnCPosition.second);
    cout << "The turning radius of "  << Car_i.carName << " is " << Car_i.turningR << " units. \n";
    
    Car_i.v_CM = GetSpeedOfCM(Car_i.turningR, OmigaA);
    cout << "The speed of " << Car_i.carName <<" is " << Car_i.v_CM << " units per second. \n";
    
    Car_i.CMVelocity = GetVelocityVector(CMPosition.first, CMPosition.second, TurnCPosition.first, TurnCPosition.second, Car_i.turningR, Car_i.v_CM);
    cout << "CM Velocity vector is (" << Car_i.CMVelocity.first << "," << Car_i.CMVelocity.second << ") \n";
    
    
    
    pair <double, double> HpointPosition = Rotation(CMPosition.first, CMPosition.second, Car_i.h1, Car_i.h2, Car_i.angPos);
    cout << "H point Position is (" << HpointPosition.first << "," << HpointPosition.second << ") \n";
    
}



