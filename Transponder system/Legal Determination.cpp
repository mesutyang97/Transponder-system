//
//  Legal Determination.cpp
//  Transponder system
//
//  Created by Xiaocheng Yang on 9/17/15.
//  Copyright (c) 2015 Xiaocheng Yang. All rights reserved.
//

#include <iostream>
#include <math.h>

#include "Legal Determination.hpp"

using namespace std;

//Get Unit Vector from Angle Position
pair<double, double> VectorNormalization(double m1, double m2)
{
    double l = sqrt(m1 * m1 + m2 * m2);
    double w1 = m1 / l;
    double w2 = m2 / l;
    return make_pair(w1, w2);
}

//Project Vector V (v1, v2) on a line with unit vector (m1, m2)
double ProjTest_Vector (double v1, double v2, double m1, double m2)
{
    //Normalization First
    double l = sqrt(m1 * m1 + m2 * m2);
    double w1 = m1 / l;
    double w2 = m2 / l;
    
    return (v1 * w1 + v2 * w2);
    
}

//V stands for a radom vector. angPos is the angular position of the line
double ProjTest_Ang (double v1, double v2, double angPos)
{
    
    double w1 = cos(angPos);
    double w2 = sin(angPos);
    return (v1 * w1 + v2 * w2);
}

//Actication
double Get_l_Activation (double Car_i_w, double Car_i_l, double Car_j_w, double Car_j_l, double Car_i_o1, double Car_i_o2, double Car_j_o1, double Car_j_o2)
{
    // longest distance that two cars can possibly touch each other
    double l_i_j = (1/2)*sqrt((Car_i_w+Car_j_w)*(Car_i_w+Car_j_w)+(Car_i_l+Car_j_l)*(Car_i_l+Car_j_l));
    // !![Default setting] Activation constant: how many times of l_i_j is needed
    const double k_Act = 1.5;
    return k_Act * l_i_j;
}


//HAS OUTPUT
int AlertP_Activation_test (char i, char j, double Car_i_o1, double Car_i_o2, double Car_j_o1, double Car_j_o2, double l_activation)
{
    double l_i_j_x = Car_j_o1-Car_i_o1;
    double l_i_j_y = Car_j_o2-Car_i_o2;
    double l_i_j_t = sqrt(l_i_j_x*l_i_j_x + l_i_j_y*l_i_j_y);
    
    if (l_i_j_t > l_activation)
    {
        cout << "AlertP test not needed. Everything good with " << i << " and " << j << " car." << endl; //[Eliminate] once tested
        return 0;
    }
    else
    {
        cout << "Car " << j << " looks dangerous to car" << i << endl;
        return 1; //AlertP test not needed.
    }
}

//DETERMINE WHICH ALERT POINT TO USE!! (G on the right OR H on the left)
//Fabricate a vector N (n1, n2) whose angular position is (-PI/2 = -1.57079632679) to the axial vector of the car ----> N's angular position is angPos_i - 1.57079632679

int AlertP_Det(double angPos_i, double l_i_j_x, double l_i_j_y)
{
    /*double m1 = cos(angPos);
    double m2 = sin(angPos);
    double n1 = m1 * cos(-1.57079632679) - m2 * sin(-1.57079632679); // x of perpendicular vector
    double n2 = m1 * sin(-1.57079632679) + m2 * cos(-1.57079632679); // y of perpendicular vector*/
    double perAngPos_N = angPos_i - 1.57079632679;
    double w1 = cos(perAngPos_N);
    double w2 = sin(perAngPos_N);
    double m_Proj = (l_i_j_x * w1 + l_i_j_x * w2);//magnitude of projection
    
    if (m_Proj <0)
    {
        cout << "Point G is Alert Point" << endl;// [Eliminate] once tested
        return -1;
    }
    
    else
    {
        cout << "Point H is Alert Point" <<endl;
        return 1;
    }
}



//Calculate Position of Alert Point (A) at runtime t
//OA stands for the vector connecting center of mass O to Alert Point (A)
pair<double, double> GetAlertP_t(double o1, double o2, double OA_x, double OA_y, double angPos)
{
    double OA_x_t = OA_x * cos(angPos) - OA_y * sin(angPos); // x componant
    double OA_y_t = OA_x * sin(angPos) + OA_y * cos(angPos); // y componant
    
    double AlertP_x = o1 + OA_x_t; // x componant of point Z at time t
    double AlertP_y = o2 + OA_y_t; // y componant of point Z at time t
    return make_pair(AlertP_x, AlertP_y);
}


//Calculate Position of Coner Point (C) at runtime t
pair<double, double> GetCorner_t(double o1, double o2, double OC_x, double OC_y, double angPos)
{
    double OC_x_t = OC_x * cos(angPos) - OC_y * sin(angPos); // x componant
    double OC_y_t = OC_x * sin(angPos) + OC_y * cos(angPos); // y componant
    
    double C_i_x = o1 + OC_x_t; // x componant of point Z at time t
    double C_i_y = o2 + OC_y_t; // y componant of point Z at time t
    return make_pair(C_i_x, C_i_y);
}




//Vector Connections from Alerting Point (AlertP) to one of four corners of the rear car (Corner_i)
// This should be run four times in total)

pair<double, double> GetVector_AC (double AlertP_x, double AlertP_y, double Corner_i_x, double Corner_i_y)
{
    return make_pair(Corner_i_x-AlertP_x, Corner_i_y-AlertP_y);
}

//The Porjection Test
int Proj_Test (double angPos, double AlertP_x, double AlertP_y, double C_1_x, double C_1_y, double C_2_x, double C_2_y, double C_3_x, double C_3_y, double C_4_x, double C_4_y)
{

    pair<double, double> AC_1 = GetVector_AC(AlertP_x, AlertP_y, C_1_x, C_1_y);
    double k_1 = ProjTest_Ang(AC_1.first, AC_1.second, angPos);
    pair<double, double> AC_2 = GetVector_AC(AlertP_x, AlertP_y, C_2_x, C_2_y);
    double k_2 = ProjTest_Ang(AC_2.first, AC_2.second, angPos);
    pair<double, double> AC_3 = GetVector_AC(AlertP_x, AlertP_y, C_3_x, C_3_y);
    double k_3 = ProjTest_Ang(AC_3.first, AC_3.second, angPos);
    pair<double, double> AC_4 = GetVector_AC(AlertP_x, AlertP_y, C_4_x, C_4_y);
    double k_4 = ProjTest_Ang(AC_4.first, AC_4.second, angPos);
    
    int sum = 0;
    double k_i;
    for (int i=1; i<=4; i++)
    {
        if (k_i <0)
        {
            sum ++;
        }
    }
    cout << "The number of points falling behind: " << sum << endl;
    
    if (sum < 2)
    {
        cout << "Pass Projection Test. The rear car is far enough into the corner." << endl;
        return 1;
    }
    else // More than 2 points are behind == illeagal
    {
        cout << "Fail Projection Test. The car trying to overtake was a bad boy. He was not into the corner far enough to claim the spot." << endl;
        return -1;
    }
}




