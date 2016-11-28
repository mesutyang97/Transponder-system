//
//  Legal Determination.hpp
//  Transponder system
//
//  Created by Xiaocheng Yang on 9/26/15.
//  Copyright (c) 2015 Xiaocheng Yang. All rights reserved.
//

#ifndef Transponder_system_Legal_Determination_hpp
#define Transponder_system_Legal_Determination_hpp

#include <iostream>
using namespace std;

pair<double, double> VectorNormalization(double m1, double m2);

double ProjTest_Vector (double v1, double v2, double m1, double m2);

double ProjTest_Ang (double v1, double v2, double angPos);

double Get_l_Activation (double Car_i_w, double Car_i_l, double Car_j_w, double Car_j_l, double Car_i_o1, double Car_i_o2, double Car_j_o1, double Car_j_o2);

int AlertP_Activation_test (char i, char j, double Car_i_o1, double Car_i_o2, double Car_j_o1, double Car_j_o2, double l_activation);

int AlertP_Det(double angPos_i, double l_i_j_x, double l_i_j_y);

pair<double, double> GetAlertP_t(double o1, double o2, double OA_x, double OA_y, double angPos);

pair<double, double> GetCorner_t(double o1, double o2, double OC_x, double OC_y, double angPos);

pair<double, double> GetVector_AC (double AlertP_x, double AlertP_y, double Corner_i_x, double Corner_i_y);

int Proj_Test (double angPos, double AlertP_x, double AlertP_y, double C_1_x, double C_1_y, double C_2_x, double C_2_y, double C_3_x, double C_3_y, double C_4_x, double C_4_y);



#endif
