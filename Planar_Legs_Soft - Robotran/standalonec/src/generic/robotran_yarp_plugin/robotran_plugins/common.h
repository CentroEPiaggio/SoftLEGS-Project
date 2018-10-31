/*
 * Copyright (C) 2015 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Alberto Cardellino, Houman Dallali, Timothée Habra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef ROBOTRANYARP_COMMON_H
#define ROBOTRANYARP_COMMON_H

#include <string>
#include <cmath>
#include "MBSdataStruct.h"


double const pi = 3.1415926535897932384626433;
/**
* \brief convert from degrees to radians
* \param degrees angle in degrees
* \return the angle converted in radians
*/
double convertDegreesToRadians(double degrees);
/**
* \brief convert from radians to degrees
* \param radians angle in radians
* \return the angle converted in degrees
*/
double convertRadiansToDegrees(double radians);

inline double convertDegreesToRadians(double degrees) { return degrees / 180.0 * pi; }
inline double convertRadiansToDegrees(double radians) { return radians * 180.0 / pi; }


namespace robotran {
//     class RobotranYarpForceTorque;
    class IRobotran;
}

class robotran::IRobotran
{
public:
    // interface with Robotran
    virtual void updateToYarp(const MBSdataStruct * MBSdata) = 0;
    virtual void updateFromYarp(MBSdataStruct *MBSdata) = 0;
};

#endif
