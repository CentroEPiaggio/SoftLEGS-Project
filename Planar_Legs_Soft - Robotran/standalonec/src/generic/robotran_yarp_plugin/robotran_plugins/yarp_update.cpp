/*
 * Copyright (C) 2015 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Alberto Cardellino, Houman Dallali, Timothée Habra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <yarp/os/Time.h>

#include <iostream>

using namespace std;

void updateDataFromYarp(void* p_robotranYarpInterface, MBSdataStruct * MBSdata)
{
//     cout << "updateDataFromYarp" << endl;

	// here should come the update from yarp to the simulator
	// write the new references (torque, position, speed ...) desired by the ControlInterface
	// - get references
	// - feed low leved PID controllers with the ref

	//cout << "update data from yarp " << endl;
    RobotranYarp_interface* robotranYarpInterface = (RobotranYarp_interface*)    p_robotranYarpInterface;
    yarp::dev::PolyDriverList *controlBoardList   = (yarp::dev::PolyDriverList*) robotranYarpInterface->controlBoardList;  // convert back into object
    robotran::IRobotran       *RobotranPlugin          	= NULL;


    if(controlBoardList == NULL)
        return;

    for(int i=0; i < controlBoardList->size(); i++)
    {
        (*controlBoardList)[i]->poly->view(RobotranPlugin);
        if(RobotranPlugin)
        {
//            printf("I found a valid plugin at %d \n\n", i);
            RobotranPlugin->updateFromYarp(MBSdata);
        }

        // for controller too?
    }
}

// here should come the update from simulator to yarp
// write the new sensor state computed by simulation
// - sensor_driver.Update()
void updateDataToYarp(void* p_robotranYarpInterface, const MBSdataStruct * MBSdata)
{
//     cout << "updateDataToYarp" << endl;

    RobotranYarp_interface*   robotranYarpInterface = (RobotranYarp_interface*) p_robotranYarpInterface;
    yarp::dev::PolyDriverList *controlBoardList = (yarp::dev::PolyDriverList*)robotranYarpInterface->controlBoardList;  // convert back into object
    robotran::IRobotran *RobotranPlugin = NULL;
    yarp::os::Port * clock;
    clock = (yarp::os::Port *) robotranYarpInterface->clockPort;
    static bool initted = false;

    if(robotranYarpInterface->clockMaster)
    {
        if(!initted)
        {

            yarp::os::Time::useNetworkClock("/clock");
            yarp::os::Time::isValid();
            initted = true;
        }

        double clockTime_secs;
        double clockTime = MBSdata->tsim;
        double clockTime_nsecs = modf (clockTime , &clockTime_secs);
        clockTime_nsecs = 1000000000*clockTime_nsecs; //export in nano second

        // publish the data USING ROBOTRAN THREAD!!
        // if we use a yarp thread here it'll get stuck because it'll wait for
        // the clock, but I'm the clock producer so it'll hang indefinitely
        yarp::os::Bottle b;
        b.clear();
        b.addInt((int) clockTime_secs);
        b.addInt((int) clockTime_nsecs);
        clock->write(b);
    }

    if(controlBoardList == NULL)
        return;

	for(int i=0; i < controlBoardList->size(); i++)
    {
        (*controlBoardList)[i]->poly->view(RobotranPlugin);
        if(RobotranPlugin)
        {
//            printf("I found a valid plugin at %d \n\n", i);
            RobotranPlugin->updateToYarp(MBSdata);
        }
    }
}

#endif
