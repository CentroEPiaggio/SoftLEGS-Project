/*
 * Copyright (C) 2015 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Alberto Cardellino, Houman Dallali, Timothée Habra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using namespace std;

void yarp_finish(void* p_robotranYarpInterface)
{
	// here should come termination of yarp
	// - closing port
	// - removing Robotran-Yarp drivers

    RobotranYarp_interface*   robotranYarpInterface = (RobotranYarp_interface*) p_robotranYarpInterface;
    yarp::dev::PolyDriverList *controlBoardList = (yarp::dev::PolyDriverList*)robotranYarpInterface->controlBoardList;  // convert back into object

    if(robotranYarpInterface->clockMaster)
    {
        yarp::os::Port *clock_p = (yarp::os::Port*) robotranYarpInterface->clockPort;
        clock_p->close();

        yarp::os::Time::useSystemClock();

        bool valid = yarp::os::Time::isValid();
    }


    if(controlBoardList != NULL)
    {
        std::cout <<" controlBoardList->size() is " <<  controlBoardList->size() << std::endl;
        for(int i=0; i < controlBoardList->size(); i++)
        {
            std::cout <<" closing device " << (*controlBoardList)[i]->key << std::endl;

            if((*controlBoardList)[i]->poly == NULL )
            {
                cout << "closing a NULL object" << endl;
            }
            else
                (*controlBoardList)[i]->poly->close();
        }
        delete controlBoardList;
    }

    if(robotranYarpInterface->yarpNetwork != NULL)
    {
        cout << "deleting yarpNetwork" << endl;
        yarp::os::Network* net = (yarp::os::Network*) robotranYarpInterface->yarpNetwork;
        net->fini();
        robotranYarpInterface->yarpNetwork = NULL;
    }
}

#endif
