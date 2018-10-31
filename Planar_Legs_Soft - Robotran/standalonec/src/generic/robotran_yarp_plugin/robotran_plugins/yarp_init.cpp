/*
 * Copyright (C) 2015 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Alberto Cardellino, Houman Dallali, Timothée Habra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriverList.h>

#include <iostream>

using namespace std;

// here should come initialization of yarp
// - getting configuration files
// - creating desired Robotran-Yarp drivers
// - opening ports



bool initYarpClock(void *p_robotranYarpInterface)
{
    RobotranYarp_interface*   robotranYarpInterface = (RobotranYarp_interface*) p_robotranYarpInterface;
    yarp::os::Port *clock = (yarp::os::Port *) robotranYarpInterface->clockPort;
    clock = new yarp::os::Port;

    robotranYarpInterface->clockPort = clock;

    std::cout << "initting clock " << std::endl;
    //     yarp::os::Time::useSystemClock();
    if(!clock->open("/clock") )
    std::cout << "failed opening clock port" << std::endl;

//    yarp::os::Time::useNetworkClock("/clock");
     return true;
}

bool yarp_init(void *p_robotranYarpInterface)
{
    RobotranYarp_interface* robotranYarpInterface = (RobotranYarp_interface*) p_robotranYarpInterface;
    robotranYarpInterface->clockMaster = false;

	cout << "initialization of yarp interface" << endl;

    yarp::dev::PolyDriverList       *p_controlBoardList = NULL;
    bool verbose = true;

    yarp::os::Network *net = new yarp::os::Network;
    if(! net)
    {
        std::cerr << "Error initting YARP network" << std::endl;
        return false;
    }

    // init YARP and instantiate yarp device driver
    if( ! net->checkNetwork() ) {
        std::cerr << "RobotranYarpControlBoard::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        // either return something invalid (NULL) and check the value in the main_simulation or directly throw an exit here.
        exit(0);
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);

    yarp::os::ConstString fileNameWithPath = rf.findFileByName("RoboTran.ini");
    if(fileNameWithPath == "")
    {
        std::cout << "Default config file ´RoboTran.ini´  was not found" << std::endl;
        return false;
    }

    p_controlBoardList = new yarp::dev::PolyDriverList;

    yarp::os::Property p;
    p.fromConfigFile(fileNameWithPath);

    if(verbose)
        std::cout << "\n\n config param are\n " << p.toString() << std::endl;

    bool found = p.check("GENERAL");
    if(!found)
    {
        std::cout << "GENERAL section not found" << std::endl;
        robotranYarpInterface->controlBoardList = NULL;
        robotranYarpInterface->yarpNetwork = NULL;
        net->fini();
        return false;
    }

    yarp::os::Bottle &general = p.findGroup("GENERAL");

    if(general.check("verbose"))
        verbose = true;

    if(!general.check("clock") )
    {
        std::cout << "Not generating yarp clock" << std::endl;
        robotranYarpInterface->clockMaster = false;
    }
    else
    {
        if(general.find("clock").asBool() )
        {
            std::cout << "Robotran is generating yarp clock" << std::endl;
            yarp::os::Time::useSystemClock();
            initYarpClock(p_robotranYarpInterface);
            robotranYarpInterface->clockMaster = true;
        }
    }

    yarp::os::ConstString robotName = general.find("robot").asString();
    if(!general.check("types"))
    {
        std::cout << "ERROR: ´types´ list was not found in the GENERAL group" << std::endl;
        robotranYarpInterface->controlBoardList = NULL;
        robotranYarpInterface->yarpNetwork = NULL;
        net->fini();
        return false;
    }

    yarp::os::Bottle * types = general.find("types").asList();
    if(verbose)
        std::cout << "\nFound following types (" << types->toString() << ")" << std::endl;

    for(int typeIndex=0; typeIndex < types->size(); typeIndex++)
    {
        yarp::os::ConstString typeName = types->get(typeIndex).asString();
        if(!general.check(typeName))
        {
            std::cout << "ERROR: I was expecting the keyword " << typeName << " followed by a list of entries like" << std::endl;
            std::cout << typeName << " (foo1 foo2 foo3)" << std::endl;
            robotranYarpInterface->controlBoardList = NULL;
            robotranYarpInterface->yarpNetwork = NULL;
            net->fini();
            return false;
        }

        if(!general.find(typeName).isList() )
        {
            std::cout << "ERROR: the keyword " << typeName << " is not a list. Correct syntax is like" << std::endl;
            std::cout << typeName << " (foo1 foo2 foo3)  maybe the ´()´ brackets are missing" << std::endl;
            robotranYarpInterface->controlBoardList = NULL;
            robotranYarpInterface->yarpNetwork = NULL;
            net->fini();
            return false;
        }

        yarp::os::Bottle * entryList = general.find(typeName).asList();


        if(verbose)
            std::cout << "type ´" << typeName << "´ has the following entries (" << entryList->toString() << ")" << std::endl;

        for(int entryIndex=0; entryIndex < entryList->size(); entryIndex++)
        {
            yarp::os::ConstString entryName = entryList->get(entryIndex).asString();
            if(!p.check(entryName))
            {
                std::cout << "cannot find device ´" << entryName << "´ referenced in the type list ´" << typeName << "´" << std::endl;
                robotranYarpInterface->controlBoardList = NULL;
                robotranYarpInterface->yarpNetwork = NULL;
                net->fini();
                return false;
            }

            yarp::os::Bottle &entryParams = p.findGroup(entryName);
            if(verbose)
                std::cout << "Entry ´" << entryName << "´ has the following parameters \n\t" << entryParams.toString() << "\n" << std::endl;

            // Looking for other files referenced by the main one

            yarp::os::Property tmpProp(entryParams.toString().c_str());

            while(tmpProp.check("file"))
            {
                yarp::os::ResourceFinder subRF;
                subRF.setVerbose(false);
                yarp::os::ConstString  subfileName = tmpProp.find("file").asString();
                if(verbose)
                    std::cout << "found subfile " << subfileName << std::endl;

                yarp::os::ConstString fileNameWithPath = subRF.findFileByName(subfileName);
                if(fileNameWithPath == "")
                {
                    std::cout << "sub config file ´" << subfileName << "´  was not found" << std::endl;
                    robotranYarpInterface->controlBoardList = NULL;
                    robotranYarpInterface->yarpNetwork = NULL;
                    net->fini();
                    return false;
                }
                yarp::os::Property p2;
                p2.fromConfigFile(fileNameWithPath);

                tmpProp.unput("file");
                tmpProp.fromString(p2.toString(), false);
            }
            yarp::dev::PolyDriver *tmp = new yarp::dev::PolyDriver;
            tmpProp.put("robot", robotName);
            tmp->open(tmpProp);


            if (!tmp->isValid())
            {
                fprintf(stderr, "driver %s did not open\n", entryName.c_str());
            }
            else
            {
                printf("controlBoard opened correctly\n");

                yarp::dev::PolyDriverDescriptor newDev( tmp, entryName.c_str());
                p_controlBoardList->push(newDev);
            }
        }
    }


    robotranYarpInterface->controlBoardList = p_controlBoardList;
    robotranYarpInterface->yarpNetwork = (void *) net;
    return true;
}

#endif
