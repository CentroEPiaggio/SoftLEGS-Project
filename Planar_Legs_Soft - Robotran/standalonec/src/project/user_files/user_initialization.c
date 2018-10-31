//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2008 
// Last update : 24/10/2008 
//--------------------------- 

#include "MBSdataStruct.h" 
#include "simu_def.h"
#include "gcm_interface.h"
#include "contact_interface.h"

#define FORCE_VEC_LENGTH 50

// User parameters initialization
#ifndef STANDALONE
void user_initialization(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds)
#else
// Returns 0 if no problem
int user_initialization(MBSdataStruct *MBSdata, LocalDataStruct *lds)
#endif
{
    // init mesh GCM (Ground Contact Model)
    #ifdef MESH_GSM
    init_mesh_gcm(MBSdata);
    #endif

    // init 3D contact
    init_contact_geom(MBSdata);

    // init actuator models
    init_actuator_model(MBSdata);

    // init low level joints PID controller
    init_joints_control(MBSdata);

    // init sensor (IMU,...)
    init_sensor_model(MBSdata);

	// controller initialization
    //controller_init_interface(MBSdata);

    // saving a suitable pelvis orientation in Floating Base state vector:
    init_floatingBase(MBSdata);


    #ifdef STANDALONE
    return 0;
    #endif
}
