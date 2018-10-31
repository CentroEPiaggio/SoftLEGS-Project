//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2008
// Last update : 10/02/2008
//---------------------------

#include "MBSdataStruct.h"
#include "simu_def.h"
#include "gcm_interface.h"
#include "contact_interface.h"

// Equivalent to user_DirDyn_io from Matlab
// Computes the voltage to apply on the motors
// uvs->Control[i] is the equivalent of MBS_user.u(i) in Matlab
#ifndef STANDALONE
void user_compute_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds)
#else
void user_compute_output(MBSdataStruct *MBSdata, LocalDataStruct *lds)
#endif
{
    // variables declaration
    double tsim;
    UserIOStruct     *uvs;

    // variables initialization
    tsim = MBSdata->tsim;
    uvs = MBSdata->user_IO;

    // GCM
    #ifdef MESH_GSM
    gcm_mesh_state_compute(MBSdata);
    #endif

    // 3D contact
    user_state_contact_geom(MBSdata);

    // update sensor data (IMU,...)
    update_sensor_model(MBSdata);

    // restart if needed
    if (uvs->restart)
    {
        printf("Resetting the floating base ... \n");
    }
    // controller called every milli-second (PERIOD_CTRL)
    if (tsim >= uvs->last_t_ctrl + PERIOD_CTRL - TIME_EPSILON)
    {
        joints_control(MBSdata);

        uvs->last_t_ctrl = tsim;
    }
}

