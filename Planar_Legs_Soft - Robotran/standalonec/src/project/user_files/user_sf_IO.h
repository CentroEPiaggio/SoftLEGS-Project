/*===========================================================================*
 *
 *  user_sf_IO.h
 * 
 *  Generation date: Sat Nov  7 15:01:04 2015

 * 
 *  (c) Universite catholique de Louvain
 *      Departement de Mecanique 
 *      Unite de Production Mecanique et Machines 
 *      2, Place du Levant 
 *      1348 Louvain-la-Neuve 
 *  http://www.robotran.be// 
 *  
/*===========================================================================*/

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/
 
#ifdef ACCELRED 
#define S_FUNCTION_NAME  mbs_sf_accelred_walkman_robotran 
#elif defined DIRDYNARED 
#define S_FUNCTION_NAME  mbs_sf_dirdynared_walkman_robotran 
#elif defined INVDYNARED 
#define S_FUNCTION_NAME  mbs_sf_invdynared_walkman_robotran 
#elif defined SENSORKIN 
#define S_FUNCTION_NAME  mbs_sf_sensorkin_walkman_robotran 
#endif 
 
#define SF_N_USER_INPUT 3 
#define SF_N_USER_OUTPUT 10

#include "userDef.h"
#include "ControllersStruct.h"
 
typedef struct UserIOStruct 
{
    double Voltage[31+1];
    double currents[31+1];
    double k_stiff[31+1];
    double k_damp[31+1];
    double last_t_ctrl;
    void *compute_gcm;
    void *contactGestion;
    double FB_state[6+1];
    int restart;
    struct ControllerPIDs *PIDs_pos;
    struct ControllerPIDs *PIDs_torque;
    struct ActuatorsStruct *actuatorsStruct;
    struct ControllerStruct *cvs;

    // stop simulation //
    int stop_simu;

    #ifdef SIMBODY
    SimbodyStruct *simbodyStruct;
    #endif

} UserIOStruct;

/*--------------------*/
#endif
