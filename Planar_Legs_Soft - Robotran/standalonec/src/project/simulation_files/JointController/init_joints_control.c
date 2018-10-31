//---------------------------
// Timothee Habra
//
// Creation : 8-4-2015
// Last update : 8-4-2015
//
// Initialize the low level joints controller (motor control board)
//
//---------------------------

#include "user_all_id.h"
#include "ActuatorsDefinitions.h"
#include "MBSdataStruct.h"
#include "simu_def.h"

// Initializes joints controller (PID, references, ...)
void init_joints_control(MBSdataStruct *MBSdata)
{

    int i,j;
    UserIOStruct *uvs;
    ControllerStruct *cvs;

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;

    /******************************
    // servo type (position control, torque control, impedance control)
    /******************************/
    // e.g. uvs->servo_type[R_HIP_LAT_MOT+1] = POSITION_CTRL;

    for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
	{
         cvs->Outputs->servo_type[i+1] = POSITION_CTRL;  // position control by default
	}

	#ifndef STANDALONE
	MBSdata->q0   = (double*) calloc(MBSdata->njoint+1,sizeof(double));
	MBSdata->qd0  = (double*) calloc(MBSdata->njoint+1,sizeof(double));
	MBSdata->qdd0 = (double*) calloc(MBSdata->njoint+1,sizeof(double));
	MBSdata->q0[0]   = (double) MBSdata->njoint;
	MBSdata->qd0[0]  = (double) MBSdata->njoint;
	MBSdata->qdd0[0] = (double) MBSdata->njoint;
	for(i=1;i<=MBSdata->njoint;i++)
	{
		MBSdata->q0[i]   = MBSdata->q[i];
		MBSdata->qd0[i]  = 0.;
		MBSdata->qdd0[i] = 0.;
	}
	#endif


    /******************************
    // references (position, torque)
    /******************************/

    for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
    {
        cvs->Outputs->q_ref[i+1]  = MBSdata->q0[uvs->actuatorsStruct->JointIds[i]];  // MBS initial position
        cvs->Outputs->Qq_ref[i+1] = 0.;  // zero
    }


    /******************************
    // position control pid
    /******************************/

    // legs
    i = R_HIP_SAG_MOT;
    j = L_HIP_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 240.;
    uvs->PIDs_pos->i[i+1] = 0.;
    uvs->PIDs_pos->d[i+1] = 0.;

    i = R_HIP_LAT_MOT;
    j = L_HIP_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 52.;

    i = R_HIP_TRANS_MOT;
    j = L_HIP_TRANS_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 12.;

    i = R_KNEE_SAG_MOT;
    j = L_KNEE_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 300.;

    i = R_ANK_LAT_MOT;
    j = L_ANK_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 250.;

    i = R_ANK_SAG_MOT;
    j = L_ANK_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 280.;


    // torso
    i = WAIST_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = 40.;

    i = WAIST_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = 50.;

    i = WAIST_TRANS_MOT;
    uvs->PIDs_pos->p[i+1] = 40.;

    // arms
    i = R_SH_SAG_MOT;
    j = L_SH_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_SH_LAT_MOT;
    j = L_SH_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 80.;

    i = R_SH_TRANS_MOT;
    j = L_SH_TRANS_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_ELB_MOT;
    j = L_ELB_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 40.;

    i = R_FORE_ARM_PLATE_MOT;
    j = L_FORE_ARM_PLATE_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_WRJ1_MOT;
    j = L_WRJ1_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_WRJ2_MOT;
    j = L_WRJ2_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    // head
    i = NECK_YAW_MOT;
    uvs->PIDs_pos->p[i+1] = 10.;

    i = NECK_PITCH_MOT;
    uvs->PIDs_pos->p[i+1] = 10.;

    /******************************
    // Torque control pid
    /******************************/

    // TBD
    i = R_HIP_SAG_MOT;
    uvs->PIDs_torque->p[i+1] = 0.;

    /******************************
    // Impedance control pid
    /******************************/

    // TBD
    i = R_HIP_SAG_MOT;
    uvs->k_stiff[i+1] = 0.;
    uvs->k_damp[i+1] = 0.;

    
}
