//---------------------------
// Nicolas Van der Noot, Allan Barrea, Timothee Habra, Houman Dallali
//
// Creation : 03/03/2013
// Last update : 8-4-2015
//
// Initialize the actuators models
//
//---------------------------

#include "MBSdataStructR7.h"
#include "user_all_id.h"
#include "ActuatorsDefinitions.h"

// Initializes actuators model (stiffness and damping coefficients)
void init_actuator_model(MBSdataStruct *MBSdata)
{

    int i;

	ActuatorsStruct* actuatorsStruct;

    /******************************
    // Joint limits
    /******************************
    // Joints limits should be define here (see CoMan for example)


    /******************************
    // Map actuators -> joints
    ******************************/

	actuatorsStruct = MBSdata->user_IO->actuatorsStruct;

    // right leg
    actuatorsStruct->JointIds[R_HIP_LAT_MOT]    = RHipLat_id;
    actuatorsStruct->JointIds[R_HIP_TRANS_MOT]  = RHipYaw_id;
    actuatorsStruct->JointIds[R_HIP_SAG_MOT]    = RHipSag_id;
    actuatorsStruct->JointIds[R_KNEE_SAG_MOT]   = RKneeSag_id;
    actuatorsStruct->JointIds[R_ANK_SAG_MOT]    = RAnkSag_id;
    actuatorsStruct->JointIds[R_ANK_LAT_MOT]    = RAnkLat_id;

    // left leg
    actuatorsStruct->JointIds[L_HIP_LAT_MOT]    = LHipLat_id;
    actuatorsStruct->JointIds[L_HIP_TRANS_MOT]  = LHipYaw_id;
    actuatorsStruct->JointIds[L_HIP_SAG_MOT]    = LHipSag_id;
    actuatorsStruct->JointIds[L_KNEE_SAG_MOT]   = LKneeSag_id;
    actuatorsStruct->JointIds[L_ANK_SAG_MOT]    = LAnkSag_id;
    actuatorsStruct->JointIds[L_ANK_LAT_MOT]    = LAnkLat_id;

    // waist
    actuatorsStruct->JointIds[WAIST_LAT_MOT]    = WaistLat_id;
    actuatorsStruct->JointIds[WAIST_SAG_MOT]    = WaistSag_id;
    actuatorsStruct->JointIds[WAIST_TRANS_MOT]  = WaistYaw_id;

    // right arm
    actuatorsStruct->JointIds[R_SH_SAG_MOT]         = RShSag_id;
    actuatorsStruct->JointIds[R_SH_LAT_MOT]         = RShLat_id;
    actuatorsStruct->JointIds[R_SH_TRANS_MOT]       = RShYaw_id;
    actuatorsStruct->JointIds[R_ELB_MOT]            = RElbj_id;
    actuatorsStruct->JointIds[R_FORE_ARM_PLATE_MOT] = RForearmPlate_id;
    actuatorsStruct->JointIds[R_WRJ1_MOT]           = RWrj1_id;
    actuatorsStruct->JointIds[R_WRJ2_MOT]           = RWrj2_id;

    // left arm
    actuatorsStruct->JointIds[L_SH_SAG_MOT]         = LShSag_id;
    actuatorsStruct->JointIds[L_SH_LAT_MOT]         = LShLat_id;
    actuatorsStruct->JointIds[L_SH_TRANS_MOT]       = LShYaw_id;
    actuatorsStruct->JointIds[L_ELB_MOT]            = LElbj_id;
    actuatorsStruct->JointIds[L_FORE_ARM_PLATE_MOT] = LForearmPlate_id;
    actuatorsStruct->JointIds[L_WRJ1_MOT]           = LWrj1_id;
    actuatorsStruct->JointIds[L_WRJ2_MOT]           = LWrj2_id;

    // head
    actuatorsStruct->JointIds[NECK_YAW_MOT]   = NeckYawj_id;
    actuatorsStruct->JointIds[NECK_PITCH_MOT] = NeckPitchj_id;


    /******************************
    // Map actuators -> joints
    ******************************/

    /*for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
	{
		actuatorsStruct->MotorIds[actuatorsStruct->JointIds[i]] = i;
    }*/

    /******************************
    // init actuator struct
    ******************************/
    // nominal values are based on Actuator Parameters Ver. 03
    // right leg
    actuatorsStruct->acs[R_HIP_SAG_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[R_HIP_LAT_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[R_HIP_TRANS_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[R_KNEE_SAG_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[R_ANK_LAT_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[R_ANK_SAG_MOT]->type = TYPE_BIG;

    // left leg
    actuatorsStruct->acs[L_HIP_SAG_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[L_HIP_LAT_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[L_HIP_TRANS_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[L_KNEE_SAG_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[L_ANK_LAT_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[L_ANK_SAG_MOT]->type = TYPE_BIG;

    // waist
    actuatorsStruct->acs[WAIST_LAT_MOT]->type = TYPE_MEDIUM;
    actuatorsStruct->acs[WAIST_SAG_MOT]->type = TYPE_BIG;
    actuatorsStruct->acs[WAIST_TRANS_MOT]->type = TYPE_MEDIUM2;

    // right arm
    actuatorsStruct->acs[R_SH_SAG_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[R_SH_LAT_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[R_SH_TRANS_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[R_ELB_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[R_FORE_ARM_PLATE_MOT]->type = TYPE_SMALL;
    actuatorsStruct->acs[R_WRJ1_MOT]->type = TYPE_SMALL;
    actuatorsStruct->acs[R_WRJ2_MOT]->type = TYPE_SMALL;

    // left arm
    actuatorsStruct->acs[L_SH_SAG_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[L_SH_LAT_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[L_SH_TRANS_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[L_ELB_MOT]->type = TYPE_MEDIUM2;
    actuatorsStruct->acs[L_FORE_ARM_PLATE_MOT]->type = TYPE_SMALL;
    actuatorsStruct->acs[L_WRJ1_MOT]->type = TYPE_SMALL;
    actuatorsStruct->acs[L_WRJ2_MOT]->type = TYPE_SMALL;

    // head
    actuatorsStruct->acs[NECK_YAW_MOT]->type    = TYPE_SMALL;
    actuatorsStruct->acs[NECK_PITCH_MOT]->type  = TYPE_SMALL;

    for (i=0; i < COMAN_NB_JOINT_ACTUATED; i++)
    {
        init_SEActuatorStruct(actuatorsStruct->acs[i]);
    }

    /******************************
    // initial user state initial value (motors positions, electric currents,...)
    ******************************/

    switch (Act_order) {

    case 1:
        justElectrical:
        // Motor (electrical) ODE
        // ux:current : init to 0 nothing to do
        break;

    case 2:
    // Motor (Mechanical) ODE
    // ux:motor position, velocity
        for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
        {
			MBSdata->ux[i + 1] = MBSdata->q[actuatorsStruct->JointIds[i]];
			MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED] = MBSdata->qd[actuatorsStruct->JointIds[i]];

            // init for sensor model
            MBSdata->user_IO->cvs->Inputs->q_mot[i + 1]  = MBSdata->ux[i + 1];
            MBSdata->user_IO->cvs->Inputs->qd_mot[i + 1] = MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED];
        }
        break;

    case 3:
    // Motor (Electrical+Mechanical) ODE
    // ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative
    //update motor velocities:
        for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
        {
			MBSdata->ux[i + 1] = MBSdata->q[actuatorsStruct->JointIds[i]];
			MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED] = MBSdata->qd[actuatorsStruct->JointIds[i]];

            // init for sensor model
            MBSdata->user_IO->cvs->Inputs->q_mot[i + 1]  = MBSdata->ux[i + 1];
            MBSdata->user_IO->cvs->Inputs->qd_mot[i + 1] = MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED];
        }

    default:
        printf("detault actuator order (1) selected \n");
        goto justElectrical;
        break;
    }


    // Actuators sensors

    for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
        // link position
        MBSdata->user_IO->cvs->Inputs->q[i + 1] = MBSdata->q[actuatorsStruct->JointIds[i]];

        // link velocity
        MBSdata->user_IO->cvs->Inputs->qd[i + 1] = 0.;

        // link torque
        MBSdata->user_IO->cvs->Inputs->Qq[i + 1] = 0.;

        // motor position & velocity see above
    }

}
