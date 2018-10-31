//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 
 
#include "simu_def.h"
 
double* user_JointForces(MBSdataStruct *MBSdata, double tsim)
{ 
	int i;

    int joint_id;
    
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;

	if (Act_type == 1) //SEA
	{
		switch (Act_order) {

		case 1:
			printf("pure electrical torque not yet implemented \n");
			break;

		case 2:
			// Motor (Mechanical) ODE
			// ux:motor position, velocity, uxd: motor velocity, acceleration

			for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
			{
				joint_id = uvs->actuatorsStruct->JointIds[i];

				MBSdata->Qq[joint_id] = uvs->actuatorsStruct->acs[i]->SeriesSpring*(MBSdata->ux[i+1] - MBSdata->q[joint_id]);
                MBSdata->Qq[joint_id] += uvs->actuatorsStruct->acs[i]->SeriesDamping*(MBSdata->ux[i+1 + COMAN_NB_JOINT_ACTUATED] - MBSdata->qd[joint_id]);
			}
			break;

		case 3:
			// Motor (Electrical+Mechanical) ODE
			// ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative

			for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
			{
				joint_id = uvs->actuatorsStruct->JointIds[i];

				MBSdata->Qq[joint_id] = uvs->actuatorsStruct->acs[i]->SeriesSpring*(MBSdata->ux[i + 1] - MBSdata->q[joint_id]);
				MBSdata->Qq[joint_id] += uvs->actuatorsStruct->acs[i]->SeriesDamping*(MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED] - MBSdata->qd[joint_id]);
			}

			break;
		}

	}

    // set Qq of driven joint (blocked) to 0 (Mandatory to use Qq for FT sensors)
    for(i=1;i<=MBSdata->nqc;i++)
    {
        MBSdata->Qq[MBSdata->qc[i]] = 0.;
    }
 
	//-----------------------------
	// Nico & Allan Actuator model
	//-----------------------------
	/*
    // adding the coupling torques between the motors and the joints (i: motor reference)
    //for (i=1; i<=COMAN_NB_JOINT_ACTUATED; i++) 
    //{
		//i_real = uvs->actuated2real[i]; // real joint number (Robotran numbering)
        //MBSdata->Qq[i_real]  = (uvs->Actuator_KKs[i_real]*(MBSdata->ux[i]-MBSdata->q[i_real]));
        //MBSdata->Qq[i_real] += (uvs->Actuator_DDs[i_real]*(MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i]-MBSdata->qd[i_real]));
    //}

    // --- Torque produced by the springs in the compliant foot --- //

    #ifdef COMP_FEET

    // Right foot
    MBSdata->Qq[R_TOE]  = uvs->Actuator_KKs[R_TOE]*(-MBSdata->q[R_TOE]);  // stiffness
    MBSdata->Qq[R_TOE] += uvs->Actuator_DDs[R_TOE]*(-MBSdata->qd[R_TOE]); // damping
    
    // Left foot
    MBSdata->Qq[L_TOE]  = uvs->Actuator_KKs[L_TOE]*(-MBSdata->q[L_TOE]);  // stiffness
    MBSdata->Qq[L_TOE] += uvs->Actuator_DDs[L_TOE]*(-MBSdata->qd[L_TOE]); // damping

    #endif
	*/
 
    return MBSdata->Qq;
} 
