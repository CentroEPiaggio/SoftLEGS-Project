//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 
 
#include "simu_def.h"

void user_Derivative(MBSdataStruct *MBSdata)
{
	int i;

    const int n = COMAN_NB_JOINT_ACTUATED;

    double rho ;
    double K_W ;
    double L_M ;
    double R_M ;
    double KT  ;

    double J_M;
    // voltage to torque gain (used in 2nd order dynamics)
    double VT ; //
    double D_M;

    double *voltage;

	SEActuatorStruct* actuator;

	int joint_id;
    
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;

    //-----------------------------
    // Houman actuator model
    //-----------------------------

    if (Act_type==1) //SEA
    {
        switch (Act_order) {

        case 1:
            justElectrical:
            // Motor (electrical) ODE
            // need a map from index i=0:4 to real joint indices
            // ux:current, uxd: current derivatives:

			voltage = uvs->Voltage;

            // All robot joints
            for (i=0; i<n; i++)
            {
				joint_id = uvs->actuatorsStruct->JointIds[i];
				actuator = uvs->actuatorsStruct->acs[i];

				rho = actuator->GearRatio;
				R_M = actuator->Resistance;
				K_W = actuator->Kbemf;
				L_M = actuator->Inductance;
				MBSdata->uxd[i+1] = (1.0 / L_M)*(voltage[i+1] - R_M*MBSdata->ux[i+1] - K_W*rho* MBSdata->qd[joint_id]);
            }
            break;

        case 2:
        // Motor (Mechanical) ODE
        // ux:motor position, velocity, uxd: motor velocity, acceleration

			voltage = uvs->Voltage;

            for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
            {
				joint_id = uvs->actuatorsStruct->JointIds[i];
				actuator = uvs->actuatorsStruct->acs[i];

				J_M =	actuator->Inertia;
				VT  =	actuator->GearRatio * actuator->Kbemf / actuator->Resistance;
				D_M =	actuator->Damping;

				//update derivative of position = motor velocity:
				MBSdata->uxd[i+1] = MBSdata->ux[i+1 + n];
				//update derivative of velocity = motor accelerations:
				MBSdata->uxd[i + 1 + n] = (1.0 / J_M)*(VT*voltage[i+1] - D_M*MBSdata->ux[n + i + 1] - MBSdata->Qq[joint_id]);
            }
            break;

        case 3:
        // Motor (Electrical+Mechanical) ODE
        // ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative
        //update motor velocities:

			voltage = uvs->Voltage;

            for (i=0; i<n; i++)
            {
				joint_id = uvs->actuatorsStruct->JointIds[i];
				actuator = uvs->actuatorsStruct->acs[i];

                MBSdata->uxd[i+1]=MBSdata->ux[i+1+n];

				rho = actuator->GearRatio;
				R_M = actuator->Resistance;
				K_W = actuator->Kbemf;
				L_M = actuator->Inductance;
                KT = K_W;

				J_M = actuator->Inertia;
                VT  = rho*(KT)/R_M;
				D_M = actuator->Damping;

                // update motor acceleration:
				MBSdata->uxd[i + 1 + n] = (1.0 / J_M)*(KT*MBSdata->ux[2 * n + i + 1] - D_M*MBSdata->ux[n + i + 1] - MBSdata->Qq[joint_id]);

                // update current derivative:
                MBSdata->uxd[i+1+2*n]=(1.0/L_M)*(voltage[i+1] -R_M*MBSdata->ux[2*n+i+1]-K_W*rho* MBSdata->ux[i+1+n]);
            }
            break;

        default:
            printf("detault actuator order (1) selected \n");
            goto justElectrical;
            break;
        }
    }

	//-----------------------------
	// Nico & Allan Actuator model
	//-----------------------------
	/*

	// convert voltage in torque form
	//      uvs->Control[i] is the torque form (in [Nm])
	//      uvs->Voltage[i] is the voltage (in [V])
	// i is the index in the motor indexes list
	for(i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
	{
	uvs->Control[i] = uvs->Voltage[i]*(uvs->Actuator_VTgain);
	}

	for (i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
	{
	// NOTE: ux[1 -> COMAN_NB_JOINT_ACTUATED] = joints positions; ux[COMAN_NB_JOINT_ACTUATED+1 -> 2*COMAN_NB_JOINT_ACTUATED] = joints velocities
	// returning motor velocity to the uxd
	MBSdata->uxd[i] = MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i];
	}

	// Computing accelerations of motors (i: motor reference)
	for (i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
	{
	i_real = uvs->actuated2real[i]; // real joint number

	// uvs->Control[i] contains the voltage applied to the motors converted in torque form.
	MBSdata->uxd[COMAN_NB_JOINT_ACTUATED+i] = (uvs->Control[i]-(uvs->Actuator_Ddrives)*(MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i])) / (uvs->Actuator_Jdrives);
	MBSdata->uxd[COMAN_NB_JOINT_ACTUATED+i]+= (-uvs->Actuator_KKs[i_real]*(MBSdata->ux[i]-MBSdata->q[i_real])) / (uvs->Actuator_Jdrives);
	MBSdata->uxd[COMAN_NB_JOINT_ACTUATED+i]+= (-uvs->Actuator_DDs[i_real]*(MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i]-MBSdata->qd[i_real])) / (uvs->Actuator_Jdrives);
	}
	*/
}

