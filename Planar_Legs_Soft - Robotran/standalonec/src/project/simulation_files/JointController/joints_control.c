#include "simu_def.h"
#include "MBSdataStruct.h"

void joints_control(MBSdataStruct *MBSdata)
{
	int i;
	int joint_id;

    UserIOStruct *uvs;
    ControllerStruct *cvs;

    double Kp, Kd, Ki;
    double *q_ref,*Qq_ref;
    double error, delta_tsim;

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;
    
    Kp=0.;
    Kd=0.;

    //double voltage[COMAN_NB_JOINT_ACTUATED]={0.0};

    q_ref   = cvs->Outputs->q_ref;
    Qq_ref  = cvs->Outputs->Qq_ref;
	
    delta_tsim = MBSdata->tsim - uvs->last_t_ctrl;

  // PD control law
    for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
		joint_id = uvs->actuatorsStruct->JointIds[i];
        
      switch (cvs->Outputs->servo_type[i+1]) {

        case POSITION_CTRL:
          position_control_mode:
  
            // PID structure for each joint is initialized with YARP Conf if YARP flag is set
            // or by the default values if YARP flag is off.
            Kp = uvs->PIDs_pos->p[i+1];
            Kd = uvs->PIDs_pos->d[i+1];
            Ki = uvs->PIDs_pos->i[i+1];

			error = q_ref[i+1] - MBSdata->q[joint_id];
            uvs->PIDs_pos->int_err[i+1] += error * delta_tsim;
            
			uvs->Voltage[i+1]//uvs->actuatorsStruct->MotorIds[joint_id]]
				= Kp*(error)-Kd*MBSdata->qd[joint_id] + Ki*uvs->PIDs_pos->int_err[i+1];
            break;

        case TORQUE_CTRL:
          torque_control_mode:
            
            // PID structure for each joint is initialized with YARP Conf if YARP flag is set
            // or by the default values if YARP flag is off.
            Kp = uvs->PIDs_torque->p[i+1];
            
			uvs->Voltage[i+1] = Kp*(Qq_ref[i+1] - MBSdata->Qq[joint_id]);
            break;

        case OPEN_LOOP_CTRL: 
            break; // Voltage directly set by Yarp interface

        case IMPEDANCE_POS_CTRL:

          Kp = uvs->k_stiff[i+1];
          Kd = uvs->k_damp[i+1];

		  error = q_ref[i+1] - MBSdata->q[joint_id];

		  Qq_ref[i+1] = Kp*(error)-Kd*MBSdata->qd[joint_id];

          goto torque_control_mode;  //Note : the gains for impedance control are the same as for torque control mode
          break;

        default:
			printf("detault control mode selected for joint %d \n", joint_id);
            goto position_control_mode;
            break;
      }

    }

        // limiting voltage
    for(i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
    {
        uvs->Voltage[i] = limit_function(uvs->Voltage[i], -MAX_ACT_VOLTAGE, MAX_ACT_VOLTAGE);
    }

}

