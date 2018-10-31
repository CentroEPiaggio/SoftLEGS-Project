/*===========================================================================*
 *
 *  user_sf_IO.c
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

#include "MBSfun.h" 
#include "user_sf_IO.h" 
#include "sfdef.h" 
#include "userDef.h"
#include "ControllersStruct.h"
#include "simu_def.h"


UserIOStruct * initUserIO(MBSdataStruct *s)
{
    UserIOStruct *uvs;

    int i;

    uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));

    // Voltage //
    for (i=1;i<=31;i++)
    {
        uvs->Voltage[i] = 0.0;
    }

    // currents //
    for (i=1;i<=31;i++)
    {
        uvs->currents[i] = 0.0;
    }

    // k_stiff //
    for (i=1;i<=31;i++)
    {
        uvs->k_stiff[i] = 0.0;
    }

    // k_damp //
    for (i=1;i<=31;i++)
    {
        uvs->k_damp[i] = 0.0;
    }

    // last_t_ctrl //
    uvs->last_t_ctrl = 0.0;

    // compute_gcm //

    // contactGestion //

    // FB_state //
    for (i=1;i<=6;i++)
    {
        uvs->FB_state[i] = 0.0;
    }

    // restart //
    uvs->restart = 0;

    // PIDs_pos //
    uvs->PIDs_pos = init_ControllerPIDs();

    // PIDs_torque //
    uvs->PIDs_torque = init_ControllerPIDs();

    // actuatorsStruct //
    uvs->actuatorsStruct = init_ActuatorsStruct();

    // cvs //
    uvs->cvs = init_ControllerStruct();
    // stop simulation //
    uvs->stop_simu = 0;

    // simbodyStruct //
    #ifdef SIMBODY
    uvs->simbodyStruct = init_SimbodyStruct();
    #endif

    return uvs;
}


void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{

    // ControllerPIDs: PIDs_pos //
    free_ControllerPIDs(uvs->PIDs_pos);

    // ControllerPIDs: PIDs_torque //
    free_ControllerPIDs(uvs->PIDs_torque);

    // ActuatorsStruct: actuatorsStruct //
    free_ActuatorsStruct(uvs->actuatorsStruct);

    // ControllerStruct: cvs //
    free_ControllerStruct(uvs->cvs);

    // SimbodyStruct: simbodyStruct //
    #ifdef SIMBODY
    free_SimbodyStruct(uvs->simbodyStruct);
    #endif

    free(uvs);
}

#ifndef STANDALONE
 
void sf_set_user_input_sizes(SimStruct *S, MBSdataStruct *MBSdata, int sf_ninput) 
{ 
   if (SF_N_USER_INPUT > 0) { // warning: index starts at sf_ninput 
        // example: ssSetInputPortWidth(S,sf_ninput,10);

	   /* User input port0 : reference position */
	   ssSetInputPortWidth(S,sf_ninput,31);
       //ssSetInputPortDirectFeedThrough(S,sf_ninput,1);

	   /* User input port1 : reference torque */
	   ssSetInputPortWidth(S,sf_ninput+1,31);
       //ssSetInputPortDirectFeedThrough(S,sf_ninput+1,1);

	   /* User input port2 : servo mode (position control , torque control) */
	   ssSetInputPortWidth(S,sf_ninput+2,31);
       //ssSetInputPortDirectFeedThrough(S,sf_ninput+2,1);
   } 
} 

void sf_set_user_output_sizes(SimStruct *S, MBSdataStruct *MBSdata) 
        // example: ssSetOutputPortWidth(S, SF_NOUTPUT, 10); 
{ 
   if (SF_N_USER_OUTPUT > 0) { // warning: index starts at SF_NOUTPUT 

       /* User output port0 : time */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT, 1); 
	   
       /* User output port1 : link position */
       ssSetOutputPortWidth(S, SF_NOUTPUT+1, 31);

       /* User output port2 : link velocity */
       ssSetOutputPortWidth(S, SF_NOUTPUT+2, 31);

       /* User output port3 : link torque */
       ssSetOutputPortWidth(S, SF_NOUTPUT+3, 31);

       /* User output port4 : motor position */
       ssSetOutputPortWidth(S, SF_NOUTPUT+4, 31);

       /* User output port5 : motor velocity */
       ssSetOutputPortWidth(S, SF_NOUTPUT+5, 31);

       /* User output port6 : Rfoot Force-torque sensor */
       ssSetOutputPortWidth(S, SF_NOUTPUT+6, 6);

       /* User output port7 : Lfoot Force-torque sensor */
       ssSetOutputPortWidth(S, SF_NOUTPUT+7, 6);

       /* User output port8 : IMU waist sensor */
       ssSetOutputPortWidth(S, SF_NOUTPUT+8, 3+3+9);

       /* User output port9 : waist pose */
       ssSetOutputPortWidth(S, SF_NOUTPUT+9, 6);
   } 
} 

void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput) 
{ 
    // warning: index starts at sf_ninput
    // example: InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);
    //          MBSdata->user_IO->var1 = *uPtrs0[0];

	int i;

	InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);
	InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,sf_ninput+1);
	InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S,sf_ninput+2);

	/* User input port0 : reference position */
	for(i=0; i<31;i++)
        MBSdata->user_IO->cvs->Outputs->q_ref[i+1] = *uPtrs0[i];

	/* User input port0 : reference torque */
	for(i=0; i<31;i++)
        MBSdata->user_IO->cvs->Outputs->Qq_ref[i+1] = *uPtrs1[i];

	/* User input port0 : servo mode (position control , torque control) */
	for(i=0; i<31;i++)
        MBSdata->user_IO->cvs->Outputs->servo_type[i+1] = *uPtrs1[i];
} 

void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
    // warning: index starts at SF_NOUTPUT  
    // example: real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
    //          *y0 = MBSdata->user_IO->var1;  
    int i;
    ControllerInputs* ivs;

    real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT);
    real_T *y1 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+1);
    real_T *y2 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+2);
    real_T *y3 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+3);
    real_T *y4 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+4);
    real_T *y5 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+5);
    real_T *y6 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+6);
    real_T *y7 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+7);
    real_T *y8 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+8);
    real_T *y9 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+9);

    ivs = MBSdata->user_IO->cvs->Inputs;

   /* User output port1 : tsim_out */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT))
      *y0 = ivs->t;

   /* User output port0 : link position */
   //if (ssGetOutputPortConnected(S,SF_NOUTPUT+1))
      for (i=1;i<=31;i++)
          y1[i-1] = ivs->q[i];

   /* User output port2 : link velocity */
  // if (ssGetOutputPortConnected(S,SF_NOUTPUT+2))
      for (i=1;i<=31;i++)
          y2[i-1] = ivs->qd[i];

  /* User output port3 : link torque */
  // if (ssGetOutputPortConnected(S,SF_NOUTPUT+3)) 
      for (i=1;i<=31;i++)
          y3[i-1] = ivs->Qq[i];

  /* User output port4 : motor position */
  // if (ssGetOutputPortConnected(S,SF_NOUTPUT+4)) 
      for (i=1;i<=31;i++)
          y4[i-1] = ivs->q_mot[i];

   /* User output port5 : motor velocity */
  // if (ssGetOutputPortConnected(S,SF_NOUTPUT+5)) 
      for (i=1;i<=31;i++)
          y5[i-1] = ivs->qd_mot[i];

  /* User output port6 : Rfoot Force-torque sensor */
  // if (ssGetOutputPortConnected(S,SF_NOUTPUT+6)) 
      for (i=1;i<=3;i++){
          y6[i-1] = ivs->FT_sensor_list[FT_RFOOT_ID]->force[i-1];
          y6[i+3-1] = ivs->FT_sensor_list[FT_RFOOT_ID]->torque[i-1];
      }

    /* User output port7 : Rfoot Force-torque sensor */
    // if (ssGetOutputPortConnected(S,SF_NOUTPUT+7))
      for (i=1;i<=3;i++){
          y7[i-1] = ivs->FT_sensor_list[FT_LFOOT_ID]->force[i-1];
          y7[i+3-1] = ivs->FT_sensor_list[FT_LFOOT_ID]->torque[i-1];
      }


      /* User output port8 : IMU waist sensor */
      // if (ssGetOutputPortConnected(S,SF_NOUTPUT+8))
      for (i=1;i<=3;i++){
          y8[i-1] = ivs->IMU_list[IMU_WAIST_ID]->Angular_Rate[i-1];
          y8[i-1+3] = ivs->IMU_list[IMU_WAIST_ID]->Acceleration[i-1];
      }
      for(i=1;i<=9;i++){
          y8[i-1+6] = ivs->IMU_list[IMU_WAIST_ID]->Orientation[i-1];
      }

      /* User output port5 : waist pose */
     // if (ssGetOutputPortConnected(S,SF_NOUTPUT+5))
         for (i=1;i<=6;i++)
             y9[i-1] = MBSdata->q[i];

} 

#endif 
