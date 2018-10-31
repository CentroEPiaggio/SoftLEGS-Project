//-------------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2008 by JF Collard
// Last update : 01/10/2008
//-------------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°42
//

#define S_FUNCTION_LEVEL 2 

#define  _CRT_SECURE_NO_DEPRECATE // avoid warning in simulink.c with Visual C++

#include "user_sf_IO.h"
#include "MBSfun.h" 
#include "sfdef.h"
#include "MBSdef.h"

/* Function: mdlInitializeSizes ===============================================
* Abstract:
*    The sizes information is used by Simulink to determine the S-function
*    block's characteristics (number of inputs, outputs, states, etc.).
*/
static void mdlInitializeSizes(SimStruct *S) 
{ 
	MBSdataStruct *MBSdata;	 
	const mxArray *mbs_data_ptr = ssGetSFcnParam(S,0); // get 1st S-function parameter
	int sf_ninput = SF_NINPUT;
#if defined DIRDYNARED || defined ACCELRED
	const bool q_ini = (bool) mxGetScalar(ssGetSFcnParam(S,1)); // get 2nd S-function parameter
#else
	const bool q_ini = false;
#endif

	/* S-function block parameters */ 
	ssSetNumSFcnParams(S,SF_NPARAM);  /* Number of expected parameters */ 
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return; /* Parameter mismatch will be reported by Simulink */ 

	/* Get pointer to MBSdataStruct */
	if (mbs_data_ptr == NULL){ 
		ssWarning(S,"Could not get structure variable");
		if (!sf_set_default_IO_sizes(S)) return;
	} 
	else
	{
		/* MBSData sizes */ 
#ifdef HARDINPUT
		MBSdata = loadMBSsizes_hard();
#else
		MBSdata = loadMBSsizes(mbs_data_ptr);
#endif

		/* Input sizes */
		if (q_ini) sf_ninput += 2;
#ifdef OLDVER
		if (!ssSetNumInputPorts(S, SF_NINPUT + SF_N_USER_INPUT)) return;
#endif
		if (!sf_set_input_sizes(S, MBSdata)) return;
#if !defined SENSORKIN
		sf_set_user_input_sizes(S, MBSdata, sf_ninput);
#endif

		/* Output sizes */
#ifdef OLDVER
		if (!ssSetNumOutputPorts(S, SF_NOUTPUT + SF_N_USER_OUTPUT)) return; 
#endif
		if (!sf_set_output_sizes(S, MBSdata)) return; 
#if !defined SENSORKIN
		sf_set_user_output_sizes(S, MBSdata);
#endif

		/* State variable size */ 
		ssSetNumContStates(S, SF_NCSTATES); 
		ssSetNumDiscStates(S, SF_NDSTATES); 

		/* Only needed for size loading */
		free(MBSdata);
	}

	/* Number of Sample Times */ 
	ssSetNumSampleTimes(S, 1); 

	/* Number of PWorks */ 
	ssSetNumPWork(S,2); // 2 PWorks: MBSdata & LocalData (local data for simulation)
} 


/* Function: mdlInitializeSampleTimes ========================================= 
* Abstract: 
*    Specifiy that we have a continuous or inherited sample time. 
*/ 
static void mdlInitializeSampleTimes(SimStruct *S) 
{ 
	ssSetSampleTime(S, 0, SF_SAMPLE_TIME); 
	ssSetOffsetTime(S, 0, 0.0); 
} 


#ifdef SENSORKIN

#define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
/*    int_T outWidth = ssGetOutputPortWidth(S, 0);
    /* Input port dimension must be unknown. Set it to scalar. *
    if(!ssSetInputPortMatrixDimensions(S, 0, 1, 1)) return;
    if(outWidth == DYNAMICALLY_SIZED){
        /* Output dimensions are unknown. Set it to scalar. *
        if(!ssSetOutputPortMatrixDimensions(S, 0, 1, 1)) return;
    }*/
}

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
										 int_T            port,
										 const DimsInfo_T *dimsInfo)
{
	if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
										  int_T            port,
										  const DimsInfo_T *dimsInfo)
{
	if(!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif


#define MDL_START
/* Function: mdlStart ======================================================= 
* Abstract: 
*    This function is called once at start of model execution. If you 
*    have states that should be initialized once, this is the place 
*    to do it. 
*/ 
static void mdlStart(SimStruct *S) 
{ 
	MBSdataStruct *MBSdata;
	LocalDataStruct *lds = NULL;
	const mxArray *mbs_data_ptr = ssGetSFcnParam(S,0); 	 
	int sf_ninput = ssGetNumInputPorts(S)-SF_N_USER_INPUT;

	/* MBSdataStruct variable name */ 
	if (mbs_data_ptr == NULL){ 
		ssSetErrorStatus(S,"Could not get structure variable");
		return;
	} 

	/* MBSDataStruct initialization --> PWork0 */ 
#ifdef HARDINPUT
	MBSdata = loadMBSdata_hard();
#else
	MBSdata = loadMBSdata(mbs_data_ptr); 
#endif
	checkMBSdata(MBSdata);
	ssSetPWorkValue(S,0,MBSdata); 

	/* LocalDataStruct initialization --> PWork1 */ 
#if !defined ACCELRED
	lds = initLocalDataStruct(S, MBSdata); 
#else
	lds = NULL;
#endif
	ssSetPWorkValue(S,1,lds);

	/* User initialization */
	sf_get_input(S, MBSdata, lds);
#if !defined SENSORKIN
	sf_get_user_input(S, MBSdata, lds, sf_ninput);
#endif
	user_initialization(S, MBSdata, lds);
} 


#ifdef MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ======================================== 
* Abstract: 
*    Initialize generalized coordinates. 
*/ 
static void mdlInitializeConditions(SimStruct *S) 
{ 
	MBSdataStruct *MBSdata = (MBSdataStruct *) ssGetPWorkValue(S,0); // get 1st S-function PWork

	sf_InitCond(S,MBSdata);
} 
#endif


/* Function: mdlOutputs ======================================================= 
* Abstract: 
*   In this function, you compute the outputs of your S-function block. 
*/ 
static void mdlOutputs(SimStruct *S, int_T tid) 
{ 
	MBSdataStruct *MBSdata = (MBSdataStruct *) ssGetPWorkValue(S,0); 
	LocalDataStruct *lds  = (LocalDataStruct *) ssGetPWorkValue(S,1);
    //int sf_ninput = ssGetNumInputPorts(S)-SF_N_USER_INPUT;
    int i;

	/* Inputs */
//	sf_get_input(S, MBSdata, lds);
//#if !defined SENSORKIN
//	sf_get_user_input(S, MBSdata, lds, sf_ninput);
//#endif

	/* Update model */	
    mbs_compute_model(S,MBSdata,lds);

	// update fixed joint forces (necessary for Force-Torque sensor (walkman project)). Otherwise, Qq a reset to 0 by dirdinared.
    for(i=1;i<=MBSdata->nqc;i++)
    {
        MBSdata->Qq[MBSdata->qc[i]] = lds->Qc[i];
    }

	/* User output computation */
#if !defined SENSORKIN
	user_compute_output(S,MBSdata,lds);
#endif

	/* Outputs */ 
	sf_set_output(S,MBSdata, lds);
#if !defined SENSORKIN
	sf_set_user_output(S,MBSdata,lds);
#endif
} 


#ifdef MDL_DERIVATIVES
/* Function: mdlDerviatives ======================================================= 
* Abstract: 
*   In this function, you compute the S-function block's derivatives. 
*/ 
static void mdlDerivatives(SimStruct *S) 
{ 
	MBSdataStruct *MBSdata = (MBSdataStruct *) ssGetPWorkValue(S,0); 
	LocalDataStruct *lds  = (LocalDataStruct *) ssGetPWorkValue(S,1); 

	int i; 

	real_T *dx = ssGetdX(S); 

    int sf_ninput = ssGetNumInputPorts(S)-SF_N_USER_INPUT;

    /* Inputs */
    sf_get_input(S, MBSdata, lds);
#if !defined SENSORKIN
    sf_get_user_input(S, MBSdata, lds, sf_ninput);
#endif

	/* Update model */
    //mbs_compute_model(S,MBSdata, lds); // model is already updated in mdlOutput

	/* Update state vector */	
	for(i=1;i<=MBSdata->nqu;i++) 
	{ 
		dx[i-1] = MBSdata->qd[MBSdata->qu[i]]; 
		dx[i+MBSdata->nqu-1] = MBSdata->qddu[i]; 
	} 
	for (i=1;i<=MBSdata->Nux;i++) 
	{ 
		dx[i+2*MBSdata->nqu-1] = MBSdata->uxd[i]; 
	}
} 
#endif


static void mdlTerminate(SimStruct *S) 
{ 
	MBSdataStruct *MBSdata = (MBSdataStruct *) ssGetPWorkValue(S,0); 
	LocalDataStruct *lds  = (LocalDataStruct *) ssGetPWorkValue(S,1);

/*	char s_name[DefaultStringLength];

	/* MBSdataStruct variable name *
	mxGetString(ssGetSFcnParam(S,0),s_name,DefaultStringLength); 

	storeMBSdata(MBSdata,s_name); //vérifier l'utilité de cette fonction
*/
#if !defined ACCELRED
	freeLocalDataStruct(S, lds,MBSdata); 
#endif

	freeMBSdata(MBSdata); 
} 

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
