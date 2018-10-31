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
// 01/10/2008 : JFC : Bug n°41
//


#include "MBSfun.h" 


#if defined DIRDYNARED || defined ACCELRED
/* Function: mbs_compute_model
* Abstract:
*    get the simstruct values to compute the model
*/
void mbs_compute_model(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
	int i; 

	real_T *x = ssGetContStates(S);  

	MBSdata->tsim = ssGetT(S); 
	
	/* Update state variables */ 
	for(i=1;i<=MBSdata->nqu;i++) 
	{ 
		MBSdata->q[MBSdata->qu[i]] = x[i-1]; 
		MBSdata->qd[MBSdata->qu[i]] = x[i+MBSdata->nqu-1]; 
	} 
	for (i=1;i<=MBSdata->Nux;i++) 
	{ 
		MBSdata->ux[i] = x[i+2*MBSdata->nqu-1]; 
	} 

	/* MBS Derivatives */ 
	if(MBSdata->DonePart==0)
	{
		ssSetErrorStatus(S,"Please perform a partitionning first !\n");
		return;
	}

	/* Direct Dynamics computation */
#ifdef DIRDYNARED
	i = dirdynared(lds,MBSdata);
#elif defined ACCELRED
	i = accelred(MBSdata->qddu,MBSdata,MBSdata->tsim);
#endif

	if (i<0) ssSetErrorStatus(S,"Loop closing Error : NR iteration overrun !\n"); 

	/* User Derivatives */ 
	if (MBSdata->Nux>0)  user_Derivative(MBSdata); 
} 

#elif defined INVDYNARED
/* Function: mbs_compute_model
* Abstract:
*    get the simstruct values to compute the model
*/
void mbs_compute_model(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
	int i;

	MBSdata->tsim = ssGetT(S);

	/* Inverse Dynamics computation */
	i = invdynared(lds,MBSdata);

	if(i<0) ssSetErrorStatus(S,"Loop closing Error : NR iteration overrun !\n"); 
} 


#elif defined SENSORKIN
/* Function: mbs_compute_model
* Abstract:
*    get the simstruct values to compute the model
*/
void mbs_compute_model(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
	int i;
	const bool is_gensensor = (bool) mxGetScalar(ssGetSFcnParam(S,1)); // get 2nd S-function parameter
	double *sensor_indices = mxGetPr(ssGetSFcnParam(S,2));
	const int nsensor = mxGetNumberOfElements(ssGetSFcnParam(S,2)); // get 3rd S-function parameter size

	if (ssIsMajorTimeStep(S)) {
		MBSdata->tsim = ssGetT(S);

		/* Sensor Kinematics computation */
		if (is_gensensor)
			for (i=1;i<=nsensor;i++) {
				init_sensor(lds->psensorStruct[i],MBSdata->njoint);
				gensensor(lds->psensorStruct[i], MBSdata, (int)sensor_indices[i-1]);
			}
		else
			for (i=1;i<=nsensor;i++) {
				init_sensor(lds->psensorStruct[i],MBSdata->njoint);
				sensor(lds->psensorStruct[i], MBSdata, (int)sensor_indices[i-1]);
			}
	}
}

#endif
