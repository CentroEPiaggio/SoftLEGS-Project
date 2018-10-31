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
// 01/10/2008 : JFC : Bug n°43
//

#include "MBSfun.h" 
#include "sfdef.h" 
#include "userDef.h"

int sf_set_default_IO_sizes(SimStruct *S)
{
	int i;

	/* INPUTS */
	if (!ssSetNumInputPorts(S, SF_NINPUT)) return 0;
	for (i=0;i<SF_NINPUT;i++)
		ssSetInputPortWidth(S,i,DYNAMICALLY_SIZED);

	/* OUTPUTS */
	if (!ssSetNumOutputPorts(S, SF_NOUTPUT)) return 0;
	for (i=0;i<SF_NOUTPUT;i++)
		ssSetOutputPortWidth(S,i,DYNAMICALLY_SIZED);

	return 1;
}

#if defined DIRDYNARED || defined ACCELRED
int sf_set_input_sizes(SimStruct *S, MBSdataStruct *MBSdata)
{
	int ninput = SF_NINPUT + SF_N_USER_INPUT;
	const bool q_ini = (bool) mxGetScalar(ssGetSFcnParam(S,1)); // get 2nd S-function parameter
	if (q_ini) ninput += 2;

	if (!ssSetNumInputPorts(S, ninput)) return 0;

	/* INPUTS */

	/* Port 0 : Qq */
	if (MBSdata->nqa)
	{
		ssSetInputPortWidth(S,0,MBSdata->nqa);
	}
	else
		ssSetInputPortWidth(S,0,1);
	ssSetInputPortDirectFeedThrough(S, 0, 1);

	/* Port 1 : qdriven */
	if (MBSdata->nqdriven)
		//if (MBSdata->nqc)
	{
		ssSetInputPortWidth(S,1,3*MBSdata->nqdriven);
		//ssSetInputPortWidth(S,1,3*MBSdata->nqc);
	}
	else
		ssSetInputPortWidth(S,1,1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);

	if (q_ini)
	{
		/* Port 2 : q_ini */
		ssSetInputPortWidth(S,2,MBSdata->nbody);
		ssSetInputPortDirectFeedThrough(S, 2, 1);

		/* Port 3 : qd_ini */
		ssSetInputPortWidth(S,3,MBSdata->nbody);
		ssSetInputPortDirectFeedThrough(S, 3, 1);
	}

	return 1;
}

int sf_set_output_sizes(SimStruct *S, MBSdataStruct *MBSdata)
{
	int noutput = SF_NOUTPUT + SF_N_USER_OUTPUT;

	if (!ssSetNumOutputPorts(S, noutput)) return 0; 

	/* OUTPUTS */

	/* Port 0 : q */
	ssSetOutputPortWidth(S,0,MBSdata->nbody);

	/* Port 1 : qd */
	ssSetOutputPortWidth(S,1,MBSdata->nbody);

	/* Port 2 : qdd */
	ssSetOutputPortWidth(S,2,MBSdata->nbody);

	return 1;
}

void sf_get_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{
	int i;

	InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0); //Joint forces
	InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1); //driven variables
	const bool q_ini = (bool) mxGetScalar(ssGetSFcnParam(S,1)); // get 2nd S-function parameter

	/* Inputs */

	/* Port 0 : Joint forces */
	if (ssGetInputPortConnected(S,0))
	{
		for(i=1;i<=MBSdata->nqa;i++) 
		{ 
			MBSdata->Qq[MBSdata->qa[i]] = *uPtrs0[i-1]; 
		} 
	}

	/* Port 1 : Driven joints qc (q,qd,qdd) */
	if (ssGetInputPortConnected(S,1))
	{
		//		for(i=1;i<=MBSdata->nqc;i++) 
		for(i=1;i<=MBSdata->nqdriven;i++) 
		{ 
			MBSdata->q[MBSdata->qdriven[i]] = *uPtrs1[3*i-3]; 
			MBSdata->qd[MBSdata->qdriven[i]] = *uPtrs1[3*i-2]; 
			MBSdata->qdd[MBSdata->qdriven[i]] = *uPtrs1[3*i-1]; 
			//MBSdata->q[MBSdata->qc[i]] = *uPtrs1[3*i-3]; 
			//MBSdata->qd[MBSdata->qc[i]] = *uPtrs1[3*i-2]; 
			//MBSdata->qdd[MBSdata->qc[i]] = *uPtrs1[3*i-1]; 
		} 
	}

	if (q_ini)
		if (ssGetT(S) == ssGetTStart(S))
		{
			InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S,2); //q_ini
			InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs(S,3); //qd_ini
			real_T *x0 = ssGetContStates(S);

			for(i=1;i<=MBSdata->nbody;i++)
			{ 
				MBSdata->q[i]  = *uPtrs2[i-1]; 
				MBSdata->qd[i] = *uPtrs3[i-1]; 
			} 		

			for(i=1;i<=MBSdata->nqu;i++) // Reset initial conditions
			{ 
				x0[i-1] = MBSdata->q[MBSdata->qu[i]]; 
				x0[MBSdata->nqu+i-1] = MBSdata->qd[MBSdata->qu[i]]; 
			}
		}
}

void sf_set_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{
	int i;

	real_T *y0 = ssGetOutputPortRealSignal(S,0); /* q */
	real_T *y1 = ssGetOutputPortRealSignal(S,1); /* qd */
	real_T *y2 = ssGetOutputPortRealSignal(S,2); /* qdd */

	/* Port 0: q   */ 
	if (ssGetOutputPortConnected(S,0))
		for(i=1;i<=MBSdata->nbody;i++)
			y0[i-1] = MBSdata->q[i]; 

	/* Port 1: qd  */ 
	if (ssGetOutputPortConnected(S,1))
		for(i=1;i<=MBSdata->nbody;i++) 
			y1[i-1] = MBSdata->qd[i]; 

	/* Port 2: qdd */ 
	if (ssGetOutputPortConnected(S,2))
		for(i=1;i<=MBSdata->nbody;i++) 
			y2[i-1] = MBSdata->qdd[i];
}

#elif defined INVDYNARED
int sf_set_input_sizes(SimStruct *S, MBSdataStruct *MBSdata)
{
	int ninput = SF_NINPUT + SF_N_USER_INPUT;

	if (!ssSetNumInputPorts(S, ninput)) return 0;

	/* INPUTS */

	/* Port 0 : q */
	//	if ((!ssGetInputPortConnected(S,0)) && ((MBSdata->nqu>0) || (MBSdata->nqc>0)))
	//		mexErrMsgTxt("Input Port q must be connected\n"); // too restrictive
	ssSetInputPortWidth(S,0,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 0, 1);

	/* Port 1 : qd */
	//	if ((!ssGetInputPortConnected(S,1)) && ((MBSdata->nqu>0) || (MBSdata->nqc>0)))
	//		mexErrMsgTxt("Input Port qd must be connected\n"); // too restrictive
	ssSetInputPortWidth(S,1,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 1, 1);

	/* Port 2 : qdd */
	//	if ((!ssGetInputPortConnected(S,2)) && ((MBSdata->nqu>0) || (MBSdata->nqc>0)))
	//		mexErrMsgTxt("Input Port qdd must be connected\n"); // too restrictive
	ssSetInputPortWidth(S,2,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 2, 1);

	/* Port 3 : Qq */
	ssSetInputPortWidth(S,3,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 3, 1);

	return 1;
}

int sf_set_output_sizes(SimStruct *S, MBSdataStruct *MBSdata)
{
	int noutput = SF_NOUTPUT + SF_N_USER_OUTPUT;

	if (!ssSetNumOutputPorts(S, noutput)) return 0; 

	/* OUTPUTS */

	/* Port 0 : Qa */
	if (MBSdata->nqa)
		ssSetOutputPortWidth(S,0,MBSdata->nqa);
	else
#ifdef MATLAB_MEX_FILE
		mexErrMsgTxt("No actuated joint\n");
#endif


	/* Port 1 : Qc */
	if (MBSdata->nqc)
		ssSetOutputPortWidth(S,1,MBSdata->nqc);
	else
		ssSetOutputPortWidth(S,1,1);


	/* Port 2 : q */
	ssSetOutputPortWidth(S,2,MBSdata->nbody);

	/* Port 3 : qd */
	ssSetOutputPortWidth(S,3,MBSdata->nbody);

	/* Port 4 : qdd */
	ssSetOutputPortWidth(S,4,MBSdata->nbody);

	return 1;
}

void sf_get_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{
	int i; 
	bool vini_from_past;

	InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0); //q
	InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1); //qd
	InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S,2); //qdd
	InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs(S,3); //Qq

	/* Inputs */ 
#ifdef MATLAB_MEX_FILE
	vini_from_past = (bool) mxGetScalar(ssGetSFcnParam(S,1)); // get 2nd S-function parameter
#else
	vini_from_past = true;
#endif

	if (vini_from_past)
	{
		/* Port 0 : independent and driven variables only */
		for(i=1;i<=lds->iquc[0];i++)
			MBSdata->q[lds->iquc[i]] = *uPtrs0[lds->iquc[i]-1];
	}
	else
	{
		/* Port 0 : q */
		for(i=1;i<=MBSdata->nbody;i++)
			MBSdata->q[i] = *uPtrs0[i-1];
	}

	for(i=1;i<=lds->iquc[0];i++) // independent and driven variables only
	{
		/* Port 1 : qd */
		MBSdata->qd[lds->iquc[i]] = *uPtrs1[lds->iquc[i]-1];
		/* Port 2 : qdd */		
		MBSdata->qdd[lds->iquc[i]] = *uPtrs2[lds->iquc[i]-1];
	}

	/* Port 3 : Qq */
	if (ssGetInputPortConnected(S,3)) {
		for(i=1;i<=MBSdata->nbody;i++)
			MBSdata->Qq[i] = *uPtrs3[i-1];
	}
}

void sf_set_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{
	int i;

	real_T *y0 = ssGetOutputPortRealSignal(S,0); /* Qa */
	real_T *y1 = ssGetOutputPortRealSignal(S,1); /* Qc */
	real_T *y2 = ssGetOutputPortRealSignal(S,2); /* q */
	real_T *y3 = ssGetOutputPortRealSignal(S,3); /* qd */
	real_T *y4 = ssGetOutputPortRealSignal(S,4); /* qdd */

	/* Port 0: Qa */
	if (ssGetOutputPortConnected(S,0))
		for(i=1;i<=MBSdata->nqa;i++)
			y0[i-1] = lds->Qact[i];

	/* Port 1: Qc */ 
	if (ssGetOutputPortConnected(S,1))
		for(i=1;i<=MBSdata->nqc;i++)
			y1[i-1] = lds->Qc[i];

	/* Port 2: q   */ 
	if (ssGetOutputPortConnected(S,2))
		for(i=1;i<=MBSdata->nbody;i++)
			y2[i-1] = MBSdata->q[i]; 

	/* Port 3: qd  */ 
	if (ssGetOutputPortConnected(S,3))
		for(i=1;i<=MBSdata->nbody;i++) 
			y3[i-1] = MBSdata->qd[i]; 

	/* Port 4: qdd */ 
	if (ssGetOutputPortConnected(S,4))
		for(i=1;i<=MBSdata->nbody;i++) 
			y4[i-1] = MBSdata->qdd[i];
}
#elif defined SENSORKIN
int sf_set_input_sizes(SimStruct *S, MBSdataStruct *MBSdata)
{
	int ninput = SF_NINPUT;

	if (!ssSetNumInputPorts(S, ninput)) return 0;

	/* INPUTS */

	/* Port 0 : q */
	ssSetInputPortWidth(S,0,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 0, 1);

	/* Port 1 : qd */
	ssSetInputPortWidth(S,1,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 1, 1);

	/* Port 2 : qdd */
	ssSetInputPortWidth(S,2,MBSdata->nbody);
	ssSetInputPortDirectFeedThrough(S, 2, 1);

	return 1;
}

int sf_set_output_sizes(SimStruct *S, MBSdataStruct *MBSdata)
{
	int noutput = SF_NOUTPUT;
	int i;//, dims[3];
	const int nsensor = mxGetNumberOfElements(ssGetSFcnParam(S,2)); // get 3rd S-function parameter size
	const bool with_OM  = (bool) mxGetScalar(ssGetSFcnParam(S,3)); // get 4th S-function parameter
	const bool with_OMD = (bool) mxGetScalar(ssGetSFcnParam(S,4)); // get 5th S-function parameter
	//	const bool with_J   = (bool) mxGetScalar(ssGetSFcnParam(S,5)); // get 6th S-function parameter
	const bool with_R   = (bool) mxGetScalar(ssGetSFcnParam(S,6)); // get 7th S-function parameter
	//	DECL_AND_INIT_DIMSINFO(di);

	if (nsensor) {
		if (with_OM)  noutput++;
		if (with_OMD) noutput++;
		//		if (with_J)   noutput++;
		if (with_R)   noutput++;

		if (!ssSetNumOutputPorts(S, noutput)) return 0; 

		/* OUTPUTS */

		/* Port 0 : P */
		ssSetOutputPortWidth(S,0,3*nsensor);

		/* Port 1 : V */
		ssSetOutputPortWidth(S,1,3*nsensor);

		/* Port 2 : A */
		ssSetOutputPortWidth(S,2,3*nsensor);

		i = 3;
		/* Port i : OM */
		if (with_OM) {
			ssSetOutputPortWidth(S,i,3*nsensor);
			i++;
		}

		/* Port i : OMD */
		if (with_OMD) {
			ssSetOutputPortWidth(S,i,3*nsensor);
			i++;
		}

		/* Port i : J *
		if (with_J) {
		di.numDims = 3;
		dims[0] = 6;
		dims[1] = MBSdata->nbody;
		dims[2] = nsensor;
		di.dims = &dims[0];
		di.width = dims[0]*dims[1]*dims[2];
		ssSetOutputPortDimensionInfo(S, i, &di);
		i++;
		}*/

		/* Port i : R */
		if (with_R) {
			/*			di.numDims = 3;
			dims[0] = 3;
			dims[1] = 3;
			dims[2] = nsensor;
			di.dims = &dims[0];
			di.width = dims[0]*dims[1]*dims[2];
			ssSetOutputPortDimensionInfo(S, i, &di);
			*/
			ssSetOutputPortWidth(S,i,3*3*nsensor);
		}

	}
	else
		if (!ssSetNumOutputPorts(S, 0)) return 0; 

	return 1;
}

void sf_get_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{
	int i; 

	InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0); //q
	InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1); //qd
	InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S,2); //qdd

	/* Inputs */ 

	for(i=1;i<=MBSdata->nbody;i++) {
		/* Port 0 : q */
		MBSdata->q[i] = *uPtrs0[i-1];

		/* Port 1 : qd */
		MBSdata->qd[i] = *uPtrs1[i-1];

		/* Port 2 : qdd */
		MBSdata->qdd[i] = *uPtrs2[i-1];
	}

}

void sf_set_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{
	int i,j;
	const int nsensor = mxGetNumberOfElements(ssGetSFcnParam(S,2)); // get 3rd S-function parameter size
	const bool with_OM  = (bool) mxGetScalar(ssGetSFcnParam(S,3)); // get 4th S-function parameter
	const bool with_OMD = (bool) mxGetScalar(ssGetSFcnParam(S,4)); // get 5th S-function parameter
	//	const bool with_J   = (bool) mxGetScalar(ssGetSFcnParam(S,5)); // get 6th S-function parameter
	const bool with_R   = (bool) mxGetScalar(ssGetSFcnParam(S,6)); // get 7th S-function parameter

	real_T *y0 = ssGetOutputPortRealSignal(S,0); /* P */
	real_T *y1 = ssGetOutputPortRealSignal(S,1); /* V */
	real_T *y2 = ssGetOutputPortRealSignal(S,2); /* A */


	for(i=1;i<=nsensor;i++) {

		/* Port 0: P */
		if (ssGetOutputPortConnected(S,0)) {
			y0[(i-1)*3]   = lds->psensorStruct[i]->P[1];
			y0[(i-1)*3+1] = lds->psensorStruct[i]->P[2];
			y0[(i-1)*3+2] = lds->psensorStruct[i]->P[3];
		}
		/* Port 1: V */ 
		if (ssGetOutputPortConnected(S,1)) {
			y1[(i-1)*3]   = lds->psensorStruct[i]->V[1];
			y1[(i-1)*3+1] = lds->psensorStruct[i]->V[2];
			y1[(i-1)*3+2] = lds->psensorStruct[i]->V[3];
		}
		/* Port 2: A */ 
		if (ssGetOutputPortConnected(S,2)) {
			y2[(i-1)*3]   = lds->psensorStruct[i]->A[1];
			y2[(i-1)*3+1] = lds->psensorStruct[i]->A[2];
			y2[(i-1)*3+2] = lds->psensorStruct[i]->A[3];
		}
	}

	j = 3;
	/* Port j: OM */
	if (with_OM) {
		real_T *yi = ssGetOutputPortRealSignal(S,j);
		for(i=1;i<=nsensor;i++) {
			yi[(i-1)*3]   = lds->psensorStruct[i]->OM[1];
			yi[(i-1)*3+1] = lds->psensorStruct[i]->OM[2];
			yi[(i-1)*3+2] = lds->psensorStruct[i]->OM[3];
		}
		j++;
	}

	/* Port j: OMD */
	if (with_OMD) {
		real_T *yi = ssGetOutputPortRealSignal(S,j);
		for(i=1;i<=nsensor;i++) {
			yi[(i-1)*3]   = lds->psensorStruct[i]->OMP[1];
			yi[(i-1)*3+1] = lds->psensorStruct[i]->OMP[2];
			yi[(i-1)*3+2] = lds->psensorStruct[i]->OMP[3];
		}
		j++;
	}

	/* Port j: R */
	if (with_R) {
		real_T *yi = ssGetOutputPortRealSignal(S,j);
		for(i=1;i<=nsensor;i++) {
			yi[(i-1)*9]   = lds->psensorStruct[i]->R[1][1];
			yi[(i-1)*9+1] = lds->psensorStruct[i]->R[2][1];
			yi[(i-1)*9+2] = lds->psensorStruct[i]->R[3][1];
			yi[(i-1)*9+3] = lds->psensorStruct[i]->R[1][2];
			yi[(i-1)*9+4] = lds->psensorStruct[i]->R[2][2];
			yi[(i-1)*9+5] = lds->psensorStruct[i]->R[3][2];
			yi[(i-1)*9+6] = lds->psensorStruct[i]->R[1][3];
			yi[(i-1)*9+7] = lds->psensorStruct[i]->R[2][3];
			yi[(i-1)*9+8] = lds->psensorStruct[i]->R[3][3];
		}
	}


}
#endif
