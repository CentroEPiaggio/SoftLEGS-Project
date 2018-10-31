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

#if defined DIRDYNARED || defined ACCELRED
void sf_InitCond(SimStruct *S, MBSdataStruct *MBSdata)
{
	int i; 
	real_T *x0 = ssGetContStates(S);

	for(i=1;i<=MBSdata->nqu;i++) 
	{ 
		x0[i-1] = MBSdata->q[MBSdata->qu[i]]; 
		x0[MBSdata->nqu+i-1] = MBSdata->qd[MBSdata->qu[i]]; 
	} 
	for(i=1;i<=MBSdata->Nux;i++) 
	{ 
		x0[i+2*MBSdata->nqu-1] = MBSdata->ux[i]; 
	} 
}
/*#elif defined INVDYNARED
void sf_InitCond(SimStruct *S, MBSdataStruct *MBSdata)
{
	int i; 

/*	if (q_ini) // initialize independent joint position and velocity from additional S-function inputs
	{
		InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,0);
		InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs(S,1);

		for(i=1;i<=MBSdata->nqu;i++) 
		{ 
			MBSdata->q[MBSdata->qu[i]]  = *uPtrs0[i-1]; 
			MBSdata->qd[MBSdata->qu[i]] = *uPtrs1[i-1]; 
		} 
/*		for(i=1;i<=MBSdata->Nux;i++) 
		{ 
			MBSdata->ux[i] = *uPtrs2[i-1]; // PLUS TARD ???
		}
*
	}*
}*/
#endif