/*===========================================================================*
  *
  *  user_sf_IO.c
  *	
  *  Project:	Planar_Legs
  * 
  *  Generation date: 13-Jun-2016 13:22:06
  * 
  *  (c) Universite catholique de Louvain
  *      Dpartement de Mcanique 
  *      Unit de Production Mcanique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#include "MBSfun.h" 
#include "user_sf_IO.h" 
#include "sfdef.h" 
#include "userDef.h"



UserIOStruct * initUserIO(MBSdataStruct *s, const mxArray *uvp)
{
	UserIOStruct *uvs;
	int i=0;
	//
	uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));
	

    return uvs;
}

void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{



	free(uvs);

}
void sf_set_user_input_sizes(SimStruct *S, MBSdataStruct *MBSdata, int sf_ninput) 
{ 
	if (SF_N_USER_INPUT > 0) { // warning: index starts at sf_ninput 
        // example: ssSetInputPortWidth(S,sf_ninput,10); 
	} 
} 

void sf_set_user_output_sizes(SimStruct *S, MBSdataStruct *MBSdata) 
        // example: ssSetOutputPortWidth(S, SF_NOUTPUT, 10); 
{ 
	if (SF_N_USER_OUTPUT > 0) { // warning: index starts at SF_NOUTPUT 
	} 
} 

void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput) 
{ 
    // warning: index starts at sf_ninput
    // example: InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);
    //          MBSdata->user_IO->var1 = *uPtrs0[0];
} 

void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
    // warning: index starts at SF_NOUTPUT  
    // example: real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
    //          *y0 = MBSdata->user_IO->var1;  
} 
void storeUserIOStruct(UserIOStruct *uvs, mxArray *uvp)
{ 
    int i;
