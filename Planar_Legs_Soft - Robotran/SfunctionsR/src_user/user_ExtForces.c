//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "MBSdef.h"
#include "MBSfun.h"
#include "MBSdataStruct.h"


double* user_ExtForces(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   MBSdataStruct *MBSdata, double tsim,int ixF)
{
	double Fx=0.0, Fy=0.0, Fz=0.0;
	double Mx=0.0, My=0.0, Mz=0.0;
	double dxF[4] ={0.0, 0.0, 0.0, 0.0};

//	double rz, anglis, angcamb gliss, Vct[4], Rtsol[4][4];

	double *SWr = MBSdata->SWr[ixF];

/* Begin of user declaration */
	
	

/* End of user declaration */


	switch(ixF){

/* Begin of user code */
		case 1: 

					
		break;

	/*	case 2: 

		// compute the terms of Swr

		SWr[1]=Fx;
		SWr[2]=Fy;
		SWr[3]=Fz;
		SWr[4]=Mx;
		SWr[5]=Mz;
		SWr[6]=Mz;
		SWr[7]=dxF[1];
		SWr[8]=dxF[2];
		SWr[9]=dxF[3];

		// or call mbs_kine_wheel and user_WheelForces

		mbs_kine_wheel(PxF,RxF,
		  			   VxF, OMxF,
					   MBSdata, tsim, ixF,
					   &rz, &anglis, &angcamb,
					   &gliss, Vct, Rtsol, dxF);

		user_WheelForces(rz, anglis, angcamb,
						 gliss, Vct[3], dxF,
						 MBSdata, tsim, ixF,
						 SWr);
			
		break;
	*/
		
/* End of user code */

	}

	return SWr;
}

 