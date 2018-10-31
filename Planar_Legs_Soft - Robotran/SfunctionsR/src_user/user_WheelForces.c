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


void user_WheelForces(double rz, double anglis, double angcamb,
	                  double gliss, double Vctz, double dxF[4],
					  MBSdataStruct *MBSdata, double tsim, int iwhl,
					  double* SWr)
{
	double Fwhl[4]={0.0, 0.0, 0.0, 0.0};
	double Mwhl[4]={0.0, 0.0, 0.0, 0.0};

/* Begin of user declaration */
	
	double rnom, K = 2e5;

/* End of user declaration */
	
	switch(iwhl){
/* Begin of user code */
		case 1: 
			// Sample code:
			// K=MBSdata->user_model.mymodel.K;
		break;
		
	/*	case 2: 
			...
		break;
	*/
		
/* End of user code */

	}

	// Tyre characteristics
	rnom = MBSdata->rnom[iwhl];

	if(MBSdata->rnom[iwhl] > rz)
	{
	// modèle ressort

		Fwhl[3] = K* (rnom - rz)*cos(angcamb);

	// modèle Calspan

		//mbs_calspan(Fwhl,Mwhl, anglis,angcamb);
		
	// modèle Bakker

		//mbs_bakker(Fwhl,Mwhl,anglis,angcamb,gliss);

	}

	SWr[1] = Fwhl[1];
	SWr[2] = Fwhl[2];
	SWr[3] = Fwhl[3];
	SWr[4] = Mwhl[1];
	SWr[5] = Mwhl[2];
	SWr[6] = Mwhl[3];
	SWr[7] = dxF[1];
	SWr[8] = dxF[2];
	SWr[9] = dxF[3];


	// Sample code:
	// MBSdata->user_var.FWheel_rad[iwhl] = whlWr[3];
	// MBSdata->user_var.FWheel_lat[iwhl] = whlWr[2];
	// MBSdata->user_var.FWheel_long[iwhl] = whlWr[1];
}