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
#include "MBSdataStruct.h"


double user_LinkForces(double Z, double Zd, MBSdataStruct *MBSdata, double tsim, int ilnk)
{
	double Fspring = 0;
	double Fdamper = 0;

/* begin user declaration */

//	double STIFF, DAMP, Z0;

/* end user declaration */

	
	switch(ilnk){
/* begin user code */

	/* sample code */
		case 1:
/*			// User model Parameters
			STIFF = MBSdata->user_model.MYMODEL.K;
			DAMP = MBSdata->user_model.MYMODEL.D;
			Z0 = MBSdata->user_model.MYMODEL.Z0;
			
			// Link Equations
			Fspring = STIFF*(Z-Z0);
			Fdamper = DAMP*Zd; 
*/
		break;

	/*	case 2 :
			...
		break; 
	*/

/* end user code */
	}
		
	return Fspring + Fdamper;
}
