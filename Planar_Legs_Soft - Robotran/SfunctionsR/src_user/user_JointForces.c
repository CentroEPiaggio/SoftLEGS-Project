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


double* user_JointForces(MBSdataStruct *MBSdata, double tsim)
{


	return MBSdata->Qq;
}
