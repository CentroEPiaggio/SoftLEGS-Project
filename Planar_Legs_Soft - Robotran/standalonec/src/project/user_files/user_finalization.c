//---------------------------
// Nicolas Van der Noot
//
// Creation : 03/06/2014
//
// call at the end of the simulation
//
// Beware: this function is only called for the Standalone version, 
//         not for the Simulink version !
//
//---------------------------

#ifdef STANDALONE

#include "MBSdataStruct.h"
#include "gcm_interface.h"
#include "contact_interface.h"

//#include "simu_def.h"

/*
 * User function called at the end of the simulation
 */
void user_finalization(MBSdataStruct *MBSdata)
{
	// GCM
	#ifdef MESH_GSM
	free_mesh_gcm(MBSdata);
	#endif

	// 3D contact
	close_contact_geom(MBSdata);

	// controller
    //controller_close_interface(MBSdata);

}
#endif
