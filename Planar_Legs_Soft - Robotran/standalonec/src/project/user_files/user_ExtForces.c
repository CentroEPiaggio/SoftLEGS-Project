//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 

#include "MBSdataStruct.h"

#ifdef SIMBODY
#include "simbody_functions.h"
#include "simbody_cpp_functions.h"
#endif

#include "simu_def.h"
#include "gcm_interface.h"
#include "contact_interface.h"

// limiting external forces
#define MAX_EXT_FORCES  10000.0
#define MAX_EXT_MOMENTS 10000.0

double* user_ExtForces(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   MBSdataStruct *MBSdata, double tsim, int ixF)
{
	int i;
	double Fx=0.0, Fy=0.0, Fz=0.0;
	double Mx=0.0, My=0.0, Mz=0.0;
	double dxF[3+1] ={0.0, 0.0, 0.0, 0.0}; // +1 because indexes begin at 1

	#ifdef SIMBODY
	SimbodyBodiesStruct *simbodyBodies;
	#endif

	double *SWr = MBSdata->SWr[ixF];
	int idpt = 0;

	idpt = MBSdata->xfidpt[ixF];

	dxF[1] = MBSdata->dpt[1][idpt];
	dxF[2] = MBSdata->dpt[2][idpt];
	dxF[3] = MBSdata->dpt[3][idpt];



	#ifdef SIMBODY
	
	//compute all the forces at once (arbitrary for 1st external force) (else compute same thing for each force sensors)
	if(ixF == 1)   
	{
		// 1) Simbody receives kinematics from Robotran
		update_simbody_kinematics(MBSdata->user_IO->simbodyStruct->simbodyBodies, MBSdata);

		// 2) Simbody computes contact force
		loop_Simbody(MBSdata->user_IO->simbodyStruct);
	}

	// 3) Simbody sends contact force to robotran dynamics
	simbodyBodies = MBSdata->user_IO->simbodyStruct->simbodyBodies;
	for(i=0; i<simbodyBodies->nb_contact_bodies; i++)
	{
	 	if (ixF == simbodyBodies->F_sensor_Robotran_index[i])
	 	{
	 		Fx += simbodyBodies->force_bodies[i][0];
	 		Fy += simbodyBodies->force_bodies[i][1];
	 		Fz += simbodyBodies->force_bodies[i][2];

	 		Mx += simbodyBodies->torque_bodies[i][0];
	 		My += simbodyBodies->torque_bodies[i][1];
	 		Mz += simbodyBodies->torque_bodies[i][2];

	 		break;
	 	}
	}

	#endif
	

	// initializes the terms of Swr
	SWr[1] = Fx;
	SWr[2] = Fy;
	SWr[3] = Fz;
	SWr[4] = Mx;
	SWr[5] = My;
	SWr[6] = Mz;
	SWr[7] = dxF[1];
	SWr[8] = dxF[2];
	SWr[9] = dxF[3];

    #ifndef SIMBODY
    if((ixF == RFOOT_FSENS_ID) || (ixF == LFOOT_FSENS_ID))
    {
    	// update the contact primitives model: kinematics and force-torque of the shapes
		if (ixF == 1)
		{
		    update_contact_geom(MBSdata);
		}

		// apply force-torque computed by the contact primitives model
		apply_contact_geom(MBSdata, ixF, &Fx, &Fy, &Fz, &Mx, &My, &Mz);

		SWr[1] = Fx;
		SWr[2] = Fy;
		SWr[3] = Fz;

		SWr[4] = Mx;
		SWr[5] = My;
		SWr[6] = Mz;

        // Compute thing resultant forces and moments from the ground
        #ifdef MESH_GSM
        gcm_mesh_compute_F_T(MBSdata, SWr, PxF, RxF, VxF, OMxF, ixF);
        #endif

        // Limit external forces
        for (i=1; i<=3; i++)
        {
            SWr[i] = (SWr[i] >  MAX_EXT_FORCES) ?  MAX_EXT_FORCES : SWr[i];
            SWr[i] = (SWr[i] < -MAX_EXT_FORCES) ? -MAX_EXT_FORCES : SWr[i];
        }

        // Limit external moments
        for (i=4; i<=6; i++)
        {
            SWr[i] = (SWr[i] >  MAX_EXT_MOMENTS) ?  MAX_EXT_MOMENTS : SWr[i];
            SWr[i] = (SWr[i] < -MAX_EXT_MOMENTS) ? -MAX_EXT_MOMENTS : SWr[i];
        }
    }
    #endif

	return SWr;
}
