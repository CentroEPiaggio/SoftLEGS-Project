
#include "ComputeGCM.hh"
#include "gcm_interface.h"

/*! \brief get the class ComputeGCM
 * 
 * \param[in,out] MBSdata Robotran structure
 * \return ComputeGCM class
 */
ComputeGCM* get_ComputeGCM(MBSdataStruct *MBSdata)
{
	return static_cast<ComputeGCM*>(MBSdata->user_IO->compute_gcm);
}

/*! \brief intialize the GCM
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void init_mesh_gcm(MBSdataStruct *MBSdata)
{
	MBSdata->user_IO->compute_gcm = new ComputeGCM(MBSdata);
}

/*! \brief release the GCM
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void free_mesh_gcm(MBSdataStruct *MBSdata)
{
	delete get_ComputeGCM(MBSdata);
}

/*! \brief compute the external ground reactions related to one ext force sensor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[out] F_tot external force output [N]
 * \param[out] T_tot external torque output [Nm]
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] index index of the xetrnal sensor
 */
void gcm_mesh_compute_F_T(MBSdataStruct *MBSdata, double *SWr, double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index)
{
	double F_tot[3], T_tot[3];

	get_ComputeGCM(MBSdata)->get_whole_feet()->compute_F_T(F_tot, T_tot, PxF, RxF, VxF, OMxF, index);

	for(int i=0; i<3; i++)
	{
		SWr[1+i] = F_tot[i];
		SWr[4+i] = T_tot[i];
	}
}

/*! \brief computation done once during a simulation time step
 *
 * \param[in] MBSdata Robotran structure
 */
void gcm_mesh_state_compute(MBSdataStruct *MBSdata)
{
	get_ComputeGCM(MBSdata)->get_whole_feet()->state_compute();
}
