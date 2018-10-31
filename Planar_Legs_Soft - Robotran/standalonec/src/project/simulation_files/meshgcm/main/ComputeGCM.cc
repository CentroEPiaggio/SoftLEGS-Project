
#include "ComputeGCM.hh"
#include "WholeRectFeet.hh"

#include "FlatGround.hh"
#include "LinearGround.hh"

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 */
ComputeGCM::ComputeGCM(MBSdataStruct *MBSdata)
{
	this->MBSdata = MBSdata;

	ground = new FlatGround(MBSdata);

	whole_feet = new WholeRectFeet(MBSdata, ground);
}

/*! \brief destructor
 */
ComputeGCM::~ComputeGCM()
{
	delete whole_feet;

	delete ground;
}

/*! \brief compute the external ground reactions related to one ext force sensor
 * 
 * \param[out] F_tot external force output [N]
 * \param[out] T_tot external torque output [Nm]
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] index index of the xetrnal sensor
 */
void ComputeGCM::compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index)
{
	whole_feet->compute_F_T(F_tot, T_tot, PxF, RxF, VxF, OMxF, index);
}

/*! \brief computation done once during a simulation time step
 */
void ComputeGCM::state_compute()
{
	whole_feet->state_compute();
}
