
#include "WholeRectFeet.hh"
#include "RectFoot.hh"
#include "simu_def.h"

#define NB_FEET_BODIES 2

enum {RIGHT_ID, LEFT_ID};

/*! \brief contructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] ground ground model
 */
WholeRectFeet::WholeRectFeet(MBSdataStruct *MBSdata, GroundModel *ground): WholeFeet(MBSdata, ground)
{
	for(int i=0; i<NB_FEET_BODIES; i++)
	{
		switch (i)
		{
            case RIGHT_ID : bodies.push_back(new RectFoot(MBSdata, MBSdata->Nsensor + RFOOT_FSENS_ID, 0)); break;
            case LEFT_ID  : bodies.push_back(new RectFoot(MBSdata, MBSdata->Nsensor + LFOOT_FSENS_ID, 1)); break;

			default: break;
		}
	}
}

/*! \brief destructor
 */
WholeRectFeet::~WholeRectFeet()
{
	// delete already done in mother class
}

/*! \brief compute the external ground reactions related to this foot
 * 
 * \param[out] F_tot external force output [N]
 * \param[out] T_tot external torque output [Nm]
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] index index of the external sensor
 */
void WholeRectFeet::compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index)
{
	ContactFoot *body;

	switch ( index ) 
	{
		case RFOOT_FSENS_ID:
			body = bodies[RIGHT_ID];
			break;

		case LFOOT_FSENS_ID:
			body = bodies[LEFT_ID];
			break;
	
		default:
			body = NULL;
			break;
	}

	body->update_pose(PxF, RxF, VxF, OMxF);

	ground->compute_F_T(F_tot, T_tot, body);
}

/*! \brief get feet forces
 * 
 * \param[in] leg_id ID of the leg
 * \param[in] axis axis requested
 * \return force requested [N]
 */
double WholeRectFeet::get_leg_feet_forces(int leg_id, int axis)
{
	switch (leg_id)
	{
		case RIGHT_ID:
			return bodies[RIGHT_ID]->get_F_tot(axis);
			break;

		case LEFT_ID:
			return bodies[LEFT_ID]->get_F_tot(axis);
			break;
	
		default:
			return 0.0;
			break;
	}
}

/*! \brief get feet torques
 * 
 * \param[in] leg_id ID of the leg
 * \param[in] axis axis requested
 * \return torque requested [Nm]
 */
double WholeRectFeet::get_leg_feet_torques(int leg_id, int axis)
{
	switch (leg_id)
	{
		case RIGHT_ID:
			return bodies[RIGHT_ID]->get_T_tot(axis);
			break;

		case LEFT_ID:
			return bodies[LEFT_ID]->get_T_tot(axis);
			break;
	
		default:
			return 0.0;
			break;
	}
}
