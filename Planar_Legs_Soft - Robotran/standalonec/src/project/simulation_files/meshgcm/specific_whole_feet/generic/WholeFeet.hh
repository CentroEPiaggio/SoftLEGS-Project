/*! 
 * \author Nicolas Van der Noot
 * \file WholeFeet.hh
 * \brief WholeFeet class
 */

#ifndef _WHOLE_FEET_HH_
#define _WHOLE_FEET_HH_

#include "ContactFoot.hh"
#include "MBSdataStruct.h"
#include "GroundModel.hh"

#include <vector>

/*! \brief Whole model of the feet (generic class)
 */
class WholeFeet
{
	public:
		WholeFeet(MBSdataStruct *MBSdata, GroundModel *ground);
		virtual ~WholeFeet();

		virtual void compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index) = 0;
		
		void state_compute();

		virtual double get_leg_feet_forces(int leg_id, int axis)  = 0;
		virtual double get_leg_feet_torques(int leg_id, int axis) = 0;

		/// get total number of bodies
		inline int get_nb_bodies() const { return bodies.size(); }

		/// get contact foot corresponding to the provided ID
		inline ContactFoot* get_foot_body(int p) { return bodies[p]; }

	protected:
		std::vector<ContactFoot*> bodies; ///< bodies of the feet

		MBSdataStruct *MBSdata; ///< Robotran structure
		GroundModel *ground;    ///< ground model
};

#endif
