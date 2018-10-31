/*! 
 * \author Nicolas Van der Noot
 * \file FlatGround.hh
 * \brief FlatGround class
 */

#ifndef _FLAT_GROUND_HH_
#define _FLAT_GROUND_HH_

#include "GroundModel.hh"

/*! \brief specific ground model: flat ground
 */
class FlatGround: public GroundModel
{
	public:
		FlatGround(MBSdataStruct *MBSdata);
		virtual ~FlatGround();

		virtual void compute();
		virtual int contact_ground(double F[3], double r[3], double v[3], int index);
		virtual void state_ground(double r[3], double v[3], int index);
};

#endif
