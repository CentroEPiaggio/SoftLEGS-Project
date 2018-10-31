/*! 
 * \author Nicolas Van der Noot
 * \file EquationGround.hh
 * \brief EquationGround class
 */

#ifndef _EQUATION_GROUND_HH_
#define _EQUATION_GROUND_HH_

#include "GroundModel.hh"

/*! \brief specific ground model: ground described with equations
 */
class EquationGround: public GroundModel
{
	public:
		EquationGround(MBSdataStruct *MBSdata);
		virtual ~EquationGround();

		virtual void compute() = 0;
		virtual int contact_ground(double F[3], double r[3], double v[3], int index);
		virtual void state_ground(double r[3], double v[3], int index);

	protected:
		inline void ground_rot_matrix(double gcm_hx, double gcm_hy, double R_g[3][3]);
		virtual void ground_description(double x, double y, double *h, double *hx, double *hy) = 0;
};

#endif
