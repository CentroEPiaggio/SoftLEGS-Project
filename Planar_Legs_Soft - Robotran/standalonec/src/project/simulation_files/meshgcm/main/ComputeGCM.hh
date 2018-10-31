/*! 
 * \author Nicolas Van der Noot
 * \file ComputeGCM.hh
 * \brief ComputeGCM class
 */

#ifndef _COMPUTE_GCM_HH_
#define _COMPUTE_GCM_HH_

#include "WholeFeet.hh"
#include "GroundModel.hh"

/*! \brief Computation of the Ground Contact Model (GCM) forces
 */
class ComputeGCM
{
	public:
		ComputeGCM(MBSdataStruct *MBSdata);
		~ComputeGCM();

		void compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index);
		void state_compute();

		/// get whole_feet
		inline WholeFeet* get_whole_feet() { return whole_feet; }

	private:
		int coman_model; ///< COMAN model

		MBSdataStruct *MBSdata; ///< Robotran structure
		WholeFeet *whole_feet;  ///< class with all the feet
		GroundModel *ground;    ///< ground model
};

#endif
