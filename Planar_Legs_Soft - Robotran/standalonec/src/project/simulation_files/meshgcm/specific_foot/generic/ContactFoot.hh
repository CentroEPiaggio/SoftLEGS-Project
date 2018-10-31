/*! 
 * \author Nicolas Van der Noot
 * \file ContactFoot.hh
 * \brief ContactFoot class
 */

#ifndef _CONTACT_FOOT_HH_
#define _CONTACT_FOOT_HH_

#include "LowFilterSimu.hh"

#include <cmath>
#ifdef _CHAR16T // math.h version
#define CHAR16_T // visual version
#endif

#include "MBSdataStruct.h"


//#include <iostream>

// GCM contact points state
enum{STICTION_GCM, SLIDING_GCM, SWING_GCM};

// filters
enum {LOW_FILT_FN, LOW_FILT_FX, LOW_FILT_FY, LOW_FILT_VX, LOW_FILT_VY, NB_LOW_FILT_MESH};

/*! \brief Generic model of a contact foot
 */
class ContactFoot
{
	public:
		ContactFoot(MBSdataStruct *MBSdata, int nb_points, int S_sens_id);
		virtual ~ContactFoot();

		void update_pose(double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4]);
		void update_pose_Fsens();

		void update_low_pass_params(double new_t);

		double get_F_tot(int id_axis) { return F_tot[id_axis]; }		
		double get_T_tot(int id_axis) { return T_tot[id_axis]; }

		int get_nb_points() const { return nb_points; }

		double** get_rn() const { return rn; }		
		double** get_rs() const { return rs; }		

		int* get_state() const { return state; }

		double* get_Fn_lp() const { return Fn_lp; }
		double* get_fx_lp() const { return fx_lp; }
		double* get_fy_lp() const { return fy_lp; }
		double* get_vx_lp() const { return vx_lp; }
		double* get_vy_lp() const { return vy_lp; }

		double get_P(int i) const { return P[i]; }
		double get_V(int i) const { return V[i]; }
		double get_OM(int i) const { return OM[i]; }
		double get_R(int i, int j) const { return R[i][j]; }

		LowFilterSimu** get_low_filters() { return low_filters; }

		void set_F_tot(int id_axis, double value) { F_tot[id_axis] = value; }
		void set_T_tot(int id_axis, double value) { T_tot[id_axis] = value; }

	protected:
		MBSdataStruct *MBSdata; ///< Robotran structure

		int nb_joints; ///< number of joints in Robotran
		int S_sens_id; ///< S sensor ID in Robotran
		MBSsensorStruct S_Sens; ///< Robotran Sensor structure

		double **rn; ///< position vector of mesh points in body coordinate system (attached to the foot) [m]
		double **rs; ///< contact point stored (absolute) [m]

		int *state; ///< state of the foot: STICTION_GCM, SLIDING_GCM, NO_CONTACT_GCM

		int nb_points; ///< number of mesh points [-]

		double *Fn_lp; ///< low-pass value of the normal force [N]
		double *fx_lp; ///< low-pass value of the tangential x force [N]
		double *fy_lp; ///< low-pass value of the tangential y force [N]
		double *vx_lp; ///< low-pass value of the tangential x speed [m/s]
		double *vy_lp; ///< low-pass value of the tangential y speed [m/s]

		LowFilterSimu *low_filters[NB_LOW_FILT_MESH]; ///< low filters for 'Fn_lp', 'fx_lp', 'fy_lp', 'vx_lp' and 'vy_lp'

		double P[3];    ///< position vector (absolute) [m]
		double V[3];    ///< velocity vector (absolute) [m/s]
		double OM[3];   ///< orientation derivative [rad/s]
		double R[3][3]; ///< absolute rotation matrix [-]

		double F_tot[3]; ///< total force applied on this body [N]
		double T_tot[3]; ///< total torque applied on this body [Nm]
};

#endif
