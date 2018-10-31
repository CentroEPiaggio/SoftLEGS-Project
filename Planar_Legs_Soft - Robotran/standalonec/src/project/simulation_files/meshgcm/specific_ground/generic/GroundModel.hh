/*! 
 * \author Nicolas Van der Noot
 * \file GroundModel.hh
 * \brief GroundModel class
 */

#ifndef _GROUND_MODEL_HH_
#define _GROUND_MODEL_HH_

#include "ContactFoot.hh"
#include "MBSdataStruct.h"

/*! \brief Ground model
 */
class GroundModel
{
	public:
		GroundModel(MBSdataStruct *MBSdata);
		virtual ~GroundModel();

		void compute_F_T(double F_tot[3], double T_tot[3], ContactFoot *foot);
		void state_switch(ContactFoot *foot);

		inline void update_current_foot(ContactFoot *foot);

		virtual void compute() = 0;
		virtual int contact_ground(double F[3], double r[3], double v[3], int index) = 0;
		virtual void state_ground(double r[3], double v[3], int index) = 0;

	protected:
		MBSdataStruct *MBSdata; ///< Robotran structure

		// time
		double t; ///< time [s]

		// foot properties
		int nb_points; ///< number of points in the mesh of the current foot

		double **rn; ///< current foot: position vector of mesh points in body coordinate system (attached to the foot) [m]
		double **rs; ///< current foot: contact point stored (absolute) [m]

		int *state; ///< current foot: state of the foot: STICTION_GCM, SLIDING_GCM, NO_CONTACT_GCM

		double *Fn_lp; ///< current foot: low-pass value of the normal force [N]
		double *fx_lp; ///< current foot: low-pass value of the tangential x force [N]
		double *fy_lp; ///< current foot: low-pass value of the tangential y force [N]
		double *vx_lp; ///< current foot: low-pass value of the tangential x speed [m/s]
		double *vy_lp; ///< current foot: low-pass value of the tangential y speed [m/s]

		double P[3];    ///< current foot: position vector (absolute) [m]
		double V[3];    ///< current foot: velocity vector (absolute) [m/s]
		double OM[3];   ///< current foot: orientation derivative [rad/s]
		double R[3][3]; ///< current foot: absolute rotation matrix [-]

		LowFilterSimu **low_filters; ///< current foot: low filters for 'Fn_lp', 'fx_lp', 'fy_lp', 'vx_lp' and 'vy_lp'

		/*! \brief update the low-pass filter computation
		 * 
		 * \param[in] index index of the low-pass filter
		 * \param[in,out] value value state to filter
		 * \param[in] new_val new value
		 */
		inline void update_low_pass(int index, double &value, double new_val)
		{
			low_filters[index]->update_low_pass(value, new_val);
		}

		/*! \brief compute the kinematics information of one point
		 * 
		 * \param[out] r0 absolute reference position [m]
		 * \param[out] r1 relative mesh point position [m]
		 * \param[out] r absolute mesh point position [m]
		 * \param[out] v absolute mesh point velocity [m/s]
		 * \param[int] index index of the current mesh point [-]
		 */
		inline void kinematics_point(double r0[3], double r1[3], double r[3], double v[3], int index)
		{
			// position vector of mesh points in body coordinate system (attached to the foot)
			r0[0] = rn[index][0];
			r0[1] = rn[index][1];
			r0[2] = rn[index][2];

			// position vector of mesh points in a frame attached to the body but aligned with the inertial frame
			r1[0] = R[0][0]*r0[0] + R[1][0]*r0[1] + R[2][0]*r0[2];
			r1[1] = R[0][1]*r0[0] + R[1][1]*r0[1] + R[2][1]*r0[2];
			r1[2] = R[0][2]*r0[0] + R[1][2]*r0[1] + R[2][2]*r0[2];

			// position vector of mesh points in global coordinate system (inertial frame)
			r[0] = r1[0] + P[0];
			r[1] = r1[1] + P[1];
			r[2] = r1[2] + P[2];

			// velocity vector of mesh points in global coordinate system (inertial frame)
			v[0] = V[0] - OM[2]*r1[1] + OM[1]*r1[2];
			v[1] = V[1] + OM[2]*r1[0] - OM[0]*r1[2];
			v[2] = V[2] - OM[1]*r1[0] + OM[0]*r1[1];
		}

		/*! \brief loop on all the contact points (mesh of a foot part) to extract the total force and the torque
		 * 
		 * \param[out] F_tot resulting force along the three axes
		 * \param[out] T_tot resulting torques along the three axes 
		 */
		inline void loop_mesh_F_T(double F_tot[3], double T_tot[3])
		{
			double r0[3], r1[3], r[3], v[3], F[3];

			// attention: for huge meshes, this loop can become a critical loop !
			for (int i=0; i<nb_points; i++)
			{
				kinematics_point(r0, r1, r, v, i);

				if (contact_ground(F, r, v, i))
				{
					for(int j=0; j<3; j++)
					{
						F_tot[j] += F[j];
					}

					T_tot[0] += - r1[2]*F[1] + r1[1]*F[2];
					T_tot[1] +=   r1[2]*F[0] - r1[0]*F[2];
					T_tot[2] += - r1[1]*F[0] + r1[0]*F[1];
				}
			}
		}

		/*! \brief loop on all the contact points (mesh of a foot part) to handle the points state
		 */
		inline void loop_mesh_state()
		{
			double r0[3], r1[3], r[3], v[3];

			// attention: for huge meshes, this loop can become a critical loop !
			for (int i=0; i<nb_points; i++)
			{
				kinematics_point(r0, r1, r, v, i);

				state_ground(r, v, i);
			}
		}
};

#endif
