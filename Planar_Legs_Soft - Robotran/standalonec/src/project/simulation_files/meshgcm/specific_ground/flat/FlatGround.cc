
#include "FlatGround.hh"

// vertical contribution
#define KP_Z_GCM 8.15e4 ///< normal stiffnes coeff [N/m]
#define KD_Z_GCM 3.0e1  ///< normal damping contribution [s/m]

// horizontal contribution
#define KP_XY_GCM 8.2e3 ///< tangential stiffness coeff [N/m]
#define KD_XY_GCM 3.0e1 ///< tangential damping contribution [s/m]
#define V_THRES_GCM 1.0e-2 ///< threshold speed to switch from sliding to stiction

// mu friction
#define MU_SL_GCM 0.8 ///< sliding friction coeff (mu) [-]
#define MU_ST_GCM 0.9 ///< static friction coeff (mu) [-]

// numerical safety
#define DIST_STICTION_SAFETY 1.0e-4 ///< minimal distance between two succesive stiction points [m] (safety against numerical issues)

// inline functions
inline double norm_2D(double dx, double dy){ return sqrt(dx*dx + dy*dy); }

inline double sgn(double x){ return (x > 0.0) ? 1.0 : (x < 0.0) ? -1.0 : 0.0; }

inline double pos(double x){ return (x > 0.0) ? x : 0.0; }

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info information coming from the sensors
 */
FlatGround::FlatGround(MBSdataStruct *MBSdata): GroundModel(MBSdata)
{

}

/*! \brief destructor
 */
FlatGround::~FlatGround()
{

}

/*! \brief compute the force on one point of the mesh
 * 
 * \param[out] F force applied on this point [N]
 * \param[in] r point absolute position [m]
 * \param[in] r point absolute velocity [m/s]
 * \param[in] index index of the point in the mesh
 * \return 1 if contact, 0 otherwise
 */
inline int FlatGround::contact_ground(double F[3], double r[3], double v[3], int index)
{
	double delta_x, delta_y, denom, fr, vr;

	if (r[2] <= 0) // contact with the ground
	{
		// -- normal force -- //

		F[2] = -KP_Z_GCM * r[2] * pos(1.0 - KD_Z_GCM * v[2]);


		// -- tangential force -- //

		if (state[index] == STICTION_GCM) // friction: stiction force
		{
			// shift motion
			delta_x = r[0] - rs[index][0];
			delta_y = r[1] - rs[index][1];

			F[0] = - KP_XY_GCM * delta_x * pos( 1.0 + KD_XY_GCM * sgn(delta_x) * v[0] );
			F[1] = - KP_XY_GCM * delta_y * pos( 1.0 + KD_XY_GCM * sgn(delta_y) * v[1] );
		}
		else // friction: sliding force
		{
			// speed norm
			vr = norm_2D(v[0], v[1]);

			// safety
			if (!vr) { return 1; }

			denom = 1.0 / vr;

			fr = MU_SL_GCM * F[2];

			F[0] = -fr * v[0] * denom;
			F[1] = -fr * v[1] * denom;
		}

		return 1;
	}
	else // no contact with the ground
	{
		for(int i=0; i<3; i++)
		{
			F[i] = 0.0;
		}

		return 0;
	}
}

/*! \brief handle the state changes on one point of the mesh
 * 
 * \param[in] r point absolute position [m]
 * \param[in] r point absolute velocity [m/s]
 * \param[in] index index of the point in the mesh
 */
void FlatGround::state_ground(double r[3], double v[3], int index)
{
	double delta_x, delta_y, denom, Fn, fr, vr;

	update_low_pass(LOW_FILT_VX, vx_lp[index], v[0]);
	update_low_pass(LOW_FILT_VY, vy_lp[index], v[1]);

	if (r[2] <= 0) // contact with the ground
	{
		// -- normal force -- //

		Fn = -KP_Z_GCM * r[2] * pos(1.0 - KD_Z_GCM * v[2]);

		update_low_pass(LOW_FILT_FN, Fn_lp[index], Fn);


		// -- new ground contact -- //

		if (state[index] == SWING_GCM)
		{
			state[index] = SLIDING_GCM;
		}


		// -- tangential force -- //

		if (state[index] == STICTION_GCM) // friction: stiction
		{
			// shift motion
			delta_x = r[0] - rs[index][0];
			delta_y = r[1] - rs[index][1];

			update_low_pass(LOW_FILT_FX, fx_lp[index], delta_x * pos(1.0 + KD_XY_GCM * sgn(delta_x) * v[0]) );
			update_low_pass(LOW_FILT_FY, fy_lp[index], delta_y * pos(1.0 + KD_XY_GCM * sgn(delta_y) * v[1]) );

			// stiction to sliding
			if (norm_2D(fx_lp[index], fy_lp[index]) > MU_ST_GCM * Fn_lp[index])
			{
				state[index] = SLIDING_GCM;
			}
		}
		else // friction: sliding
		{
			fr = MU_SL_GCM * Fn;

			// speed norm
			vr = norm_2D(v[0], v[1]);

			// safety
			if (!vr) { return; }

			denom = 1.0 / vr;

			update_low_pass(LOW_FILT_FX, fx_lp[index], -fr * v[0] * denom);
			update_low_pass(LOW_FILT_FY, fy_lp[index], -fr * v[1] * denom);

			// sliding to stiction
			if (norm_2D(vx_lp[index], vy_lp[index]) < V_THRES_GCM)
			{
				// numerical issue safety
				if ( norm_2D(r[0]-rs[index][0], r[1]-rs[index][1]) > DIST_STICTION_SAFETY)
				{
					// store the initial stiction point of contact
					for(int j=0; j<3; j++)
					{
						rs[index][j] = r[j];
					}
				}

				state[index] = STICTION_GCM;
			}
		}
	}
	else // no contact with the ground
	{
		update_low_pass(LOW_FILT_FN, Fn_lp[index], 0.0);
		update_low_pass(LOW_FILT_FX, fx_lp[index], 0.0);
		update_low_pass(LOW_FILT_FY, fy_lp[index], 0.0);

		state[index] = SWING_GCM;
	}
}

/*! \brief compute specific values for the ground model
 *
 * nothing to do for this ground
 */
void FlatGround::compute()
{

}
