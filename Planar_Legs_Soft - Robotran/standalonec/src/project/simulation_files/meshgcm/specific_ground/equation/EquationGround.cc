
#include "EquationGround.hh"

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

/*! \brief compute the direct rotation of a vector
 * 
 * \param[in] R rotation matrix
 * \param[in] x input vector
 * \param[out] y output vector
 */
inline void direct_rot(double R[3][3], double x[3], double y[3])
{
	y[0] = R[0][0] * x[0] + R[0][1] * x[1] + R[0][2] * x[2];
	y[1] = R[1][0] * x[0] + R[1][1] * x[1] + R[1][2] * x[2];
	y[2] = R[2][0] * x[0] + R[2][1] * x[1] + R[2][2] * x[2];
}

/*! \brief compute the inverse rotation of a vector
 * 
 * \param[in] R rotation matrix
 * \param[in] x input vector
 * \param[out] y output vector
 */
inline void inverse_rot(double R[3][3], double x[3], double y[3])
{
	y[0] = R[0][0] * x[0] + R[1][0] * x[1] + R[2][0] * x[2];
	y[1] = R[0][1] * x[0] + R[1][1] * x[1] + R[2][1] * x[2];
	y[2] = R[0][2] * x[0] + R[1][2] * x[1] + R[2][2] * x[2];
}

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info information coming from the sensors
 */
EquationGround::EquationGround(MBSdataStruct *MBSdata): GroundModel(MBSdata)
{

}

/*! \brief destructor
 */
EquationGround::~EquationGround()
{

}

/*! \brief compute the rotation matrix associated with the linearization of a ground
 * 
 * \param[in] gcm_hx ground description derivative (along x)
 * \param[in] gcm_hy ground description derivative (along y)
 * \param[out] R_g output rotation matrix
 *
 * This function is placed in the 'EquationGround' class header, so that it can be inlined
 */
inline void EquationGround::ground_rot_matrix(double gcm_hx, double gcm_hy, double R_g[3][3])
{
	double a, b, c;
	double x_a, x_b, x_c, y_a, y_b, y_c;
	double x_norm_inv, y_norm_inv, z_norm_inv;

	double aa, bb, ab;

	a = gcm_hx;
	b = gcm_hy;
	c = 1.0;

	aa = a*a;
	bb = b*b;
	ab = a*b;

	x_a = 1.0;
	x_b = 0.0;
	x_c = -a; /* -a/c (with c = 1.0) */

	y_a = -ab; /* -a*b/c (with c = 1.0) */
	y_b = 1.0 + aa; /* c + a*a/c (with c = 1.0) */
	y_c = -b;

	x_norm_inv = 1.0 / sqrt(1.0 + aa);
	y_norm_inv = 1.0 / sqrt(y_a*y_a + y_b*y_b + bb);
	z_norm_inv = 1.0 / sqrt(aa + bb + 1.0);

	R_g[0][0] = x_a * x_norm_inv;
	R_g[1][0] = x_b * x_norm_inv;
	R_g[2][0] = x_c * x_norm_inv;

	R_g[0][1] = y_a * y_norm_inv;
	R_g[1][1] = y_b * y_norm_inv;
	R_g[2][1] = y_c * y_norm_inv;

	R_g[0][2] = a * z_norm_inv;
	R_g[1][2] = b * z_norm_inv;
	R_g[2][2] = c * z_norm_inv;
}	

/*! \brief compute the force on one point of the mesh
 * 
 * \param[out] F force applied on this point [N]
 * \param[in] r point absolute position [m]
 * \param[in] r point absolute velocity [m/s]
 * \param[in] index index of the point in the mesh
 * \return 1 if contact, 0 otherwise
 */
inline int EquationGround::contact_ground(double F[3], double r[3], double v[3], int index)
{
	// variables declaration
	double R_g[3][3], v_g[3], F_g[3], diff_r[3], diff_r_g[3];
	double denom, dz, dz_g, fr_g, vr_g;
	double gcm_h, gcm_hx, gcm_hy;

	// 3D ground description
	ground_description(r[0], r[1], &gcm_h, &gcm_hx, &gcm_hy);

	// vertical penetration
	dz = r[2] - gcm_h;

	if (dz <= 0) // contact with the ground
	{
		// get ground local frame as a rotation matrix
		ground_rot_matrix(gcm_hx, gcm_hy, R_g);


		// -- normal force -- //

		// vertical penetration in the ground frame
		dz_g = dz * R_g[2][2];

		// velocity in the ground frame
		direct_rot(R_g, v, v_g);

		// normal force
		F_g[2] = -KP_Z_GCM * dz_g * pos(1.0 - KD_Z_GCM * v_g[2]);


		// -- tangential force -- //

		if (state[index] == STICTION_GCM) // friction: stiction force
		{
			// shift motion
			for (int j=0; j<3; j++)
			{
				diff_r[j] = r[j] - rs[index][j];
			}

			// shift motion in the ground frame
			direct_rot(R_g, diff_r, diff_r_g);

			F_g[0] = - KP_XY_GCM * diff_r_g[0] * pos( 1.0 + KD_XY_GCM * sgn(diff_r_g[0]) * v_g[0] );
			F_g[1] = - KP_XY_GCM * diff_r_g[1] * pos( 1.0 + KD_XY_GCM * sgn(diff_r_g[1]) * v_g[1] );
		}
		else // friction: sliding force
		{
			// speed norm
			vr_g = norm_2D(v_g[0], v_g[1]);

			// safety
			if (!vr_g) { return 1; }

			denom = 1.0 / vr_g;

			fr_g = MU_SL_GCM * F_g[2];

			F_g[0] = -fr_g * v_g[0] * denom;
			F_g[1] = -fr_g * v_g[1] * denom;
		}

		// inertial frame forces
		inverse_rot(R_g, F_g, F);

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
void EquationGround::state_ground(double r[3], double v[3], int index)
{
	double R_g[3][3], v_g[3], diff_r[3], diff_r_g[3];
	double dz, fr_g, Fn, vr_g, denom;
	double gcm_h, gcm_hx, gcm_hy;

	// 3D ground description
	ground_description(r[0], r[1], &gcm_h, &gcm_hx, &gcm_hy);

	// get ground local frame as a rotation matrix
	ground_rot_matrix(gcm_hx, gcm_hy, R_g);

	// velocity in the ground frame
	direct_rot(R_g, v, v_g);

	update_low_pass(LOW_FILT_VX, vx_lp[index], v_g[0]);
	update_low_pass(LOW_FILT_VY, vy_lp[index], v_g[1]);

	// vertical penetration
	dz = r[2] - gcm_h;

	if (dz <= 0) // contact with the ground
	{
		// -- normal force -- //

		Fn = -KP_Z_GCM * dz * R_g[2][2] * pos(1.0 - KD_Z_GCM * v_g[2]);

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
			for (int j=0; j<3; j++)
			{
				diff_r[j] = r[j] - rs[index][j];
			}

			// shift motion in the ground frame
			direct_rot(R_g, diff_r, diff_r_g);

			update_low_pass(LOW_FILT_FX, fx_lp[index], diff_r_g[0] * pos(1.0 + KD_XY_GCM * sgn(diff_r_g[0]) * v_g[0]) );
			update_low_pass(LOW_FILT_FY, fy_lp[index], diff_r_g[1] * pos(1.0 + KD_XY_GCM * sgn(diff_r_g[1]) * v_g[1]) );

			// stiction to sliding
			if (norm_2D(fx_lp[index], fy_lp[index]) > MU_ST_GCM * Fn_lp[index])
			{
				state[index] = SLIDING_GCM;
			}
		}
		else // friction: sliding
		{
			fr_g = MU_SL_GCM * Fn;

			// speed norm
			vr_g = norm_2D(v_g[0], v_g[1]);

			// safety
			if (!vr_g) { return; }

			denom = 1.0 / vr_g;

			update_low_pass(LOW_FILT_FX, fx_lp[index], -fr_g * v_g[0] * denom);
			update_low_pass(LOW_FILT_FY, fy_lp[index], -fr_g * v_g[1] * denom);

			// sliding to stiction
			if (norm_2D(vx_lp[index], vy_lp[index]) < V_THRES_GCM)
			{
				// shift motion
				for (int j=0; j<3; j++)
				{
					diff_r[j] = r[j] - rs[index][j];
				}

				// shift motion in the ground frame
				direct_rot(R_g, diff_r, diff_r_g);

				// numerical issue safety
				if ( norm_2D(diff_r_g[0], diff_r_g[1]) > DIST_STICTION_SAFETY)
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
