#include "GroundModel.hh"

// limiting external forces
#define MAX_EXT_FORCES	5000.0 ///< max force [N]
#define MAX_EXT_MOMENTS	5000.0 ///< max moment [Nm]

/// return value limited in a given range
inline double range_limit(double x, double x_min, double x_max) { return (x < x_min) ? x_min : ((x > x_max) ? x_max : x); }

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info information coming from the sensors
 */
GroundModel::GroundModel(MBSdataStruct *MBSdata)
{
	this->MBSdata = MBSdata;

	t = MBSdata->tsim;

	nb_points = 0;

	rn = NULL;
	rs = NULL;

	state = NULL;

	Fn_lp = NULL;
	fx_lp = NULL;
	fy_lp = NULL;
	vx_lp = NULL;
	vy_lp = NULL;

	low_filters = NULL;

	for(int i=0; i<3; i++)
	{
		P[i]  = 0.0;
		V[i]  = 0.0;
		OM[i] = 0.0;

		for(int j=0; j<3; j++)
		{
			R[i][j] = 0.0;
		}
	}
}

/*! \brief destructor
 */
GroundModel::~GroundModel()
{

}

/*! \brief copy the current foot pose information
 * 
 * \param[in] foot current foot body
 */
inline void GroundModel::update_current_foot(ContactFoot *foot)
{
	nb_points = foot->get_nb_points();

	rn = foot->get_rn();
	rs = foot->get_rs();

	state = foot->get_state();

	Fn_lp = foot->get_Fn_lp();
	fx_lp = foot->get_fx_lp();
	fy_lp = foot->get_fy_lp();
	vx_lp = foot->get_vx_lp();
	vy_lp = foot->get_vy_lp();

	low_filters = foot->get_low_filters();

	for(int i=0; i<3; i++)
	{
		P[i]  = foot->get_P(i);
		V[i]  = foot->get_V(i);
		OM[i] = foot->get_OM(i);

		for(int j=0; j<3; j++)
		{
			R[i][j] = foot->get_R(i, j);
		}
	}
}

/*! \brief compute the force and torque applied on the contact body by the ground
 * 
 * \param[out] F_tot external force applied on the body [N]
 * \param[out] F_tot external torque applied on the body [Nm]
 * \param[in] foot current foot body
 */
void GroundModel::compute_F_T(double F_tot[3], double T_tot[3], ContactFoot *foot)
{
	t = MBSdata->tsim;

	update_current_foot(foot);

	for(int i=0; i<3; i++)
	{
		F_tot[i] = 0.0;
		T_tot[i] = 0.0;
	}

	loop_mesh_F_T(F_tot, T_tot);

	for(int i=0; i<3; i++)
	{
		F_tot[i] = range_limit(F_tot[i], -MAX_EXT_FORCES , MAX_EXT_FORCES );
		T_tot[i] = range_limit(T_tot[i], -MAX_EXT_MOMENTS, MAX_EXT_MOMENTS);

		foot->set_F_tot(i, F_tot[i]);
		foot->set_T_tot(i, T_tot[i]);
	}
}

/*! \brief check and manage the mesh contact point states
 * 
 * \param[in,out] foot current foot body
 */
void GroundModel::state_switch(ContactFoot *foot)
{
	t = MBSdata->tsim;

	foot->update_pose_Fsens();

	foot->update_low_pass_params(t);

	update_current_foot(foot);

	loop_mesh_state();
}
