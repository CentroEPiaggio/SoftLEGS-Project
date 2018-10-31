
#include "ContactFoot.hh"

extern "C" {
	#include "MBSfun.h"
}

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] nb_points number of mesh points [-]
 */
ContactFoot::ContactFoot(MBSdataStruct *MBSdata, int nb_points, int S_sens_id)
{
	double tsim;

	this->nb_points = nb_points;

	this->MBSdata = MBSdata;

	this->S_sens_id = S_sens_id;

	nb_joints = MBSdata->njoint;

	for (int i=0; i<3; i++)
	{
		P[i] = 0.0;
		V[i] = 0.0;
		OM[i] = 0.0;

		F_tot[i] = 0.0;
		T_tot[i] = 0.0;

		for (int j=0; j<3; j++)
		{
			R[i][j] = 0.0;
		}
	}

	// foot mesh
	rn = (double**) malloc(nb_points*sizeof(double*));
	rs = (double**) malloc(nb_points*sizeof(double*));

	state = (int*) malloc(nb_points*sizeof(int));
	
	Fn_lp = (double*) malloc(nb_points*sizeof(double));
	fx_lp = (double*) malloc(nb_points*sizeof(double));
	fy_lp = (double*) malloc(nb_points*sizeof(double));
	vx_lp = (double*) malloc(nb_points*sizeof(double));
	vy_lp = (double*) malloc(nb_points*sizeof(double));

	for (int i=0; i<nb_points; i++)
	{
		rn[i] = (double*) malloc(3*sizeof(double));
		rs[i] = (double*) malloc(3*sizeof(double));

		for (int j=0; j<3; j++)
		{
			rn[i][j] = 0.0;
			rs[i][j] = 0.0;
		}

		state[i] = SLIDING_GCM;

		Fn_lp[i] = 0.0;
		fx_lp[i] = 0.0;
		fy_lp[i] = 0.0;
		vx_lp[i] = 0.0;
		vy_lp[i] = 0.0;
	}

	// S sensor
	allocate_sensor(&S_Sens, nb_joints);
	init_sensor(&S_Sens, nb_joints);

	// low pass filters
	tsim = MBSdata->tsim;

	low_filters[LOW_FILT_FN] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_FX] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_FY] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_VX] = new LowFilterSimu(1.0e-2, tsim);
	low_filters[LOW_FILT_VY] = new LowFilterSimu(1.0e-2, tsim);
}

/*! \brief destructor
 */
ContactFoot::~ContactFoot()
{
	for(int i=0; i<NB_LOW_FILT_MESH; i++)
	{
		delete low_filters[i];
	}

	for (int i=0; i<nb_points; i++)
	{
		free(rn[i]);
		free(rs[i]);
	}

	free(rn);
	free(rs);

	free(state);

	free(Fn_lp);
	free(fx_lp);
	free(fy_lp);
	free(vx_lp);
	free(vy_lp);

	// S sensor
	free_sensor(&S_Sens);
}

/*! \brief update the pose (position, rotation and derivatives) with the user_ExtForces inputs
 * 
 * \param[in] PxF absolute position (provided by Robotran)
 * \param[in] RxF absolute rotation matrix (provided by Robotran)
 * \param[in] VxF position derivative (provided by Robotran)
 * \param[in] OMxF orientation derivative (provided by Robotran)
 */
void ContactFoot::update_pose(double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4])
{
	for (int i=0; i<3; i++)
	{
		P[i]  = PxF[i+1];
		V[i]  = VxF[i+1];
		OM[i] = OMxF[i+1];

		for (int j=0; j<3; j++)
		{
			R[i][j] = RxF[i+1][j+1];
		}
	}
}

/*! \brief update the pose (position, rotation and derivatives) with the model F sensors
 */
void ContactFoot::update_pose_Fsens()
{
	init_sensor(&S_Sens, nb_joints);

	sensor(&S_Sens, MBSdata, S_sens_id);

	for (int i=0; i<3; i++)
	{
		P[i]  = S_Sens.P[i+1];
		V[i]  = S_Sens.V[i+1];
		OM[i] = S_Sens.OM[i+1];

		for (int j=0; j<3; j++)
		{
			R[i][j] = S_Sens.R[i+1][j+1];
		}
	}
}

/*! \brief update the low-pass filter parameters
 * 
 * \param[in] new_t new time [s]
 */
void ContactFoot::update_low_pass_params(double new_t)
{
	for(int i=0; i<NB_LOW_FILT_MESH; i++)
	{
		low_filters[i]->update_params(MBSdata->tsim);
	}
}
