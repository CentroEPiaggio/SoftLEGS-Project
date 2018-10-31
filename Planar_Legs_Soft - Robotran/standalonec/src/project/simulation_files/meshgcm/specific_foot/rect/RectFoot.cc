
#include "RectFoot.hh"

#include <cstdlib>

/*
 * Foot dimensions:
 * x: -0.16  to 0.16  = 0.32
 * y: -0.09 to 0.07 = 0.16 (right)
 * y: -0.07 to 0.09 = 0.16 (left)
 */
#define NB_X_POINTS 5
#define NB_Y_POINTS 4
#define NB_POINTS (NB_X_POINTS * NB_Y_POINTS)

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] S_sens_id S sensor ID
 * \param[in] left_flag 1 for left foot, 0 otherwise
 */
RectFoot::RectFoot(MBSdataStruct *MBSdata, int S_sens_id, int left_flag): ContactFoot(MBSdata, NB_POINTS, S_sens_id)
{
	int index;
	double x_min, x_max, y_min, y_max, x_res, y_res;

	x_min = -0.16;
	x_max =  0.16;

	if (left_flag) // left foot
	{
		y_min = -0.07;
		y_max =  0.09;
	}
	else // right foot
	{
		y_min = -0.09;
		y_max =  0.07;
	}

	x_res = (x_max - x_min) / (NB_X_POINTS - 1);
	y_res = (y_max - y_min) / (NB_Y_POINTS - 1);

	for (int i=0; i<NB_X_POINTS; i++)
	{
		for (int j=0; j<NB_Y_POINTS; j++)
		{
			index = j + i*NB_Y_POINTS;

			rn[index][0] = x_min + i*x_res; ///< x indexes
			rn[index][1] = y_min + j*y_res; ///< y indexes
			rn[index][2] = 0.0;             ///< z indexes
		}	
	}
}

/*! \brief destructor
 */
RectFoot::~RectFoot()
{
	// free already done in mother class
}
