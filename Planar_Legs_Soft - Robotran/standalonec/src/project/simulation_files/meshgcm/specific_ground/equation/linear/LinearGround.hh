/*! 
 * \author Nicolas Van der Noot
 * \file LinearGround.hh
 * \brief LinearGround class
 */

#ifndef _LINEAR_GROUND_HH_
#define _LINEAR_GROUND_HH_

#include "EquationGround.hh"

#define FLAT_DIST 5.0
#define SLOPE_ALPHA 0.0524 // 3 degrees -> -tg(3*pi/180)

/*! \brief specific ground model: flat ground, linearly increasing
 */
class LinearGround: public EquationGround
{
	public:
		LinearGround(MBSdataStruct *MBSdata);
		virtual ~LinearGround();

		virtual void compute();

	private:
		/*! \brief description of the ground for ground equations models
		 * 
		 * \param[in] x x position [m]
		 * \param[in] y y position [m]
		 * \param[out] h height of the ground at point (x,y) [m]
		 * \param[out] hx derivative along x of the height of the ground at point (x,y) [-]
		 * \param[out] hy derivative along y of the height of the ground at point (x,y) [-]
		 */
		virtual inline void ground_description(double x, double y, double *h, double *hx, double *hy)
		{
			double delta_x;

			delta_x = x - FLAT_DIST;

			if (delta_x < 0.0)
			{
				*h = 0.0;
				*hx = 0.0;
			}
			else
			{
				*h = SLOPE_ALPHA * delta_x;
				*hx = SLOPE_ALPHA;
			}
			
			*hy = 0.0;
		}
};

#endif
