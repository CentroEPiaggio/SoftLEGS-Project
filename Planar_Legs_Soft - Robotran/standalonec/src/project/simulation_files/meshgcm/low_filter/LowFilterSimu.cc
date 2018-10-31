#include "LowFilterSimu.hh"

/*! \brief constructor
 * 
 * \param[in] tau time constant [s]
 * \param[in] cur_t current time [s]
 */
LowFilterSimu::LowFilterSimu(double tau, double cur_t)
{
	this->tau = tau;

	last_t = cur_t;
	delta_t = 0.0;

	f1 = 0.0;
	f2 = 1.0;
}

/*! \brief destructor
 */
LowFilterSimu::~LowFilterSimu()
{

}

/*! \brief update the low-pass filter parameters
 * 
 * \param[in] new_t new time [s]
 */
void LowFilterSimu::update_params(double new_t)
{
	double f, frac, new_delta_t;

	if ((new_t > last_t) && (tau > 0.0))
	{
		new_delta_t = new_t - last_t;
		last_t = new_t;

		if (delta_t != new_delta_t)
		{
			delta_t = new_delta_t;

			f = delta_t / tau;
			frac = 1.0 / (1.0 + f);

			f1 = f * frac;
			f2 = frac;
		}			
	}
	else
	{
		delta_t = 0.0;

		f1 = 0.0;
		f2 = 1.0;
	}
}
