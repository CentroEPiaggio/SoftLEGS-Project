/*! 
 * \author Nicolas Van der Noot
 * \file LowFilterSimu.hh
 * \brief LowFilterSimu class
 */

#ifndef _LOW_FILTER_SIMU_HH_
#define _LOW_FILTER_SIMU_HH_

/*! \brief First order low pass filter (for the simulation, no state value of the output)
 */
class LowFilterSimu
{
	public:
		LowFilterSimu(double tau, double cur_t);
		~LowFilterSimu();

		void update_params(double new_t);

		/*! \brief low-pass filter computation
		 * 
		 * \param[in] old_val old filtered value
		 * \param[in] new_val new value
		 * \return filtered low_pass value
		 */
		inline double low_pass(double old_val, double new_val)
		{
			if (!f1)
			{
				return old_val;
			}
			else
			{
				return f1 * new_val + f2 * old_val;
			}
		}

		/*! \brief low-pass filter computation for one value
		 * 
		 * \param[in,out] value state value to update
		 * \param[in] new_val new value
		 */
		inline void update_low_pass(double &value, double new_val)
		{
			value = low_pass(value, new_val);
		}

	private:
		double tau;     ///< time constant [s]
		double last_t;  ///< last time value added [s]
		double delta_t; ///< time interval [s]
		double f1;      ///< first low-pass filter param
		double f2;      ///< second low-pass filter param
};

#endif
