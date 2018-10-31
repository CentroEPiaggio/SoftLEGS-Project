
#include "LinearGround.hh"

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info information coming from the sensors
 */
LinearGround::LinearGround(MBSdataStruct *MBSdata): EquationGround(MBSdata)
{

}

/*! \brief destructor
 */
LinearGround::~LinearGround()
{

}

/*! \brief compute specific values for the ground model
 *
 * nothing to do for this ground
 */
void LinearGround::compute()
{
	
}
