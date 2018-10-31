
#include "WholeFeet.hh"

/*! \brief contructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] ground ground model
 */
WholeFeet::WholeFeet(MBSdataStruct *MBSdata, GroundModel *ground)
{
	this->MBSdata = MBSdata;
	this->ground  = ground;
}

/*! \brief destructor
 */
WholeFeet::~WholeFeet()
{
	for (unsigned int i=0; i<bodies.size(); i++)
	{
		delete bodies[i];
	}
}

/*! \brief computation called only once each valid time-step
 */
void WholeFeet::state_compute()
{
	for(unsigned int i=0; i<bodies.size(); i++)
	{
		ground->state_switch(bodies[i]);
	}
}
