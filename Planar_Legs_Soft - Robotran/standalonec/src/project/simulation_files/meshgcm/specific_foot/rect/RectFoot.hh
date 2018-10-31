/*! 
 * \author Nicolas Van der Noot
 * \file RectFoot.hh
 * \brief RectFoot class
 */

#ifndef _RECT_FOOT_HH_
#define _RECT_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: short foot
 */
class RectFoot: public ContactFoot
{
	public:
		RectFoot(MBSdataStruct *MBSdata, int S_sens_id, int left_flag);
		virtual ~RectFoot();
};

#endif
