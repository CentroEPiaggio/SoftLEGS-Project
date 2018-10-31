#include "user_contact_states.hh"

namespace ContactGeom{

/*! \brief user possible contact states update
 * 
 * \param[in,out] shape current shape to (possibly) update
 * \param[in,out] MBSdata Robotran structure
 *
 * This function is called once during each time step, for each basic shape.
 * It can be used for instance to update a FSM (Finite State Machine),
 * switching from stiction to sliding.
 *
 * You can use 'shape->get_module()' to access this shape own module (defined in 'UserShapeModule.hh').
 */
void user_contact_states(BasicShape *shape, MBSdataStruct *MBSdata)
{

}

}
