/*! 
 * \author Nicolas Van der Noot
 * \file UserShapeModule.cc
 * \brief user own class for the shapes
 * 
 * This class can be used by the user to add variables and functions (also in 'UserShapeModule.hh')
 * to each shape (union, rigid or basic). This might for instance be used to assign a different
 * friction coefficients to the different basic shapes.
 *
 * To this end, you can for instance use this line in 'user_shapes.cc':
 *       cur_basic = cur_rigid->add_sphere(0.2, 0.0, 0.0, 0.0);
 * where 'cur_basic' is the current BasicShape created and 'cur_rigid' is the current RigidShape.
 *
 * Then, the function 'UserShapeModule& get_module()' can be used to recover the corresponding
 * UserShapeModule. You can for instance use these lines:
 *       cur_user_module = cur_basic->get_module();
 * and then call your own fucntions to set the requested friction coefficient for instance.
 *
 * Then, to use it, you can for instance go inside 'user_contact_force.cc' or 'user_contact_states.cc'
 * and call this line:
 *       cur_user_module = shape->get_module();
 * Then, you can retrieve the shape properties defined in 'user_shapes.cc'.
 */
#include "UserShapeModule.hh"

namespace ContactGeom{

/*! \brief constructor
 */
UserShapeModule::UserShapeModule()
{

}

/*! \brief destructor
 */
UserShapeModule::~UserShapeModule()
{

}

}
