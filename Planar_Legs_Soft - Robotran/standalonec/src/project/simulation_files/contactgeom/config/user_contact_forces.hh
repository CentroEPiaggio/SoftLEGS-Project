/*! 
 * \author Nicolas Van der Noot
 * \file user_contact_forces.hh
 * \brief user contact force parametrization
 */

#ifndef _USER_CONTACT_FORCES_HH_
#define _USER_CONTACT_FORCES_HH_

#include "BasicShape.hh"

namespace ContactGeom{

// function prototypes
double normal_force(BasicShape *shape, BasicShape *other_shape, double volume, double volume_dot);
double tangentiel_force(BasicShape *shape, BasicShape *other_shape, double v, double F_N);

}
#endif
