/*! 
 * \author Nicolas Van der Noot
 * \file add_shape.hh
 * \brief add shapes for the user
 */

#ifndef _ADD_SHAPES_HH_
#define _ADD_SHAPES_HH_

#include "RigidShape.hh"
#include "MainUnionShape.hh"

namespace ContactGeom{

RigidShape* add_rigid_fixed_zero(UnionShape *union_shape);
RigidShape* add_rigid_fixed(UnionShape *union_shape, double x, double y, double z);
RigidShape* add_rigid_Ssens(MainUnionShape *main_union, int id);
RigidShape* add_rigid_Ssens(MainUnionShape *main_union, UnionShape *union_shape, int id);
RigidShape* add_rigid_Fsens(MainUnionShape *main_union, int id);
RigidShape* add_rigid_Fsens(MainUnionShape *main_union, UnionShape *union_shape, int id);

}
#endif
