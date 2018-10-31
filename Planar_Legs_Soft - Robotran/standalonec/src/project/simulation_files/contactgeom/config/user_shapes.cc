
#ifndef STANDALONE
#include <cmath>
#ifdef _CHAR16T // math.h version
#define CHAR16_T // visual version
#endif
#endif

#include "user_shapes.hh"
#include "add_shapes.hh"
#include "RigidShape.hh"

#include "simu_def.h"

namespace ContactGeom{

/// get - for 0, + for 1
inline double plus_minus(int i) { return !i ? -1.0 : 1.0; }

#define DEG_TO_RAD (M_PI / 180.0)

/*! \brief configure the user shapes added in the contact model
 * 
 * \param[in,out] main_union main union of shapes
 * \param[in,out] MBSdata Robotran structure
 *
 * There are three different types of rigid shapes (class RigidShape)
 * - fixed: they don't move during the simulation, they apply forces on other bodies,
 *          but they are not impacted by these forces.
 * - related to a S sensor: they move with the body where the S sensor is attached.
 *          Similary to the fixed rigid shapes, they apply forces on other bodies,
 *          but they are not impacted by these forces.
 * - related to a F sensor: they move with the body where the F sensor is attached.
 *          They apply forces on other bodies and are impacted by the opposite of
 *          these forces (action-reaction).
 *
 * For the shapes related to a S sensor or a F sensor, you must activate the following
 * fields in MBSysPad for these sensors:
 * - Position
 * - Rotation Matrix
 * - Velocity
 * The last two fields ('Acceleration' and 'Jacobian Matrix') are useless for this contact model.
 * So, uncheck them to improve the computational speed.
 *
 * Each RigidShape belongs to a union shape (UnionShape): a union of one or more shapes.
 * A union shape can contain other union shapes (daughters) or rigid shapes. The ancestor
 * of all union shapes is a special union shape: main_union (provided as argument).
 *
 * Except in 'main_union', bodies inside a union shape can only interact with bodies
 * inside other unions. On top of that, if applicable (i.e. no plane inside the union),
 * wrapping objects are computed around the unions to quickly check if the contact is
 * possible. Consequently, add unions when you do not want the objects inside the same
 * union to interact and to speed up the computation by indicating unions within which
 * objects will probably stay close during the simulation. Consequently, it can be useful
 * to make unions of unions to speed up the computation. For instance, if we only consider
 * the interactions between a humanoid and its environment, we can place all the bodies
 * of this humanoid inside a single union (no interactions computed between the humanoid
 * different parts). Then, we can define daughter unions for each arm, leg... because we know
 * that the different bodies will stay close to the other bodies in the same daughter union.
 *
 * To add a union, use this line:
 *      cur_union = previous_union->add_union();
 * where 'cur_union' and 'previous_union' are two UnionShape shapes.
 *
 * When you want to add a rigid body to the union 'cur_union' (it can also be 'main_union'
 * if you do not have any union daughter for this rigid shape), use the line:
 *      cur_rigid = xxx;
 * where xxx can be:
 * - add_rigid_fixed_zero(cur_union) : basic rigid shape not moving, with the frame origin
 *        being the point (0,0,0) (see later)
 * - add_rigid_fixed(cur_union, x, y, z) : rigid shape not moving, with the frame origin
 *        being the point (x,y,z)
 * - add_rigid_Ssens(main_union, cur_union, isens) : rigid shape related to the S sensor (reference
 *        point) with the index 'isens' (starting to 1, coherent with 'sensor')
 * - add_rigid_Ssens(main_union, isens) : like 'add_rigid_Ssens(main_union, cur_union, isens)',
 *        but when the current union 'cur_union' is 'main_union'
 * - add_rigid_Fsens(main_union, cur_union, isens) : rigid shape related to the F sensor (reference
 *        point) with the index 'isens' (starting to 1, add the number of S sensors to get its ID
 *        in 'sensor')
 * - add_rigid_Fsens(main_union, isens) : like 'add_rigid_Fsens(main_union, cur_union, isens)',
 *        but when the current union 'cur_union' is 'main_union'
 * All these fucntions return a pointer to the initialized BasicShape.
 *
 * Each rigid shape can be made of one or different basic shapes (these shapes can be different
 * inside the same rigid shape). Currently, there are three basic shapes: infinite plane, sphere
 * and cuboid. The structure of the code is designed to easily allow people to extend this set
 * by implementing new basic shapes if needed.
 *
 * These are the lines you can use to add a basic shape to the rigid shape 'cur_rigid'
 * (all arguments are in [m] or [rad]):
 *
 * - cur_rigid->add_plane(z) : add a plane with the normal in the +z direction, z being its height.
 *        This function is useful for a flat ground.
 * - cur_rigid->add_plane(a,b,c,d) : add a plane with the equation 'ax + by + cz +d = 0'
 * - cur_rigid->add_plane(normal_x, normal_y, normal_z, pos_x, pos_y, pos_z) : add a plane whose
 *        normal is (normal_x, normal_y, normal_z) and its position relative to the reference point
 *        is (pos_x, pos_y, pos_z). This is expressed in the frame local to the rigid shape.
 *
 * - cur_rigid->add_sphere(radius, pos_x, pos_y, pos_z) : sphere with a radius 'radius' and a position
 *        relative to the reference point (pos_x, pos_y, pos_z).
 *
 * - cur_rigid->add_cuboid(d, w, h, pos_x, pos_y, pos_z, theta_x, theta_y, theta_z) : cuboid with dimensions
 *        d (depth in 'x'), w (width in 'y') and h (height in 'z'), relative position (pos_x, pos_y, pos_z)
 *        and relative orientation (theta_x, theta_y, theta_z), all expressed in the frame local to the
 *        rigid shape.
 *
 * Shapes inside the same union (except 'main_union') will never interact with each other. By default, all
 * the other shapes can have interactions. However, you can change this default behaviour with the function
 * 'set_no_contact_default' (its opposite being 'set_contact_default'), as you can see in this example:
 *        cur_shape->set_no_contact_default();
 * This can be applied to any shape (union, rigid, basic).This function changes the behaviour of the selected
 * shape and of all its descendents (even the ones created later).
 *
 * Then, you can add exceptions with the function 'add_prohibited(GeometryShape *other_shape)' when the default
 * behaviour is to have contact with all other shapes (except in the same union) and with the function
 * 'add_no_prohibited(GeometryShape *other_shape)' when the default behaviour is to have no interaction.
 * For instance, you can use this fucntion:
 *        cur_shape->add_prohibited(other_shape);
 * where 'cur_shape' and 'other_shape' are two shapes (union, rigid or basic) to have no contact between
 * these two shapes and all their descendents (even the ones created later).
 */
void config_shapes(MainUnionShape *main_union, MBSdataStruct *MBSdata)
{
	#ifdef CUBOID_GCM
	RigidShape *cur_rigid;
	double z_height;

	z_height = 0.02;

	// ground
	cur_rigid = add_rigid_fixed_zero(main_union);
	cur_rigid->add_plane(0.0);

	// right foot
	cur_rigid = add_rigid_Fsens(main_union, RFOOT_FSENS_ID);
	cur_rigid->add_cuboid(0.32, 0.16, z_height, 0.0, -0.01, z_height/2.0, 0.0, 0.0, 0.0);

	// left foot 
	cur_rigid = add_rigid_Fsens(main_union, LFOOT_FSENS_ID);
	cur_rigid->add_cuboid(0.32, 0.16, z_height, 0.0, 0.01, z_height/2.0, 0.0, 0.0, 0.0);
	#endif
}

}
