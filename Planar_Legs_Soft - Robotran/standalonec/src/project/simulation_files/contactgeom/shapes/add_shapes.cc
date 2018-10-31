#include "add_shapes.hh"

namespace ContactGeom{

/*! \brief add a fixed rigid body with zero as reference position
 *
 * \param[in,out] union_shape union shape receiving the new geometry shape
 * \return created rigid shape
 */
RigidShape* add_rigid_fixed_zero(UnionShape *union_shape)
{
    RigidShape *new_rigid_shape;

    new_rigid_shape = new RigidShape(union_shape->get_mbs_data(), union_shape);

    union_shape->add_geometry(new_rigid_shape);

    return new_rigid_shape;
}

/*! \brief add a fixed rigid body
 * 
 * \param[in,out] union_shape union shape receiving the new geometry shape
 * \param[in] x reference position x component
 * \param[in] y reference position y component
 * \param[in] z reference position z component
 * \return created rigid shape
 */
RigidShape* add_rigid_fixed(UnionShape *union_shape, double x, double y, double z)
{
    RigidShape *new_rigid_shape;

    new_rigid_shape = new RigidShape(union_shape->get_mbs_data(), get_vector_3D(x, y, z), union_shape);

    union_shape->add_geometry(new_rigid_shape);

    return new_rigid_shape;
}

/*! \brief add a rigid body with a S sensor (apply forces, do not receive forces)
 * 
 * \param[in,out] main_union main union shape with all shapes
 * \param[in] id ID of the Ssensor
 * \return created rigid shape
 */
RigidShape* add_rigid_Ssens(MainUnionShape *main_union, int id)
{
    RigidShape *new_rigid_shape;

    new_rigid_shape = new RigidShape(main_union->get_mbs_data(), RIGID_S_SENS, id, main_union);

    main_union->add_geometry(new_rigid_shape);

    main_union->add_rigid_S(new_rigid_shape);

    return new_rigid_shape;
}

/*! \brief add a rigid body with a S sensor (apply forces, do not receive forces)
 * 
 * \param[in,out] main_union main union shape with all shapes
 * \param[in,out] union_shape union shape receiving the new geometry shape
 * \param[in] id ID of the Ssensor
 * \return created rigid shape
 */
RigidShape* add_rigid_Ssens(MainUnionShape *main_union, UnionShape *union_shape, int id)
{
    RigidShape *new_rigid_shape;

    new_rigid_shape = new RigidShape(union_shape->get_mbs_data(), RIGID_S_SENS, id, union_shape);

    union_shape->add_geometry(new_rigid_shape);

    main_union->add_rigid_S(new_rigid_shape);

    return new_rigid_shape;
}

/*! \brief add a rigid body with a F sensor (apply forces, receive forces)
 * 
 * \param[in,out] main_union main union shape with all shapes
 * \param[in] id ID of the Fsensor
 * \return created rigid shape
 */
RigidShape* add_rigid_Fsens(MainUnionShape *main_union, int id)
{
    RigidShape *new_rigid_shape;

    new_rigid_shape = new RigidShape(main_union->get_mbs_data(), RIGID_F_SENS, id, main_union);

    main_union->add_geometry(new_rigid_shape);

    main_union->add_rigid_F(new_rigid_shape);

    return new_rigid_shape;
}

/*! \brief add a rigid body with a F sensor (apply forces, receive forces)
 * 
 * \param[in,out] main_union main union shape with all shapes
 * \param[in,out] union_shape union shape receiving the new geometry shape
 * \param[in] id ID of the Fsensor
 * \return created rigid shape
 */
RigidShape* add_rigid_Fsens(MainUnionShape *main_union, UnionShape *union_shape, int id)
{
    RigidShape *new_rigid_shape;

    new_rigid_shape = new RigidShape(union_shape->get_mbs_data(), RIGID_F_SENS, id, union_shape);

    union_shape->add_geometry(new_rigid_shape);

    main_union->add_rigid_F(new_rigid_shape);

    return new_rigid_shape;
}

}
