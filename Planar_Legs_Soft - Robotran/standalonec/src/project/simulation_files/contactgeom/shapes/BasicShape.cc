#include "BasicShape.hh"
#include "RigidShape.hh"
#include "user_contact_forces.hh"
#include "user_contact_states.hh"

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] rel_position relative position vector from the ExtForce of RigidShape to the shape reference position [m]
 * \param[in] parent_shape parent shape
 */
BasicShape::BasicShape(Vector3D const& rel_position, int rigid_flag, GeometryShape *parent_shape): GeometryShape(parent_shape)
{
    type = GEOM_BASIC;

    this->rigid_flag = rigid_flag;

    this->rel_position = rel_position;

    reset_F_T();

    contact.reset();
}

/*! \brief constructor
 * 
 * \param[in] rel_position relative position vector from the ExtForce of RigidShape to the shape reference position [m]
 * \param[in] check_flag 1 if check sphere applicable, 0 otherwise
 * \param[in] parent_shape parent shape
 */
BasicShape::BasicShape(Vector3D const& rel_position, int check_flag, int rigid_flag, GeometryShape *parent_shape): GeometryShape(check_flag, parent_shape)
{
    type = GEOM_BASIC;

    this->rigid_flag = rigid_flag;

    this->rel_position = rel_position;

    reset_F_T();

    contact.reset();
}

/*! \brief constructor
 * 
 * \param[in] rel_position relative position vector from the ExtForce of RigidShape to the shape reference position [m]
 * \param[in] check_flag 1 if check sphere applicable, 0 otherwise
 * \param[in] check_radius radius of the check sphere (if applicable) [m]
 * \param[in] check_center position of the center of the check sphere (if applicable) [m]
 * \param[in] parent_shape parent shape
 */
BasicShape::BasicShape(Vector3D const& rel_position, int check_flag, double check_radius, Vector3D const& check_center, int rigid_flag, GeometryShape *parent_shape): GeometryShape(check_flag, check_radius, check_center, parent_shape)
{
    type = GEOM_BASIC;

    this->rigid_flag = rigid_flag;

    this->rel_position = rel_position;

    reset_F_T();

    contact.reset();
}

/*! \brief destructor
 */
BasicShape::~BasicShape()
{

}

/*! \brief update the kinematics values of the basic body
 * 
 * \param[in] parent_P  absolute position vector of parent RigidShape [m]
 * \param[in] parent_V  absolute velocity vector of parent RigidShape [m/s]
 * \param[in] parent_OM absolute angle velocity vector of parent RigidShape [rad/s]
 * \param[in] parent_R  absolute rotation matrix of parent RigidShape [-]
 */
void BasicShape::update_kinematics(Vector3D const& parent_P, Vector3D const& parent_V, Vector3D const& parent_OM, RotMatrix const& parent_R)
{
    // rotation matrix
    this->parent_R = parent_R;

    // relative position vector from ExtForce in the inertial frame
    rel_pos_frame = product_trans_matrix_vector_3d(parent_R, rel_position);

    // absolute position [m]
    P = parent_P + rel_pos_frame;

    // absolute angular rate [rad/s]
    OM = parent_OM;

    // absolute velocity [m/s]
    V = parent_V + cross_vector_3D(parent_OM, rel_pos_frame);

    // check_flag
    update_check();
}

/*! \brief update the forces and torques based on the contact of the two shapes
 */
void BasicShape::update_F_T(GeometryShape *other_shape)
{
    BasicShape *other_basic;

    int other_type = other_shape->get_type();

    if ((other_type == GEOM_SPHERE) || (other_type == GEOM_CUBOID) || (other_type == GEOM_PLANE))
    {
        other_basic = static_cast<BasicShape*>(other_shape);

        if ( (rigid_flag == RIGID_F_SENS) || (other_basic->get_rigid_flag() == RIGID_F_SENS) )
        {
            // reset contact
            contact_reset(other_basic);
            
            // update forces-torques of contact
            if (update_F_T_basic(other_basic))
            {
                F_T_contact(other_basic);
                other_basic->F_T_contact(this);
            }
        }   
    }
    else if ((other_type == GEOM_UNION) || (other_type == GEOM_RIGID))
    {
        other_shape->update_F_T(this);
    }
    else
    {
        std::cout << "Error: unknown shape !" << std::endl;
        exit(EXIT_FAILURE);
    }
}

/*! \brief update the force and torque at the position reference of the basic shape
 * 
 * \param[in] cur_F current force to add [N]
 * \param[in] cur_F current torque to add [Nm]
 * \param[in] cur_pos current absolute position of the force (application point), in the inertial frame [m]
 */
void BasicShape::update_F_T_local(Vector3D const& cur_F, Vector3D const& cur_T, Vector3D const& cur_pos)
{
    F += cur_F;
    T += cur_T;
    T += cross_vector_3D(cur_pos - P, cur_F);
}

/*! \brief update the total force and torque of the parent RigidShape
 * 
 * \param[out] F_tot total force of the RigidShape at the ExtForce sensor [N]
 * \param[out] T_tot total torque of the RigidShape at the ExtForce sensor [Nm]
 */
void BasicShape::update_F_T_rigid(Vector3D & F_tot, Vector3D & T_tot)
{
    F_tot += F;
    T_tot += T;

    if ((F.get_x() != 0.0) || (F.get_y() != 0.0) || (F.get_z() != 0.0))
    {
        T_tot += cross_vector_3D(rel_pos_frame, F);
    }
}

/*! \brief reset the contact volume of this shape and of another one
 * 
 * \param[out] other_shape other shape to reset
 */
void BasicShape::contact_reset(BasicShape *other_shape)
{
    contact.reset();
    other_shape->contact_simple_reset();
}

/*! \brief get the speed in the inertial frame of a point attached the shape
 * 
 * \param[in] point_abs point position in the absolute frame [m]
 * \return requested speed vector [m/s]
 */
Vector3D BasicShape::get_point_speed(Vector3D const& point_abs)
{
    return V + cross_vector_3D(OM, point_abs - P);
}

/*! \brief add tangential contact point with relative speed
 * 
 * \param[in] other_shape other shape generating the contact
 * \param[in] point_abs point position in the absolute frame [m]
 */
void BasicShape::add_tg_point_speed(BasicShape *other_shape, Vector3D const& point_abs)
{
    contact.add_point(point_abs, get_point_speed(point_abs) - other_shape->get_point_speed(point_abs));
}

/*! \brief update force and torque with the contact volume
 */
void BasicShape::F_T_contact(BasicShape *other_basic)
{
    // variables declaration
    int nb_points;
    double F_N, F_N_local, cur_F_T, cur_norm_v;
    Vector3D normal_F, cur_tg_F;
    Vector3D center_force, norm_vec;
    Vector3D cur_point, cur_vel, cur_vel_norm;

    // application point
    center_force = contact.get_center();

    // normal force
    F_N = normal_force(this, other_basic, contact.get_volume(), contact.get_volume_dot());

    norm_vec = contact.get_norm_vec();
    normal_F = F_N * norm_vec;

    F += normal_F;
    T += cross_vector_3D(center_force - P, normal_F);
    
    // tangential forces
    nb_points = contact.get_nb_points();

    F_N_local = F_N / nb_points;

    for(int i=0; i<nb_points; i++)
    {
        cur_point = contact.get_point(i);
        cur_vel   = contact.get_point_vel(i);

        // projection on the tangential plane
        cur_vel -= scalar_vector_3D(cur_vel, norm_vec) * norm_vec;

        cur_norm_v = cur_vel.get_norm();

        if (cur_norm_v > 0.0)
        {
            cur_vel_norm = cur_vel / cur_norm_v;

            cur_F_T = tangentiel_force(this, other_basic, cur_norm_v, F_N_local);

            cur_tg_F = -cur_F_T * cur_vel_norm;

            F += cur_tg_F;
            T += cross_vector_3D(cur_point - P, cur_tg_F);
        }
    }
}

/*! \brief update the user possible states
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void BasicShape::user_states(MBSdataStruct *MBSdata)
{
    user_contact_states(this, MBSdata);
}

}
