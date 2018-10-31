#include "PlaneShape.hh"
#include "SphereShape.hh"
#include "CuboidShape.hh"

namespace ContactGeom{

/*! \brief constructor
 *
 * \param[in] norm_vec vector normal to the surface
 * \param[in] rel_position relative position vector from the reference position of RigidShape to the shape reference position [m]
 * \param[in] parent_shape parent shape
 */
PlaneShape::PlaneShape(Vector3D const& normal_vec, Vector3D const& rel_position, int rigid_flag, GeometryShape *parent_shape): BasicShape(rel_position, 0, rigid_flag, parent_shape)
{
    type = GEOM_PLANE;

    set_absolute_norm(normal_vec);

    flag_z_norm = ( (norm_vec.get_x() == 0.0) && (norm_vec.get_y() == 0.0) && (norm_vec.get_z() == 1.0) );
    
    norm_vec_local = norm_vec;
    norm_vec_dot = get_nul_vector_3D();

    P = rel_position;

    update_d();

    norm_vec.ortho_matrix_3D(u_local, v_local);
}

/*! \brief constructor
 *
 * \param[in] a value 'a' in the equation 'ax + by + cz + d = 0'
 * \param[in] b value 'b' in the equation 'ax + by + cz + d = 0'
 * \param[in] c value 'c' in the equation 'ax + by + cz + d = 0'
 * \param[in] d value 'd' in the equation 'ax + by + cz + d = 0'
 * \param[in] parent_shape parent shape
 */
PlaneShape::PlaneShape(double a, double b, double c, double d, int rigid_flag, GeometryShape *parent_shape): BasicShape(get_nul_vector_3D(), 0, rigid_flag, parent_shape)
{
    type = GEOM_PLANE;

    double norm_square = a*a + b*b + c*c;

    P = get_vector_3D((a*d)/norm_square, (b*d)/norm_square, (c*d)/norm_square);

    set_absolute_norm(get_vector_3D(a, b, c));

    flag_z_norm = ( (norm_vec.get_x() == 0.0) && (norm_vec.get_y() == 0.0)  && (norm_vec.get_z() == 1.0) );

    norm_vec_local = norm_vec;
    norm_vec_dot = get_nul_vector_3D();

    update_d();

    set_rel_position(P);

    u_local = get_vector_3D(1.0, 0.0, 0.0);
    v_local = get_vector_3D(0.0, 1.0, 0.0);
}

/*! \brief constructor with a ground whose normal is aligned with the z+ axis
 * 
 * \param[in] z_height distance from rigid reference position along the z+ axis
 * \param[in] parent_shape parent shape
 */
PlaneShape::PlaneShape(double z_height, int rigid_flag, GeometryShape *parent_shape): BasicShape(get_vector_3D(0.0, 0.0, z_height), rigid_flag, parent_shape)
{
    type = GEOM_PLANE;

    flag_z_norm = 1;

    norm_vec_local = get_vector_3D(0.0, 0.0, 1.0);
    norm_vec = get_vector_3D(0.0, 0.0, 1.0);
    norm_vec_dot = get_nul_vector_3D();

    P = get_vector_3D(0.0, 0.0, z_height);

    d = -z_height;
    d_dot = 0.0;

    u_local = get_vector_3D(1.0, 0.0, 0.0);
    v_local = get_vector_3D(0.0, 1.0, 0.0);
}

/*! \brief destructor
 */
PlaneShape::~PlaneShape()
{

}

/*! \brief set the normal vector normalized with a vector in absolute coordinates
 * 
 * \param[in] normal_vec vector normal to the surface
 */
void PlaneShape::set_absolute_norm(Vector3D const& normal_vec)
{
    double norm;

    norm = normal_vec.get_norm();

    if (norm <= 0.0)
    {
        std::cout << "Error: norm vector cannot be set to 0 !" << std::endl;
        exit(EXIT_FAILURE);
    }

    norm_vec = normal_vec / norm;
}

/*! \brief update the constant d
 */
void PlaneShape::update_d()
{
    d = -scalar_vector_3D(norm_vec, P);
    d_dot = -scalar_vector_3D_dot(norm_vec, P, norm_vec_dot, V);
}

/*! \brief update the check related variables
 */
void PlaneShape::update_check()
{
    // check sphere not applicable
    return;
}

/*! \brief update the kinematics values of the basic body
 * 
 * \param[in] parent_P  absolute position vector of parent RigidShape [m]
 * \param[in] parent_V  absolute velocity vector of parent RigidShape [m/s]
 * \param[in] parent_OM absolute angle velocity vector of parent RigidShape [rad/s]
 * \param[in] parent_R  absolute rotation matrix of parent RigidShape [-]
 */
void PlaneShape::update_kinematics(Vector3D const& parent_P, Vector3D const& parent_V, Vector3D const& parent_OM, RotMatrix const& parent_R)
{
    Vector3D u, v, u_dot, v_dot;
    double norm_normal;

    R = parent_R;
    R_T = get_trans_rot_matrix(parent_R);

    u = R_T * u_local;
    v = R_T * v_local;

    u_dot = cross_vector_3D(parent_OM, u);
    v_dot = cross_vector_3D(parent_OM, v);

    norm_vec = cross_vector_3D(u, v);
    norm_vec_dot = cross_vector_3D_dot(u, v, u_dot, v_dot);

    norm_normal = norm_vec.get_norm();

    if (norm_normal > 0.0)
    {
        norm_vec /= norm_normal;
        norm_vec_dot /= norm_normal; // norm_normal is a constant
    }

    flag_z_norm = ( (norm_vec.get_x() == 0.0) && (norm_vec.get_y() == 0.0) && (norm_vec.get_z() == 1.0) );

    update_d();

    BasicShape::update_kinematics(parent_P, parent_V, parent_OM, parent_R);
}

/*! \brief update the forces and torques with another basic shape
 * 
 * \param[in,out] other_shape other basic shape
 * \return 1 if contact, 0 otherwise
 */
int PlaneShape::update_F_T_basic(BasicShape *other_shape)
{
    switch (other_shape->get_type())
    {
        case GEOM_PLANE : // no contact between two infinite planes
            return 0;
            break;

        case GEOM_SPHERE :
            return update_F_T_sphere(other_shape);
            break;

        case GEOM_CUBOID :
            return update_F_T_cuboid(other_shape);
            break;
    
        default:
            std::cout << "Error: unknown basic shape !" << std::endl;
            exit(EXIT_FAILURE);
            break;
    }
}

/*! \brief update the forces and torques with another sphere shape
 * 
 * \param[in,out] other_shape other sphere shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int PlaneShape::update_F_T_sphere(BasicShape *other_shape)
{
    // update computed in 'SphereShape.cc'
    return other_shape->update_F_T_basic(this);
}

/*! \brief update the forces and torques with another cuboid shape
 * 
 * \param[in,out] other_shape other cuboid shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int PlaneShape::update_F_T_cuboid(BasicShape *other_shape)
{
    // update computed in 'CuboidShape.cc'
    return other_shape->update_F_T_basic(this);
}

/*! \brief compute the closest distance vector from this plane to a point
 * 
 * \param[in] point point to reach [m]
 * \return dist closest distance [m]
 */
double PlaneShape::dist_point(Vector3D const& point)
{
    if (flag_z_norm)
    {
        return (point.get_z() + d);
    }
    else
    {
        return scalar_vector_3D(norm_vec, point - P);
    }
}

/*! \brief compute the closest distance vector from this plane to a point
 * 
 * \param[in] point point to reach [m]
 * \param[in] point_dot time derivative of 'point' [m/s]
 * \param[out] closest_point closest point on the plane [m]
 * \param[out] norm_res normal vector of the plane
 * \param[out] dist closest distance [m]
 * \param[out] dist_dot closest distance derivative [m/s] 
 */
void PlaneShape::dist_point(Vector3D const& point, Vector3D const& point_dot, Vector3D & closest_point, Vector3D & norm_res, double & dist, double & dist_dot)
{
    Vector3D diff_point;

    // in case the normal is aligned with the +z axis
    if (flag_z_norm)
    {
        dist = point.get_z() + d;

        dist_dot = point_dot.get_z() + d_dot;

        closest_point = get_vector_3D(point.get_x(), point.get_y(), -d);
    }
    else // other planes
    {
        diff_point = point - P;

        dist = scalar_vector_3D(norm_vec, diff_point);

        dist_dot = scalar_vector_3D(norm_vec_dot, diff_point) + scalar_vector_3D(norm_vec, point_dot - V);

        closest_point = point - dist*norm_vec;
    }

    norm_res = norm_vec;
}

/*! \brief check if a point is inside the corresponding shape
 * 
 * \param[in] point point to check
 * \return 1 if inside, 0 otherwise
 */
int PlaneShape::point_inside(Vector3D const& point)
{
    Vector3D vec;

    if (flag_z_norm)
    {
        return point.get_z() <= -d;
    }
    else
    {
        vec = point - P;

        return (scalar_vector_3D(vec, norm_vec) <= 0.0);
    }
}

}
