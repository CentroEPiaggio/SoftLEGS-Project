#include "SphereShape.hh"
#include "PlaneShape.hh"
#include "CuboidShape.hh"

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] radius radius of the sphere [m]
 * \param[in] rel_position relative position vector from the reference position of RigidShape to the shape reference position [m]
 * \param[in] parent_shape parent shape
 */
SphereShape::SphereShape(double radius, Vector3D const& rel_position, int rigid_flag, GeometryShape *parent_shape): BasicShape(rel_position, 1, radius, get_nul_vector_3D(), rigid_flag, parent_shape) 
{
    type = GEOM_SPHERE;

    this->radius = radius;

    square_radius = radius*radius;
}

/*! \brief destructor
 */
SphereShape::~SphereShape()
{

}

/*! \brief update the check related variables
 */
void SphereShape::update_check()
{
    set_check_center(P);

    update_check_min_max();
}

/*! \brief update the forces and torques with another basic shape
 * 
 * \param[in,out] other_shape other basic shape
 * \return 1 if contact, 0 otherwise
 */
int SphereShape::update_F_T_basic(BasicShape *other_shape)
{
    switch (other_shape->get_type())
    {
        case GEOM_PLANE  :
            return update_F_T_plane(other_shape);
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

/*! \brief update the forces and torques with another plane shape
 * 
 * \param[in,out] other_shape other plane shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int SphereShape::update_F_T_plane(BasicShape *other_shape)
{
    double dist, dist_dot, h, h_dot, r_surf, h_abs;
    PlaneShape *other_plane;
    Vector3D center_point, norm_plane;
    Vector3D axis_1, axis_2;

    other_plane = static_cast<PlaneShape*>(other_shape);

    other_plane->dist_point(P, V, center_point, norm_plane, dist, dist_dot);

    // contact
    if (fabs(dist) <= radius)
    {
        h = radius - dist;
        h_dot = -dist_dot;

        // volume
        contact.set_volume(( M_PI*h*h*(3.0*radius-h) ) / 3.0);
        contact.set_volume_dot(M_PI*h*(2*radius*h_dot - h*h_dot));

        // center force
        contact.set_center(center_point);

        // normal vector
        contact.set_norm_vec(norm_plane);

        h_abs = fabs(h);

        // radius of the contact surface
        r_surf = sqrt(h_abs*(2.0*radius-h_abs));

        // normalized tangential plane vectors (orthogonal)
        norm_plane.ortho_matrix_3D(axis_1, axis_2);

        // contact points for tangential forces
        add_tg_point_speed(other_shape, center_point + r_surf*axis_1);
        add_tg_point_speed(other_shape, center_point - r_surf*axis_1);
        add_tg_point_speed(other_shape, center_point + r_surf*axis_2);
        add_tg_point_speed(other_shape, center_point - r_surf*axis_2);

        // copy result to other shape
        contact.copy_reverse(other_shape->get_contact());

        return 1;
    }

    // no contact
    return 0;
}

/*! \brief update the forces and torques with another sphere shape
 * 
 * \param[in,out] other_shape other sphere shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int SphereShape::update_F_T_sphere(BasicShape *other_shape)
{
    double dist, dist_dot;
    double alpha, alpha_dot;
    double radius_2;
    double diff_square_rad;
    double h1, h2, h1_dot, h2_dot;
    double h_abs, r_surf;
    Vector3D diff_center, diff_center_dot, norm_vec, center_point;
    Vector3D axis_1, axis_2;

    SphereShape *other_sphere;

    other_sphere = static_cast<SphereShape*>(other_shape);

    // distance between centers
    diff_center = other_shape->get_P() - P;

    dist = diff_center.get_norm();

    // safety
    if (dist <= 0.0)
    {
        return 0;
    }

    radius_2 = other_sphere->get_radius();

    // spheres not in contact
    if (dist > radius + radius_2)
    {
        return 0;
    }

    // derivative of the distance
    diff_center_dot = other_shape->get_V() - V;

    dist_dot = scalar_vector_3D(diff_center, diff_center_dot) / dist;

    // distance from first center to contact surface
    diff_square_rad = square_radius - other_sphere->get_square_radius();

    alpha = 0.5 * (dist + diff_square_rad/dist);

    alpha_dot = 0.5 * dist_dot * (1.0 - diff_square_rad/(dist*dist));

    // heights of the spherical caps
    h1 = radius - alpha;

    h2 = radius_2 - (dist - alpha);

    h1_dot = -alpha_dot;

    h2_dot = -(dist_dot - alpha_dot);

    // volume
    contact.set_volume(( M_PI*h1*h1*(3.0*radius-h1) + M_PI*h2*h2*(3.0*radius_2-h2) ) / 3.0);
    contact.set_volume_dot(M_PI*( h1*(2*radius*h1_dot - h1*h1_dot) + h2*(2*radius_2*h2_dot - h2*h2_dot) ));

    // center force
    center_point = P + diff_center * (alpha / dist);
    contact.set_center(center_point);

    // normal vector
    norm_vec = -(diff_center/dist);
    contact.set_norm_vec(norm_vec);

    h_abs = fabs(h1);

    if (h_abs <= 2.0*radius)
    {
        // radius of the contact surface
        r_surf = sqrt(h_abs*(2.0*radius-h_abs));

        // normalized tangential plane vectors (orthogonal)
        norm_vec.ortho_matrix_3D(axis_1, axis_2);

        // contact points for tangential forces
        add_tg_point_speed(other_shape, center_point + r_surf*axis_1);
        add_tg_point_speed(other_shape, center_point - r_surf*axis_1);
        add_tg_point_speed(other_shape, center_point + r_surf*axis_2);
        add_tg_point_speed(other_shape, center_point - r_surf*axis_2);
    }

    // copy result to other shape
    contact.copy_reverse(other_shape->get_contact());

    return 1;
}

/*! \brief update the forces and torques with another cuboid shape
 * 
 * \param[in,out] other_shape other cuboid shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int SphereShape::update_F_T_cuboid(BasicShape *other_shape)
{
    // update computed in 'CuboidShape.cc'
    return other_shape->update_F_T_basic(this);
}

/*! \brief check if a point is inside the corresponding shape
 * 
 * \param[in] point point to check
 * \return 1 if inside, 0 otherwise
 */
int SphereShape::point_inside(Vector3D const& point)
{
    Vector3D diff_vec = point - P;

    return (diff_vec.get_square_norm() <= square_radius);
}

}
