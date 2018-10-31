#include "GeometryShape.hh"

namespace ContactGeom{

/// square function
inline double square(double x) { return x*x; }

/// max function
inline double max(double x, double y) { return (x < y) ? y : x; }

/*! \brief constructor
 *
 * \param[in] parent_shape parent shape
 */
GeometryShape::GeometryShape(GeometryShape *parent_shape)
{
    type = GEOM_MOTHER;

    all_contact_flag = 1;

    check_flag   = 0;
    check_radius = 0.0;
    check_center = get_nul_vector_3D();

    update_check_min_max();

    this->parent_shape = parent_shape;

    genealogy.push_back(this);
}

/*! \brief constructor
 *
 * \param[in] check_flag 1 if check sphere applicable, 0 otherwise
 * \param[in] parent_shape parent shape
 */
GeometryShape::GeometryShape(int check_flag, GeometryShape *parent_shape)
{
    type = GEOM_MOTHER;

    all_contact_flag = 1;

    this->check_flag = check_flag;
    check_radius = 0.0;
    check_center = get_nul_vector_3D();

    update_check_min_max();

    this->parent_shape = parent_shape;

    genealogy.push_back(this);
}

/*! \brief constructor
 *
 * \param[in] check_flag 1 if check sphere applicable, 0 otherwise
 * \param[in] check_radius radius of the check sphere (if applicable) [m]
 * \param[in] check_center position of the center of the check sphere (if applicable) [m]
 * \param[in] parent_shape parent shape
 */
GeometryShape::GeometryShape(int check_flag, double check_radius, Vector3D const& check_center, GeometryShape *parent_shape)
{
    type = GEOM_MOTHER;

    all_contact_flag = 1;

    this->check_flag   = check_flag;
    this->check_radius = check_radius;
    this->check_center = check_center;

    update_check_min_max();

    this->parent_shape = parent_shape;

    genealogy.push_back(this);
}

/*! \brief destructor
 */
GeometryShape::~GeometryShape()
{

}

/*! \brief print the genealogy of this body
 */
void GeometryShape::list_genealogy()
{
    for(unsigned int i=0; i<genealogy.size()-1; i++)
    {
        std::cout << genealogy[i]  << " ; ";
    }

    if (genealogy.size() > 0)
    {
        std::cout << genealogy.back() << std::endl;
    }
}

/*! \brief update the minimal and maximal x,y,z values of the check sphere (if applicable)
 */
void GeometryShape::update_check_min_max()
{
    // no check possible
    if (!check_flag)
    {
        return;
    }

    for(int i=0; i<3; i++)
    {
        check_min_max[i][0] = check_center.get_comp(i) - check_radius; // min value
        check_min_max[i][1] = check_center.get_comp(i) + check_radius; // max value
    }
}

/*! \brief check if two shapes can be in contact, based on their minimal and maximal values
 * 
 * \param[in] other_shape other shape to check for possible contact
 * \return 1 if possible contact, 0 otherwise
 */
int GeometryShape::possible_min_max_contact(GeometryShape *other_shape)
{
    // at least one of the two shapes cannot use sphere check
    if ( (!check_flag) || (!other_shape->get_check_flag()) )
    {
        return 1;
    }

    // check all dimensions
    for(int i=0; i<3; i++)
    {
        if ( (check_min_max[i][0] >= other_shape->get_check_min_max(i,1)) || (check_min_max[i][1] <= other_shape->get_check_min_max(i,0)) )
        {
            return 0;
        }
    }

    return 1;
}

/*! \brief check if two shapes can be in contact, based on their check spheres
 * 
 * \param[in] other_shape other shape to check for possible contact
 * \return 1 if possible contact, 0 otherwise
 */
int GeometryShape::possible_sphere_contact(GeometryShape *other_shape)
{
    // at least one of the two shapes cannot use sphere check
    if ( (!check_flag) || (!other_shape->get_check_flag()) )
    {
        return 1;
    }

    // centers of the two shapes are close enough
    return ( get_square_dist_vectors_3D(check_center, other_shape->get_check_center()) < square(check_radius + other_shape->get_check_radius()) );
}

/*! \brief compute a new sphere wrapping two other spheres
 * 
 * \param[in] radius_1 radius of the first sphere
 * \param[in] center_1 center of the first sphere
 * \param[in] radius_2 radius of the second sphere
 * \param[in] center_2 center of the second sphere
 * \param[out] new_radius radius of the new wrapping sphere
 * \param[out] new_center center of the new wrapping sphere
 */
void GeometryShape::wrapping_spheres(double radius_1, Vector3D center_1, double radius_2, Vector3D center_2, double & new_radius, Vector3D & new_center)
{
    double lambda_1, lambda_2, dist;
    Vector3D diff_vec, extr_1, extr_2;

    dist = get_dist_vectors_3D(center_1, center_2);

    // safety
    if (!dist)
    {
        new_radius = max(radius_1, radius_2);
        new_center = center_1;
        return;
    }

    lambda_1 = -(radius_1 / dist);
    lambda_2 = 1.0 + (radius_2 / dist);

    diff_vec = center_2 - center_1;

    extr_1 = center_1 + (diff_vec * lambda_1);
    extr_2 = center_1 + (diff_vec * lambda_2);

    new_center = mid_vectors_3D(extr_1, extr_2);

    diff_vec = extr_2 - extr_1;

    new_radius = 0.5 * diff_vec.get_norm();
}

/*! \brief add prohibited shape to the list
 * 
 * \param[in,out] other_shape prohibited shape to add
 */
void GeometryShape::add_prohibited(GeometryShape *other_shape)
{
    for(int i=0; i<other_shape->get_size_genealogy(); i++)
    {
        prohibited_list.push_back(other_shape->get_genealogy_elem(i));
    }

    for(unsigned int i=0; i<genealogy.size(); i++)
    {
        other_shape->add_prohibited_simple(genealogy[i]);
        
    }
}

/*! \brief add no prohibited shape to the list
 * 
 * \param[in,out] other_shape no prohibited shape to add
 */
void GeometryShape::add_no_prohibited(GeometryShape *other_shape)
{
    for(int i=0; i<other_shape->get_size_genealogy(); i++)
    {
        no_prohibited_list.push_back(other_shape->get_genealogy_elem(i));
    }

    for(unsigned int i=0; i<genealogy.size(); i++)
    {
        other_shape->add_no_prohibited_simple(genealogy[i]);
        
    }
}

/*! \brief check if contact is possible
 * 
 * \param[in] other_shape other shape to check
 * \return 1 if possible contact, 0 otherwise
 */
int GeometryShape::contact_possible(GeometryShape *other_shape)
{
    int found_shape;

    // check prohibited lists
    if (all_contact_flag)
    {
        for(unsigned int i=0; i<prohibited_list.size(); i++)
        {
            if (prohibited_list[i] == other_shape)
            {
                return 0;
            }
        }
    }
    else
    {
        found_shape = 0;

        for(unsigned int i=0; i<no_prohibited_list.size(); i++)
        {
            if (no_prohibited_list[i] == other_shape)
            {
                found_shape = 1;
                break;
            }
        }

        if (!found_shape)
        {
            return 0;
        }
    }

    // check: min-max
    if (possible_min_max_contact(other_shape))
    {
        // check: sphere
        if (possible_sphere_contact(other_shape))
        {
            return 1;
        }
    }

    return 0;
}

}
