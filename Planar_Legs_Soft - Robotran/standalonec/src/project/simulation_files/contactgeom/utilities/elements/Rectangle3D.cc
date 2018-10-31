#include "Rectangle3D.hh"
#include <iostream>
#include <stdlib.h>

namespace ContactGeom{

inline int opp_i(int i) { return (i > 0) ? 0 : 1; }
inline int is_in_range(double x, double x_min, double x_max) { return ((x_min <= x) && (x <= x_max)); }

/*! \brief constructor
 */
Rectangle3D::Rectangle3D()
{
    nb_seg = 4;
    flag_two_vec = 0;

    aa = 0.0;
    bb = 0.0;
    ab = 0.0;

    denom = 0.0;
}

/*! \brief destructor
 */
Rectangle3D::~Rectangle3D()
{

}

/*! \brief add new segment to the polygon
 * 
 * \param[in,out] seg segment to add
 * \param[in] inside_point point inside the volume (to check normal sens)
 */
void Rectangle3D::add_segment(Segment3D *seg, Vector3D const& inside_point)
{
    Point3D *cur_point;

    if (segments_list.size() < nb_seg)
    {
        Polygon3D::add_segment(seg, inside_point);

        // first segment added
        if (segments_list.size() == 1)
        {
            two_vec_points[0] = seg->get_point(0);
            two_vec_points[1] = seg->get_point(1);
        }
        else if (!flag_two_vec)
        {
            for(int i=0; i<2; i++)
            {
                cur_point = seg->get_point(i);

                if (cur_point == two_vec_points[0])
                {
                    two_vec_points[2] = two_vec_points[1];
                    two_vec_points[1] = two_vec_points[0];
                    two_vec_points[0] = seg->get_point(opp_i(i));
                    flag_two_vec = 1;
                    break;
                }
                else if (cur_point == two_vec_points[1])
                {
                    two_vec_points[2] = seg->get_point(opp_i(i));
                    flag_two_vec = 1;
                    break;
                }
            }
        }

        if ((!flag_two_vec) && (segments_list.size() >= nb_seg))
        {
            std::cout << "Error: two vectors not initialized for Rectangle3D !" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

/*! \brief add new point to the polygon
 * 
 * \param[in,out] point point to add
 */
void Rectangle3D::add_point(Point3D *point)
{
    if (points_list.size() < nb_seg)
    {
        Polygon3D::add_point(point);
    }
}

/*! \brief update the kinematics
 *
 * \param[in] inside_point point inside the volume (to check normal sens)
 */
void Rectangle3D::update_kinematics(Vector3D const& inside_point)
{
    Polygon3D::update_kinematics(inside_point);

    if (flag_two_vec)
    {
        vec_a = two_vec_points[0]->get_point() - two_vec_points[1]->get_point();
        vec_b = two_vec_points[2]->get_point() - two_vec_points[1]->get_point();

        aa = vec_a.get_square_norm();
        bb = vec_b.get_square_norm();

        ab = scalar_vector_3D(vec_a, vec_b);

        denom = ab*ab - aa*bb;
    }
}

/*! \brief compute the contact point with a segment
 * 
 * \param[in] seg segment in contact with the rectangle
 * \param[out] point_res resulting contact point
 * \return 1 if contact, 0 otherwise
 */
int Rectangle3D::contact_segment(Segment3D &seg, Point3D &point_res)
{
    if (!seg.intersect_plane(normal_vec, normal_vec_dot, d, d_dot, flag_z_norm, point_res))
    {
        return 0;
    }

    if (!check_point_inside(point_res.get_point()))
    {
        point_res.clear();
        return 0;
    }

    return 1;
}

/*! \brief check if a point located inside the plane of this rectangle is inside the rectangle surface
 * 
 * \param[in] point point to check
 * \return 1 if inside, 0 otherwise
 *
 * source: http://geomalgorithms.com/a06-_intersect-2.html
 */
int Rectangle3D::check_point_inside(Vector3D const& point)
{
    double ac, bc;
    double s_star, t_star;
    Vector3D vec_c;

    // safety
    if ((!flag_two_vec) || (!denom))
    {
        return 0;
    }

    vec_c = point - two_vec_points[1]->get_point();

    ac = scalar_vector_3D(vec_a, vec_c);
    bc = scalar_vector_3D(vec_b, vec_c);

    s_star = (ab*bc - bb*ac) / denom;
    t_star = (ab*ac - aa*bc) / denom;

    return (is_in_range(s_star, 0.0, 1.0) &&is_in_range(t_star, 0.0, 1.0));
}

}
