#include "Polygon3D.hh"
#include <iostream>
#include <stdlib.h>

namespace ContactGeom{

/*! \brief constructor
 */
Polygon3D::Polygon3D()
{
    d = 0.0;
    d_dot = 0.0;

    flag_z_norm = 0;
}

/*! \brief destructor
 */
Polygon3D::~Polygon3D()
{

}

/*! \brief add new segment to the polygon
 * 
 * \param[in,out] seg segment to add
 * \param[in] inside_point point inside the volume (to check normal sens)
 */
void Polygon3D::add_segment(Segment3D *seg, Vector3D const& inside_point)
{
    segments_list.push_back(seg);

    update_kinematics(inside_point);
}

/*! \brief add new point to the polygon
 * 
 * \param[in,out] point point to add
 */
void Polygon3D::add_point(Point3D *point)
{
    points_list.push_back(point);
}

/*! \brief get the first point
 * 
 * \return first point
 */
Point3D* Polygon3D::get_first_point()
{
    if (!points_list.size())
    {
        std::cout << "Error: this Polygon3D does not have any point !" << std::endl;
        exit(EXIT_FAILURE);
    }

    return points_list[0];
}

/*! \brief update the kinematics
 *
 * \param[in] inside_point point inside the volume (to check normal sens)
 */
void Polygon3D::update_kinematics(Vector3D const& inside_point)
{
    double cross_prod_norm;
    Vector3D cross_prod, cross_prod_dot;
    Segment3D *first_seg, *second_seg;

    if (segments_list.size() >= 2)
    {
        for(unsigned int i=1; i<segments_list.size(); i++)
        {
            first_seg  = segments_list[i-1];
            second_seg = segments_list[i];

            cross_prod = cross_vector_3D(first_seg->get_diff_vec(), second_seg->get_diff_vec());
            cross_prod_norm = cross_prod.get_norm(); // because the shape is rigid, the segments length and relative position is fixes, so this norm is constant

            if (cross_prod_norm > 0.0)
            {
                cross_prod_dot = cross_vector_3D_dot(first_seg->get_diff_vec(), second_seg->get_diff_vec(), 
                                    first_seg->get_diff_vec_dot(), second_seg->get_diff_vec_dot());

                normal_vec     = cross_prod     / cross_prod_norm;
                normal_vec_dot = cross_prod_dot / cross_prod_norm;

                if (scalar_vector_3D(normal_vec, first_seg->get_pos(0) - inside_point) < 0.0)
                {
                    normal_vec = -normal_vec;
                    normal_vec_dot = -normal_vec_dot;
                }

                d     = -scalar_vector_3D(    normal_vec, first_seg->get_pos(0));
                d_dot = -scalar_vector_3D_dot(normal_vec, first_seg->get_pos(0), normal_vec_dot, first_seg->get_pos_dot(0));

                flag_z_norm = ((normal_vec.get_x() == 0.0) && (normal_vec.get_y() == 0.0) && (normal_vec.get_z() == 1.0));

                return;
            }
        }
    }
}

/*! \brief compute the closest distance vector from the plane of this polygon to a point
 * 
 * \param[in] point point to reach [m]
 * \param[in] point_dot point to reach velocity [m/s]
 * \param[out] closest_point closest point of the plane
 * \param[out] dist closest distance [m]
 * \param[out] dist_dot derivative of 'dist'
 *
 * attention: 'dist' and 'dist_dot' signs may be uncorrect !
 */
void Polygon3D::dist_point_plane(Vector3D const& point, Vector3D const& point_dot, Vector3D & closest_point, double &dist, double &dist_dot)
{
    Vector3D diff_vec;

    if (flag_z_norm)
    {
        dist = point.get_z() + d;
        dist_dot = point_dot.get_z() + d_dot;

        closest_point = get_vector_3D(point.get_x(), point.get_y(), -d);        
    }
    else
    {
        // safety
        if (points_list.size() <= 0)
        {
            std::cout << "Error: 'points_list' is empty !" << std::endl;
            exit(EXIT_FAILURE);
        }

        diff_vec = point - points_list[0]->get_point();

        dist = scalar_vector_3D(normal_vec, diff_vec);
        dist_dot = scalar_vector_3D_dot(normal_vec, diff_vec, normal_vec_dot, point_dot - points_list[0]->get_point_dot());

        closest_point = point - dist*normal_vec;
    }
}

}
