#include "Point3D.hh"
#include <iostream>
#include <cmath>

namespace ContactGeom{

/// return the index of the max of three values
inline int max_three(double x0, double x1, double x2) { return (x0 < x1) ? ((x1 < x2) ? 2 : 1) : ((x0 < x2) ? 2 : 0); }

// square
inline double square(double x) { return (x*x); }

/*! \brief constructor
 */
Point3D::Point3D()
{

}

/*! \brief constructor
 * 
 * \param[in] point point position [m]
 * \param[in] point_dot point position derivative [m/s]
 */
Point3D::Point3D(Vector3D const& point, Vector3D const& point_dot)
{
    this->point = point;
    this->point_dot = point_dot;
}

/*! \brief destructor
 */
Point3D::~Point3D()
{

}

/*! \brief compute the centroid of a convex polygon
 * 
 * \pre the size of 'vertex_list' must be at least 3
 * \pre vertices must be ordered (clock-wise or counter-clockwise)
 * \param[in] vertex_list list of vertices
 * \param[out] centroid_pos position of the centroid
 */
void centroid_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D &centroid_pos)
{
    int n;
    double cur_area, tot_area;
    Vector3D point_0, cur_point_1, cur_point_2;

    n = vertex_list.size();
    centroid_pos.reset();
    tot_area = 0.0;

    // safety
    if (n < 3) { return; }

    point_0 = vertex_list[0]->get_point();

    // loop on all triangles
    for(int i=0; i<n-2; i++)
    {
        cur_point_1 = vertex_list[i+1]->get_point();
        cur_point_2 = vertex_list[i+2]->get_point();

        cur_area = area_triangle_3D(point_0, cur_point_1, cur_point_2);
        tot_area += cur_area;

        centroid_pos += cur_area * (point_0 + cur_point_1 + cur_point_2);
    }

    centroid_pos /= (3.0 * tot_area);
}

/*! \brief compute the centroid of a pyramid
 *
 * \pre the size of 'vertex_base' must be at least 3
 * \pre vertices must be ordered (clock-wise or counter-clockwise)
 * \param[in] vertex_base list of vertices of the base
 * \param[in] appex appex of the pyramid
 * \param[out] centroid_pos position of the centroid
 */
void centroid_pyramid_3D(std::vector<Point3D*> &vertex_base, Point3D *appex, Vector3D &centroid_pos)
{
    Vector3D centroid_base;
    Vector3D diff_vec;

    // safety
    if (vertex_base.size() < 3)
    {
        centroid_pos.reset();
        return;
    }

    // centroid base computation
    centroid_polygon_3D(vertex_base, centroid_base);

    diff_vec = appex->get_point() - centroid_base;

    centroid_pos = centroid_base + 0.25 * diff_vec;
}

/*! \brief compute the normal vector of a convex polygon
 * 
 * \pre the size of 'vertex_list' must be at least 3
 * \pre vertices must be ordered (clock-wise or counter-clockwise)
 * \param[in] vertex_list list of vertices
 * \param[out] normal_vec normal vector to the polygon 
 */
void normal_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D &normal_vec)
{
    int n;
    double norm;
    Vector3D vec_a, vec_b;

    n = vertex_list.size();
    normal_vec.reset();

    // safety
    if (n < 3) { return; }

    for(int i=0; i<n-2; i++)
    {
        vec_a = vertex_list[i+1]->get_point() - vertex_list[i]->get_point();
        vec_b = vertex_list[i+2]->get_point() - vertex_list[i+1]->get_point();

        normal_vec = cross_vector_3D(vec_a, vec_b);
        norm = normal_vec.get_norm();

        if (norm != 0.0)
        {
            normal_vec /= norm;
            return;
        }
    }
}

/*! \brief compute the normal vector of a convex polygon and its derivative
 * 
 * \pre the size of 'vertex_list' must be at least 3
 * \pre vertices must be ordered (clock-wise or counter-clockwise)
 * \param[in] vertex_list list of vertices
 * \param[out] normal_vec normal vector to the polygon 
 * \param[out] normal_vec_dot derivative of 'normal_vec'
 */
void normal_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D &normal_vec, Vector3D &normal_vec_dot)
{
    int n;
    double norm, norm_dot;
    Vector3D vec_a, vec_b, vec_a_dot, vec_b_dot;

    n = vertex_list.size();
    normal_vec.reset();
    normal_vec_dot.reset();

    // safety
    if (n < 3) { return; }

    for(int i=0; i<n-2; i++)
    {
        vec_a = vertex_list[i+1]->get_point() - vertex_list[i]->get_point();
        vec_b = vertex_list[i+2]->get_point() - vertex_list[i+1]->get_point();

        vec_a_dot = vertex_list[i+1]->get_point_dot() - vertex_list[i]->get_point_dot();
        vec_b_dot = vertex_list[i+2]->get_point_dot() - vertex_list[i+1]->get_point_dot();

        normal_vec = cross_vector_3D(vec_a, vec_b);
        normal_vec_dot = cross_vector_3D_dot(vec_a, vec_b, vec_a_dot, vec_b_dot);

        norm = normal_vec.get_norm();

        // safety
        if (!norm)
        {
            normal_vec.reset();
            normal_vec_dot.reset();
            return;
        }

        norm_dot = scalar_vector_3D(normal_vec, normal_vec_dot) / norm;

        normal_vec_dot = (normal_vec_dot*norm - normal_vec*norm_dot) / square(norm);
        normal_vec = normal_vec / norm;

        return;
    }
}

/*! \brief compute the area of a convex polygon
 *
 * \pre the size of 'vertex_list' must be at least 3
 * \pre vertices must be ordered (clock-wise or counter-clockwise)
 * \param[in] vertex_list list of vertices
 * \param[in] normal_vec normal vector to the polygon plane (normalized)
 * \param[in] normal_vec_dot derivative of 'normal_vec'
 * \param[out] area area of the polygon
 * \param[out] area_dot derivative of 'area'
 *
 * source: http://geomalgorithms.com/a01-_area.html
 */
void area_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D const& normal_vec, Vector3D const& normal_vec_dot, double &area, double &area_dot)
{
    int n;
    Vector3D sum_vec, sum_vec_dot;
    Point3D *vertex_a, *vertex_b;

    n = vertex_list.size();

    // safety
    if (n < 3)
    {
        area = 0.0;
        area_dot = 0.0;
        return;
    }

    for(int i=0; i<n; i++)
    {
        vertex_a = vertex_list[i];
        vertex_b = (i == n-1) ? vertex_list[0] : vertex_list[i+1];

        sum_vec += cross_vector_3D(vertex_a->get_point(), vertex_b->get_point());

        sum_vec_dot += cross_vector_3D_dot(vertex_a->get_point(), vertex_b->get_point(), vertex_a->get_point_dot(), vertex_b->get_point_dot());
    }

    // area and derivative
    area = 0.5 * scalar_vector_3D(normal_vec, sum_vec);

    area_dot = 0.5 * scalar_vector_3D_dot(normal_vec, sum_vec, normal_vec_dot, sum_vec_dot);

    // area must be positive
    if (area < 0.0)
    {
        area = -area;
        area_dot = -area_dot;
    }
}

/*! \brief compute the area of a triangle
 * 
 * \param[in] vertex_0 first vertex
 * \param[in] vertex_1 second vertex
 * \param[in] vertex_2 third vertex
 * \return area of the triangle
 */
double area_triangle_3D(Vector3D const& vertex_0, Vector3D const& vertex_1, Vector3D const& vertex_2)
{
    return (0.5 * cross_vector_3D(vertex_1 - vertex_0, vertex_2 - vertex_0).get_norm());
}

/*! \brief compute the volume of a pyramid
 *
 * \pre the size of 'vertex_base' must be at least 3
 * \pre vertices must be ordered (clock-wise or counter-clockwise)
 * \param[in] vertex_base list of vertices of the base
 * \param[in] normal_base normal vector to the base plane (normalized)
 * \param[in] normal_base_dot derivative of 'normal_base'
 * \param[in] appex appex of the pyramid
 * \param[out] volume volume of the pyramid
 * \param[out] volume_dot derivative of 'volume'
 *
 * source: http://geomalgorithms.com/a01-_area.html
 */
void volume_pyramid_3D(std::vector<Point3D*> &vertex_base, Vector3D const& normal_base, Vector3D const& normal_base_dot, Point3D *appex, double &volume, double &volume_dot)
{
    double h, h_dot;
    double area_base, area_base_dot;
    Vector3D diff_appex, diff_appex_dot;

    // safety
    if (vertex_base.size() < 3)
    {
        volume = 0.0;
        volume_dot = 0.0;
        return;
    }

    area_polygon_3D(vertex_base, normal_base, normal_base_dot, area_base, area_base_dot);

    diff_appex = appex->get_point() - vertex_base[0]->get_point();
    diff_appex_dot = appex->get_point_dot() - vertex_base[0]->get_point_dot();

    h = scalar_vector_3D(diff_appex, normal_base);
    h_dot = scalar_vector_3D_dot(diff_appex, normal_base, diff_appex_dot, normal_base_dot);

    // height must be positive
    if (h < 0.0)
    {
        h = -h;
        h_dot = -h_dot;
    }

    volume = (area_base * h) / 3.0;
    volume_dot = (area_base_dot*h + area_base*h_dot) / 3.0;
}

/*! \brief sort a vector of doubles from min to max and sort a vector of int in the same order
 * 
 * \pre vectors must have the same size
 * \param[in,out] sort_vec vector of double to sort
 * \param[out] copy_sort vector of int to sort as 'sort_vec'
 */
void sort_vector_polygon_3D(std::vector<double> &sort_vec, std::vector<int> &copy_sort)
{
    int n;
    int min_index, save_int;
    double min_val, save_double;

    n = sort_vec.size();

    // safety
    if (!n) { return; }

    for(int i=0; i<n; i++)
    {
        min_val = sort_vec[i];
        min_index = i;

        for(int j=i+1; j<n; j++)
        {
            if (sort_vec[j] < min_val)
            {
                min_val = sort_vec[j];
                min_index = j;
            }
        }

        // swap needed
        if (min_index > i)
        {
            save_double = sort_vec[i];
            save_int    = copy_sort[i];

            sort_vec[i]  = sort_vec[min_index];
            copy_sort[i] = copy_sort[min_index];

            sort_vec[min_index]  = save_double;
            copy_sort[min_index] = save_int;
        }
    }
}

/*! \brief apply the ordering of a list based on a given list
 * 
 * \pre both list must have the same size
 * \param[out] list_to_move list to re-order
 * \param[in] order_list list with indexes to get the correct order
 */
void apply_order_polygon_3D(std::vector<Point3D*> &list_to_move, std::vector<int> &order_list)
{
    unsigned int n;
    std::vector<Point3D*> copy_list;

    n = order_list.size();

    // safety
    if (list_to_move.size() != n) { return; }

    // copy list
    for(unsigned int i=0; i<n; i++)
    {
        copy_list.push_back(list_to_move[i]);
    }

    // apply changes
    for(unsigned int i=0; i<n; i++)
    {
        list_to_move[i] = copy_list[order_list[i]];
    }
}

/*! \brief order a list of vertices of a polygon
 * 
 * \param[in] vertex_list list of vertices
 * \param[out] order_list indexes of the vertices ordered
 * \param[in] normal vector of the polygon plane
 */
void order_vertex_polygon_3D(std::vector<Point3D*> &vertex_list, std::vector<int> &order_list, Vector3D const& norm_vec)
{
    int n;
    int proj_index;
    double mean_a, mean_b;
    double proj_a, proj_b;
    std::vector<double> angle_list;

    n = vertex_list.size();

    // safety
    if(!n) { return; }

    order_list.clear();

    for(unsigned int i=0; i<vertex_list.size(); i++)
    {
        order_list.push_back(i);
    }

    // cases no requiring any ordering
    if (n <= 3) { return; }

    proj_index = max_three(fabs(norm_vec.get_x()), fabs(norm_vec.get_y()), fabs(norm_vec.get_z()));

    // compute mean
    mean_a = 0.0;
    mean_b = 0.0;

    for(int i=0; i<n; i++)
    {
        switch (proj_index)
        {
            case 0:
                mean_a += vertex_list[i]->get_y();
                mean_b += vertex_list[i]->get_z();
                break;

            case 1:
                mean_a += vertex_list[i]->get_x();
                mean_b += vertex_list[i]->get_z();
                break;

            case 2:
                mean_a += vertex_list[i]->get_x();
                mean_b += vertex_list[i]->get_y();
                break;
        
            default:
                std::cout << "Error: unknown index !" << std::endl;
                return;
        }
    }

    mean_a /= n;
    mean_b /= n;

    for(int i=0; i<n; i++)
    {
        switch (proj_index)
        {
            case 0:
                proj_a = vertex_list[i]->get_y();
                proj_b = vertex_list[i]->get_z();               
                break;

            case 1:
                proj_a = vertex_list[i]->get_x();
                proj_b = vertex_list[i]->get_z();               
                break;

            case 2:
                proj_a = vertex_list[i]->get_x();
                proj_b = vertex_list[i]->get_y();
                break;
        
            default:
                std::cout << "Error: unknown index !" << std::endl;
                return;
        }

        angle_list.push_back(atan2(proj_b - mean_b, proj_a - mean_a));
    }

    sort_vector_polygon_3D(angle_list, order_list);
}

}
