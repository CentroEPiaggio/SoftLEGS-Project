/*! 
 * \author Nicolas Van der Noot
 * \file Point3D.hh
 * \brief Point3D class
 */

#ifndef _POINT_3D_HH_
#define _POINT_3D_HH_

#include "Vector3D.hh"
#include <vector>

namespace ContactGeom{

class Point3D
{
    public:
        Point3D();
        Point3D(Vector3D const& point, Vector3D const& point_dot);
        ~Point3D();

        /// update the kinematics
        void update_kinematics(Vector3D const& new_pos, Vector3D const& new_dot)
        {
            point = new_pos;
            point_dot = new_dot;
        }

        Vector3D& get_point() { return point; }
        Vector3D& get_point_dot() { return point_dot; }

        double get_x() const { return point.get_x(); }
        double get_y() const { return point.get_y(); }
        double get_z() const { return point.get_z(); }

        double get_x_dot() const { return point_dot.get_x(); }
        double get_y_dot() const { return point_dot.get_y(); }
        double get_z_dot() const { return point_dot.get_z(); }

        // set all to 0.0
        void clear()
        {
            point.reset();
            point_dot.reset();
        }

        Point3D* get_ptr() { return this; }

    private:
        Vector3D point;     ///< point position [m]
        Vector3D point_dot; ///< point time derivative [m/s]
};

/*! \brief sum two points with the + operation
 * 
 * \param[in] a first point to sum
 * \param[in] b second point to sum
 * \return sum of the two points
 */
inline Point3D operator+(Point3D &a, Point3D &b)
{
    return Point3D(a.get_point() + b.get_point(), a.get_point_dot() + b.get_point_dot());
}

/*! \brief difference of two points with the - operation
 * 
 * \param[in] a first point for the difference
 * \param[in] b second point for the difference
 * \return difference of the two points
 */
inline Point3D operator-(Point3D &a, Point3D &b)
{
    return Point3D(a.get_point() - b.get_point(), a.get_point_dot() - b.get_point_dot());
}

/*! \brief get middle between two points
 * 
 * \param[in] a first point
 * \param[in] b second point
 * \return mid-point between the two points
 */
inline Point3D get_mid_point(Point3D &a, Point3D &b)
{
    return Point3D(0.5*(a.get_point() + b.get_point()), 0.5*(a.get_point_dot() + b.get_point_dot()));
}

// function prototypes
void area_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D const& normal_vec, Vector3D const& normal_vec_dot, double &area, double &area_dot);
double area_triangle_3D(Vector3D const& vertex_0, Vector3D const& vertex_1, Vector3D const& vertex_2);
void centroid_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D &centroid_pos);
void centroid_pyramid_3D(std::vector<Point3D*> &vertex_base, Point3D *appex, Vector3D &centroid_pos);
void volume_pyramid_3D(std::vector<Point3D*> &vertex_base, Vector3D const& normal_base, Vector3D const& normal_base_dot, Point3D *appex, double &volume, double &volume_dot);
void normal_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D &normal_vec);
void normal_polygon_3D(std::vector<Point3D*> &vertex_list, Vector3D &normal_vec, Vector3D &normal_vec_dot);
void order_vertex_polygon_3D(std::vector<Point3D*> &vertex_list, std::vector<int> &order_list, Vector3D const& norm_vec);
void sort_vector_polygon_3D(std::vector<double> &sort_vec, std::vector<int> &copy_sort);
void apply_order_polygon_3D(std::vector<Point3D*> &list_to_move, std::vector<int> &order_list);

}
#endif
