/*! 
 * \author Nicolas Van der Noot
 * \file Segment3D.hh
 * \brief Segment3D class
 */

#ifndef _SEGMENT_3D_HH_
#define _SEGMENT_3D_HH_

#include "Point3D.hh"

namespace ContactGeom{

/*! \brief segment in 3D space
 */
class Segment3D
{
    public:
        Segment3D();
        Segment3D(Point3D *point_A, Point3D *point_B);
        ~Segment3D();

        void update_kinematics();

        void add_point(Point3D *new_point);

        void renew_seg(Point3D *point_A, Point3D *point_B);

        /// return the requested point
        Point3D* get_point(int i) { return point[i]; }

        /// return the position of the ith element
        Vector3D& get_pos(int i)     { return point[i]->get_point(); }

        /// return the position derivative of the ith element
        Vector3D& get_pos_dot(int i) { return point[i]->get_point_dot(); }

        Vector3D& get_diff_vec() { return diff_vec; }
        Vector3D& get_diff_vec_dot() { return diff_vec_dot; }

        double get_diff_vec_norm() const { return diff_vec_norm; }
        double get_diff_vec_norm_dot() const { return diff_vec_norm_dot; }

        int intersect_plane(Vector3D const& normal_vec, Vector3D const& normal_vec_dot, double d, double d_dot, int flag_z_norm, Point3D &result_point);
        int intersect_sphere(Vector3D const& center, Vector3D const& center_dot, double radius, Point3D &result_point_1, Point3D &result_point_2);

    private:
        int nb_points; ///< number of points added

        Point3D *point[2];    ///< points at the extremity of the segment

        Vector3D diff_vec;     ///< vector from first to second point
        Vector3D diff_vec_dot; ///< derivative of 'diff_vec'

        double diff_vec_norm;  ///< norm of 'diff_vec'
        double diff_vec_norm_dot; ///< derivative of 'diff_vec_norm'
};

}
#endif
