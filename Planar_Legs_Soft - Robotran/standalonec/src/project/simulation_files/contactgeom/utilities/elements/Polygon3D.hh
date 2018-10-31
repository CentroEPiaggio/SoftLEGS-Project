/*! 
 * \author Nicolas Van der Noot
 * \file Polygon3D.hh
 * \brief Polygon3D class
 */

#ifndef _POLYGON_3D_HH_
#define _POLYGON_3D_HH_

#include "Segment3D.hh"
#include <vector>

namespace ContactGeom{

/*! \brief polygon in 3D space (convex !)
 */
class Polygon3D
{
    public:
        Polygon3D();
        virtual ~Polygon3D();

        virtual void add_segment(Segment3D *seg, Vector3D const& inside_point);
        virtual void add_point(Point3D *point);

        virtual void update_kinematics(Vector3D const& inside_point);

        Vector3D& get_normal_vec() { return normal_vec; }
        Vector3D& get_normal_vec_dot() { return normal_vec_dot; }

        Point3D* get_first_point();

        double get_d() const { return d; }
        double get_d_dot() const { return d_dot; }

        void dist_point_plane(Vector3D const& point, Vector3D const& point_dot, Vector3D & closest_point, double &dist, double &dist_dot);

    protected:
        std::vector<Segment3D*> segments_list;
        std::vector<Point3D*> points_list;

        Vector3D normal_vec; ///< normal vector, out of the corresponding convex volume, normalized
        Vector3D normal_vec_dot; ///< derivative of 'normal_vec'

        double d; ///< value of 'd' in the equation 'ax + by +cz +d = 0'
        double d_dot; ///< derivative of 'd'

        int flag_z_norm; ///< 1 if norm is (0;0;1), 0 otherwise
};


}
#endif
