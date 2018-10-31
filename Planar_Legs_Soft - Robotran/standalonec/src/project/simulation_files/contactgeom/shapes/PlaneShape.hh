/*! 
 * \author Nicolas Van der Noot
 * \file PlaneShape.hh
 * \brief PlaneShape class
 */

#ifndef _PLANE_SHAPE_HH_
#define _PLANE_SHAPE_HH_

#include "BasicShape.hh"

namespace ContactGeom{

/*! \brief basic shape: infinite plane
 */
class PlaneShape: public BasicShape
{
    public:
        PlaneShape(Vector3D const& normal_vec, Vector3D const& rel_position, int rigid_flag, GeometryShape *parent_shape);
        PlaneShape(double a, double b, double c, double d, int rigid_flag, GeometryShape *parent_shape);
        PlaneShape(double z_height, int rigid_flag, GeometryShape *parent_shape);
        virtual ~PlaneShape();

        virtual void update_check();

        virtual void update_kinematics(Vector3D const& parent_P, Vector3D const& parent_V, Vector3D const& parent_OM, RotMatrix const& parent_R);

        void set_absolute_norm(Vector3D const& normal_vec);
        void update_d();

        // update the check_flag flag
        virtual void update_check_flag() { check_flag = 0; }

        virtual int update_F_T_basic(BasicShape *other_shape);
        virtual int point_inside(Vector3D const& point);

        double get_d() const { return d; }
        double get_d_dot() const { return d_dot; }

        int get_flag_z_norm() const { return flag_z_norm; }

        double dist_point(Vector3D const& point);
        void dist_point(Vector3D const& point, Vector3D const& point_dot, Vector3D & closest_point, Vector3D & norm_res, double & dist, double & dist_dot);

        Vector3D& get_norm_vec() { return norm_vec; }
        Vector3D& get_norm_vec_dot() { return norm_vec_dot; }

    private:
        int flag_z_norm; ///< 1 if normal is along the z axis upward (e.g. flat ground)

        RotMatrix R;   ///< rotation matrix
        RotMatrix R_T; ///< rotation matrix transpose

        Vector3D u_local; ///< first vector inside the plane (local frame)
        Vector3D v_local; ///< second vector inside the plane (local frame)

        Vector3D norm_vec_local; ///< normal vector to the plane (normalized) in the local frame
        Vector3D norm_vec; ///< normal vector to the plane (normalized), equal to (a,b,c) in 'ax + by + cz + d = 0'
        Vector3D norm_vec_dot; ///< derivative of 'norm_vec'

        double d; ///< constant d in the equation 'ax + by + cz + d = 0'
        double d_dot; ///< derivative of 'd'

        // functions prototypes
        int update_F_T_sphere(BasicShape *other_shape);
        int update_F_T_cuboid(BasicShape *other_shape);
};

}
#endif
