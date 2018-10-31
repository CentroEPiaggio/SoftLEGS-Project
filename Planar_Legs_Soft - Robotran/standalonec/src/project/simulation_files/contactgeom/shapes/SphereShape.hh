/*! 
 * \author Nicolas Van der Noot
 * \file SphereShape.hh
 * \brief SphereShape class
 */

#ifndef _SPHERE_SHAPE_HH_
#define _SPHERE_SHAPE_HH_

#include "BasicShape.hh"

namespace ContactGeom{

/*! \brief basic shape: sphere
 */
class SphereShape: public BasicShape
{
    public:
        SphereShape(double radius, Vector3D const& rel_position, int rigid_flag, GeometryShape *parent_shape);
        virtual ~SphereShape();

        virtual void update_check();

        // update the check_flag flag
        virtual void update_check_flag() { check_flag = 1; }

        virtual int update_F_T_basic(BasicShape *other_shape);
        virtual int point_inside(Vector3D const& point);

        double get_radius() const { return radius; }
        double get_square_radius() const { return square_radius; }

    private:
        double radius; ///< radius of the sphere [m]
        double square_radius; ///< square of the radius [m^2]

        // functions prototypes
        int update_F_T_plane(BasicShape *other_shape);
        int update_F_T_sphere(BasicShape *other_shape);
        int update_F_T_cuboid(BasicShape *other_shape);
};

}
#endif
