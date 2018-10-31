/*! 
 * \author Nicolas Van der Noot
 * \file Rectangle3D.hh
 * \brief Rectangle3D class
 */

#ifndef _RECTANGLE_3D_HH_
#define _RECTANGLE_3D_HH_

#include "Polygon3D.hh"

namespace ContactGeom{

/*! \brief rectangle surface in 3D space 
 */
class Rectangle3D: public Polygon3D
{
    public:
        Rectangle3D();
        virtual ~Rectangle3D();

        virtual void add_segment(Segment3D *seg, Vector3D const& inside_point);
        virtual void add_point(Point3D *point);

        virtual void update_kinematics(Vector3D const& inside_point);

        int contact_segment(Segment3D &seg, Point3D &point_res);
        int check_point_inside(Vector3D const& point);

    protected:
        unsigned int nb_seg; ///< number of segments
        int flag_two_vec; ///< 1 if 'two_vec_points' correctly filled, 0 otherwise

        Point3D* two_vec_points[3]; ///< three points definining two consecutives segment vectors

        Vector3D vec_a; ///< first consecutive vector
        Vector3D vec_b; ///< second consecutive vector

        double aa; ///< square norm of vec_a
        double bb; ///< square norm of vec_b
        double ab; ///< scalar product of vec_a with vec_b
        double denom; ///< denominator to check point inclusion
};

}
#endif
