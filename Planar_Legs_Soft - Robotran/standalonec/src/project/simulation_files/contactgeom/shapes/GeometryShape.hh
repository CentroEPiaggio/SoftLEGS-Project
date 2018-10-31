/*! 
 * \author Nicolas Van der Noot
 * \file GeometryShape.hh
 * \brief GeometryShape class
 */

#ifndef _GEOMETRY_SHAPE_HH_
#define _GEOMETRY_SHAPE_HH_

#include "Vector3D.hh"
#include "RotMatrix.hh"
#include "UserShapeModule.hh"
#include <vector>
#include <iostream>
//#include <stdlib.h>

namespace ContactGeom{

enum{GEOM_MOTHER, GEOM_UNION, GEOM_RIGID, GEOM_BASIC, GEOM_SPHERE, GEOM_CUBOID, GEOM_PLANE};

/*! \brief mother class of all shapes
 */
class GeometryShape
{
    public:
        GeometryShape(GeometryShape *parent_shape);
        GeometryShape(int check_flag, GeometryShape *parent_shape);
        GeometryShape(int check_flag, double check_radius, Vector3D const& check_center, GeometryShape *parent_shape);
        virtual ~GeometryShape();

        virtual void update_check_flag() = 0;
        virtual void update_kinematics() = 0;
        virtual void update_check() = 0;
        virtual void check_list() = 0;
        virtual void reset_F_T() = 0;
        virtual void update_F_T(GeometryShape *other_shape) = 0;
        virtual void gather_F_T() = 0;
        virtual void user_states() = 0;
        virtual void add_child_recursively(GeometryShape *child) = 0;

        virtual void update_check_min_max();
        virtual void list_genealogy();

        int possible_min_max_contact(GeometryShape *other_shape);
        int possible_sphere_contact(GeometryShape *other_shape);
        int contact_possible(GeometryShape *other_shape);

        int get_type() const { return type; }
        int get_check_flag() const { return check_flag; }
        double get_check_radius() const { return check_radius; }
        Vector3D get_check_center() const { return check_center; }

        void set_check_center(Vector3D const& new_pos) { check_center = new_pos; }

        double get_check_min_max(int i, int j) const { return check_min_max[i][j]; }

        void wrapping_spheres(double radius_1, Vector3D center_1, double radius_2, Vector3D center_2, double & new_radius, Vector3D & new_center);

        virtual void set_all_contact_default() { all_contact_flag = 1; }
        virtual void set_no_contact_default()  { all_contact_flag = 0; }

        void add_prohibited(GeometryShape *other_shape);
        void add_no_prohibited(GeometryShape *other_shape);

        void add_prohibited_simple(GeometryShape* other_shape)    {    prohibited_list.push_back(other_shape); }
        void add_no_prohibited_simple(GeometryShape* other_shape) { no_prohibited_list.push_back(other_shape); }

        int get_prohibited_list_size()    const { return prohibited_list.size(); }
        int get_no_prohibited_list_size() const { return no_prohibited_list.size(); }

        UserShapeModule& get_module() { return module; }

        int get_size_genealogy() const { return genealogy.size(); }
        std::vector<GeometryShape*>& get_genealogy() { return genealogy; }
        GeometryShape* get_genealogy_elem(int i) { return genealogy[i]; }

    protected:
        int type; ///< flag indicating the type of GeometryShape

        // genealogy
        std::vector<GeometryShape*> genealogy; ///< all descendant (+ themself)

        // prohibited list
        int all_contact_flag; ///< 1: contact possible with all shapes, except the ones in 'prohibited_list', 0: only contact with 'no_prohibited_list'
        std::vector<GeometryShape*> prohibited_list;    ///< list of prohibited shapes: no contact with them
        std::vector<GeometryShape*> no_prohibited_list; ///< list of no prohibited shapes: contact with them

        // check possible contact
        int check_flag;        ///< 1 if check sphere applicable, 0 otherwise
        double check_radius;   ///< radius of the check sphere (if applicable) [m]
        Vector3D check_center; ///< position of the center of the check sphere (if applicable) [m]
        double check_min_max[3][2]; ///< minimal and maximal x,y,z values of the check sphere (if applicable) [m]

        UserShapeModule module; ///< module used to let the user add properties to the shape

        GeometryShape *parent_shape; ///< parent of this shape
};

}
#endif
