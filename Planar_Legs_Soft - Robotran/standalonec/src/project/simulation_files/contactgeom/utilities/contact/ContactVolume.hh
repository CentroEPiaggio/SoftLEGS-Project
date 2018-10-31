/*! 
 * \author Nicolas Van der Noot
 * \file ContactVolume.hh
 * \brief ContactVolume class
 */

#ifndef _CONTACT_VOLUME_HH_
#define _CONTACT_VOLUME_HH_

#include <vector>
#include "Vector3D.hh"

namespace ContactGeom{

class ContactVolume
{
    public:
        ContactVolume();
        ~ContactVolume();

        void reset();
        void copy_reverse(ContactVolume *other_volume);

        // get
        double get_volume()     const { return volume;     }
        double get_volume_dot() const { return volume_dot; }

        Vector3D& get_norm_vec() { return norm_vec; }
        Vector3D& get_center()   { return center;   }

        int get_nb_points() const { return contact_points.size(); }
        Vector3D& get_point(int i) { return contact_points[i]; }
        Vector3D& get_point_vel(int i) { return vel_contact_points[i]; }    

        // set
        void set_volume(double value) { volume = value; }
        void set_volume_dot(double value) { volume_dot = value; }
        void set_norm_vec(Vector3D const& value) { norm_vec = value; }
        void set_center(Vector3D const& value) { center = value; }
        void clear_points();

        void add_point(Vector3D const& point, Vector3D const& rel_vel);

    private:
        // volume
        double volume; ///< volume [m^3]
        double volume_dot; ///< volume time derivative [m^3/s]

        // contact suface (surface bounded by the contact line)
        Vector3D norm_vec; ///< vector normal to the contact surface (in direction of the repulsive normal force)
        Vector3D center;   ///< contact surface center point position

        std::vector<Vector3D> contact_points; ///< points of the contact surface where the tangential forces will be computed

        /*! \brief relative velocities of the contact points [m/s]
         *         (relative speed of the current shape in the frame of the other one)
         */
        std::vector<Vector3D> vel_contact_points;
};

}
#endif
