#include "ContactVolume.hh"

namespace ContactGeom{

/*! \brief constructor
 */
ContactVolume::ContactVolume()
{
    reset();
}

/*! \brief destructor
 */
ContactVolume::~ContactVolume()
{

}

/*! \brief reset
 */
void ContactVolume::reset()
{
    volume = 0.0;
    volume_dot = 0.0;

    norm_vec.reset();
    center.reset();

    clear_points();
}

/*! \brief copy the volume, but reversing the force result
 * 
 * \param[out] other_volume volume to modify
 */
void ContactVolume::copy_reverse(ContactVolume *other_volume)
{
    // volume
    other_volume->set_volume(volume);
    other_volume->set_volume_dot(volume_dot);

    // contact surface
    other_volume->set_norm_vec(-norm_vec);
    other_volume->set_center(center);

    other_volume->clear_points();

    for(unsigned int i=0; i<contact_points.size(); i++)
    {
        other_volume->add_point(contact_points[i], -vel_contact_points[i]);
    }
}

/*! \brief clear the contact points list
 */
void ContactVolume::clear_points()
{
    contact_points.clear();
    vel_contact_points.clear();
}

/*! \brief add a new point to the tangential contact points list
 * 
 * \param[in] point new contact point position [m]
 * \param[in] rel_vel relative velocity of the contact point
 */
void ContactVolume::add_point(Vector3D const& point, Vector3D const& rel_vel)
{
    contact_points.push_back(point);
    vel_contact_points.push_back(rel_vel);
}

}
