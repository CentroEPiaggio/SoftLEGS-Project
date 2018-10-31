/*! 
 * \author Nicolas Van der Noot
 * \file BasicShape.hh
 * \brief BasicShape class
 */

#ifndef _BASIC_SHAPE_HH_
#define _BASIC_SHAPE_HH_

#include "GeometryShape.hh"
#include "ContactVolume.hh"
#include "MBSdataStruct.h"

// forward declaration
class RigidShape;

namespace ContactGeom{

class BasicShape: public GeometryShape
{
    public:
        BasicShape(Vector3D const& rel_position, int rigid_flag, GeometryShape *parent_shape);
        BasicShape(Vector3D const& rel_position, int check_flag, int rigid_flag, GeometryShape *parent_shape);
        BasicShape(Vector3D const& rel_position, int check_flag, double check_radius, Vector3D const& check_center, int rigid_flag, GeometryShape *parent_shape);
        virtual ~BasicShape();

        virtual void update_kinematics() { /* global update done in RigidShape */ }
        virtual void update_kinematics(Vector3D const& parent_P, Vector3D const& parent_V, Vector3D const& parent_OM, RotMatrix const& parent_R);

        virtual void update_F_T(GeometryShape *other_shape);
        virtual int update_F_T_basic(BasicShape *other_shape) = 0;
        void update_F_T_local(Vector3D const& cur_F, Vector3D const& cur_T, Vector3D const& cur_pos);
        void update_F_T_rigid(Vector3D & F_tot, Vector3D & T_tot);
        void F_T_contact(BasicShape *other_basic);

        Vector3D get_P()  const { return P; }
        Vector3D get_V()  const { return V; }
        Vector3D get_OM() const { return OM; }
        RotMatrix get_parent_R() const { return parent_R; }

        int get_rigid_flag() const { return rigid_flag; }

        virtual void check_list() { /* always correct */ }
        virtual void gather_F_T() { /* nothing to do */  }
        virtual void user_states() { /* nothing to do */  }
        virtual void add_child_recursively(GeometryShape *child) { /* nothing to do */ }

        void user_states(MBSdataStruct *MBSdata);

        virtual void reset_F_T()
        {
            F.reset();
            T.reset();
        }

        void contact_reset(BasicShape *other_shape);

        void set_rel_position(Vector3D const & new_position) { rel_position = new_position; }

        void contact_simple_reset() { contact.reset(); }

        ContactVolume* get_contact() { return &contact; }

        Vector3D get_point_speed(Vector3D const& point_abs);
        void add_tg_point_speed(BasicShape *other_shape, Vector3D const& point_abs);

        virtual int point_inside(Vector3D const& point) = 0;
        

    protected:
        int rigid_flag; ///< flag of the parrent rigid body (RIGID_FIXED, RIGID_S_SENS, RIGID_F_SENS)

        /*! \brief relative position vector from the ExtForce of RigidShape to the shape reference position [m]
         *
         * This position vector is expressed in the frame local to the body
         * (i.e. the one given by the ExtForce sensor of the corresponding RigidShape).
         */
        Vector3D rel_position;

        Vector3D P;  ///< absolute position vector [m]
        Vector3D V;  ///< absolute velocity vector [m/s]
        Vector3D OM; ///< absolute angle position derivative [rad/s]
        RotMatrix parent_R; ///< absolute rotation matrix of the parent RigidShape [-]

        /*! relative position from the ExtForce of RigidShape to the position reference
         *  of BasicShape, in the inertial frame
         */
        Vector3D rel_pos_frame;

        Vector3D F; ///< force applied at the position reference of the basic shape [N]
        Vector3D T; ///< torque applied at the position reference of the basic shape [Nm]

        ContactVolume contact; ///< contact volume result
};

}

#endif
