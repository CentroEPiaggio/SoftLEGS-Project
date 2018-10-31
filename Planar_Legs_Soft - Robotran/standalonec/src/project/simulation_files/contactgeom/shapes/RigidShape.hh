/*! 
 * \author Nicolas Van der Noot
 * \file RigidShape.hh
 * \brief RigidShape class
 */

#ifndef _RIGID_SHAPE_HH_
#define _RIGID_SHAPE_HH_

#include "GeometryShape.hh"
#include "BasicShape.hh"
#include "UnionShape.hh"
#include "user_contact_kinematics.hh"

namespace ContactGeom{

// types of rigid shapes
enum{RIGID_FIXED, RIGID_S_SENS, RIGID_F_SENS};

/*! \brief rigid shape
 */
class RigidShape: public UnionShape
{
    public:
        RigidShape(MBSdataStruct *MBSdata, GeometryShape *parent_shape);
        RigidShape(MBSdataStruct *MBSdata, Vector3D const& ref_pos, GeometryShape *parent_shape);
        RigidShape(MBSdataStruct *MBSdata, int flag, int id, GeometryShape *parent_shape);
        virtual ~RigidShape();

        virtual void update_kinematics();
        virtual void check_list();
        virtual void reset_F_T();
        virtual void gather_F_T();
        virtual void user_states();

        void update_kinematics(KinInfo const& kin_info);

        BasicShape* add_sphere(double radius, Vector3D const& rel_position);
        BasicShape* add_sphere(double radius, double pos_x, double pos_y, double pos_z);

        BasicShape* add_cuboid(double d, double w, double h, Vector3D const& rel_position, RotMatrix const& rel_matrix);
        BasicShape* add_cuboid(double d, double w, double h, double pos_x, double pos_y, double pos_z, double theta_x, double theta_y, double theta_z);

        BasicShape* add_plane(double a, double b, double c, double d);
        BasicShape* add_plane(Vector3D const& normal_vec, Vector3D const& rel_position);
        BasicShape* add_plane(double norm_x, double norm_y, double norm_z, double pos_x, double pos_y, double pos_z);
        BasicShape* add_plane(double z_height);

        /// reset F_tot and T_tot
        void reset_F_T_tot()
        {
            F_tot.reset();
            T_tot.reset();
        }

        int get_flag()  const { return flag;  }
        int get_Fsens() const { return Fsens; }
        int get_isens() const { return isens; }

        void apply_F_T(double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);


    private:
        /*! \brief flag of the rigid body (RIGID_FIXED, RIGID_S_SENS, RIGID_F_SENS)
         * 
         * RIGID_FIXED:  not moving, can apply force-torque, but no force-torque is applied on it
         * RIGID_S_SENS: moving, can apply force-torque, but no force-torque is applied on it
         * RIGID_F_SENS: moving, can apply force-torque, the opposite force-torque is applied on it
         */
        int flag;

        /*! \brief ID of the sensor in 'sensor'
         * 
         * In case, this is a Ssensor, it is its normal ID.
         * In case, this is a Fsensor, its is the number of Ssensors (MBSdata->Nsensor) + normal ID of the ExtForce sensor
         * In case it is not moving, 'isens' should be set to a negative value.
         */
        int isens;

        /*! \brief ID of the ExtForce sensor
         * 
         * In case the flag is not RIGID_F_SENS, this is set to a negative value.
         */
        int Fsens;

        Vector3D P;  ///< absolute position vector [m]
        Vector3D V;  ///< absolute velocity vector [m/s]
        Vector3D OM; ///< absolute angle position derivative [rad/s]
        RotMatrix R; ///< absolute rotation matrix [-]

        Vector3D F_tot; ///< total force applied at the ExtForce sensor [N]
        Vector3D T_tot; ///< total torque applied at the ExtForce sensor [Nm]
};

}
#endif
