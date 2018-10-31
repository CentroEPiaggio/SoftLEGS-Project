#include "RigidShape.hh"
#include "SphereShape.hh"
#include "CuboidShape.hh"
#include "PlaneShape.hh"
#include <iostream>

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] parent_shape parent shape
 *
 * This constructor is used for rigid bodies only.
 */
RigidShape::RigidShape(MBSdataStruct *MBSdata, GeometryShape *parent_shape): UnionShape(MBSdata, parent_shape)
{
    type = GEOM_RIGID;

    flag  = RIGID_FIXED;
    isens = -1;
    Fsens = -1;

    P.reset();
    V.reset();
    OM.reset();
    R.reset();

    reset_F_T_tot();
}

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] ref_pos reference position
 * \param[in] parent_shape parent shape
 *
 * This constructor is used for rigid bodies only.
 */
RigidShape::RigidShape(MBSdataStruct *MBSdata, Vector3D const& ref_pos, GeometryShape *parent_shape): UnionShape(MBSdata, parent_shape)
{
    type = GEOM_RIGID;

    flag  = RIGID_FIXED;
    isens = -1;
    Fsens = -1;

    P = ref_pos;
    V.reset();
    OM.reset();
    R.reset();

    reset_F_T_tot();
}

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 * \param[in] flag flag of the rigid body 
 * \param[in] id ID of the related sensor
 * \param[in] parent_shape parent shape
 */
RigidShape::RigidShape(MBSdataStruct *MBSdata, int flag, int id, GeometryShape *parent_shape): UnionShape(MBSdata, parent_shape)
{
    type = GEOM_RIGID;

    this->flag = flag;

    // safety check
    if (flag == RIGID_FIXED)
    {
        isens = -1;
        Fsens = -1;

        P.reset();
        V.reset();
        OM.reset();
        R.reset();
    }
    else
    {
        if (id <= 0)
        {
            std::cout << "Error: a moving body should have a strictly positive 'id' value !" << std::endl;
            exit(EXIT_FAILURE);
        }

        switch (flag)
        {
            case RIGID_S_SENS:
                
                if (id > MBSdata->Nsensor)
                {
                    std::cout << "Error: the Ssensor ID (" << id << ") should be smaller than " << MBSdata->Nsensor + 1 << "!" << std::endl;
                    exit(EXIT_FAILURE);
                }

                isens = id;
                Fsens = -1;     
                break;

            case RIGID_F_SENS:
                
                if (id > MBSdata->Nxfrc)
                {
                    std::cout << "Error: the Fsensor ID (" << id << ") should be smaller than " << MBSdata->Nxfrc + 1 << "!" << std::endl;
                    exit(EXIT_FAILURE);
                }

                isens = MBSdata->Nsensor + id;
                Fsens = id;     
                break;
        
            default:
                std::cout << "Error: unknown rigid body flag !" << std::endl;
                exit(EXIT_FAILURE);
                break;
        }

        update_kinematics();
    }

    reset_F_T_tot();
}

/*! \brief destructor
 */
RigidShape::~RigidShape()
{

}

/*! \brief add a sphere
 * 
 * \param[in] radius radius of the sphere [m]
 * \param[in] rel_position relative position vector from the reference position of RigidShape to the shape reference position [m]
 */
BasicShape* RigidShape::add_sphere(double radius, Vector3D const& rel_position)
{
    return static_cast<SphereShape*>(add_geometry(new SphereShape(radius, rel_position, flag, this)));
}

/*! \brief add a sphere
 * 
 * \param[in] radius radius of the sphere [m]
 * \param[in] pos_x relative position vector x component
 * \param[in] pos_y relative position vector y component
 * \param[in] pos_z relative position vector z component
 */
BasicShape* RigidShape::add_sphere(double radius, double pos_x, double pos_y, double pos_z)
{
    return add_sphere(radius, get_vector_3D(pos_x, pos_y, pos_z));
}

/*! \brief add a cuboid
 *
 * \param[in] d cuboid depth [m]
 * \param[in] w cuboid width [m]
 * \param[in] h cuboid height [m]
 * \param[in] rel_position relative position vector from the reference position of RigidShape to the shape reference position [m]
 * \param[in] rel_matrix relative rotation matrix from the RigidShape to the current BasicShape [-]
 */
BasicShape* RigidShape::add_cuboid(double d, double w, double h, Vector3D const& rel_position, RotMatrix const& rel_matrix)
{
    return static_cast<CuboidShape*>(add_geometry(new CuboidShape(d, w, h, rel_position, rel_matrix, flag, this)));
}

/*! \brief add a cuboid
 *
 * \param[in] d cuboid depth [m]
 * \param[in] w cuboid width [m]
 * \param[in] h cuboid height [m]
 * \param[in] pos_x relative position vector x component
 * \param[in] pos_y relative position vector y component
 * \param[in] pos_z relative position vector z component
 * \param[in] theta_x relative angle along the x axis [rad]
 * \param[in] theta_y relative angle along the y axis [rad]
 * \param[in] theta_z relative angle along the z axis [rad]
 */
BasicShape* RigidShape::add_cuboid(double d, double w, double h, double pos_x, double pos_y, double pos_z, double theta_x, double theta_y, double theta_z)
{
    return add_cuboid(d, w, h, get_vector_3D(pos_x, pos_y, pos_z), RotMatrix(theta_x, theta_y, theta_z));
}

/*! \brief add a plane with the equation 'ax + by + cz + d = 0'
 *
 * \param[in] a value 'a' in the equation 'ax + by + cz + d = 0'
 * \param[in] b value 'b' in the equation 'ax + by + cz + d = 0'
 * \param[in] c value 'c' in the equation 'ax + by + cz + d = 0'
 * \param[in] d value 'd' in the equation 'ax + by + cz + d = 0'
 */
BasicShape* RigidShape::add_plane(double a, double b, double c, double d)
{
    return static_cast<PlaneShape*>(add_geometry(new PlaneShape(a, b, c, d, flag, this)));
}

/*! \brief add a plane with normal and relative position
 *
 * \param[in] norm_vec vector normal to the surface
 * \param[in] rel_position relative position vector from the reference position of RigidShape to the shape reference position [m]
 */
BasicShape* RigidShape::add_plane(Vector3D const& normal_vec, Vector3D const& rel_position)
{
    return static_cast<PlaneShape*>(add_geometry(new PlaneShape(normal_vec, rel_position, flag, this)));
}

/*! \brief add a plane whose normal is aligned with the z+ axis
 *
 * \param[in] z_height distance from rigid reference position along the z+ axis
 */
BasicShape* RigidShape::add_plane(double z_height)
{
    return static_cast<PlaneShape*>(add_geometry(new PlaneShape(z_height, flag, this)));
}

/*! \brief add a plane with normal and relative position
 *
 * \param[in] norm_x normal x component
 * \param[in] norm_y normal y component
 * \param[in] norm_z normal z component
 * \param[in] pos_x relative position vector x component
 * \param[in] pos_y relative position vector y component
 * \param[in] pos_z relative position vector z component
 */
BasicShape* RigidShape::add_plane(double norm_x, double norm_y, double norm_z, double pos_x, double pos_y, double pos_z)
{
    return add_plane(get_vector_3D(norm_x, norm_y, norm_z), get_vector_3D(pos_x, pos_y, pos_z));
}

/*! \brief check if all the elements of list_geom are basic shapes
 */
void RigidShape::check_list()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        switch (list_geom[i]->get_type())
        {
            case GEOM_BASIC  : break;
            case GEOM_SPHERE : break;
            case GEOM_CUBOID : break;
            case GEOM_PLANE  : break;
        
            default:
                std::cout << "Error: not a BasicShape !" << std::endl;
                exit(EXIT_FAILURE);
                break;
        }
    }
}

/*! \brief update the kinematics based on the KinInfo structure
 * 
 * \param[in] kin_info KinInfo structure
 */
void RigidShape::update_kinematics(KinInfo const& kin_info)
{
    for(int i=0; i<3; i++)
    {
        P.set_comp( i, kin_info.P[i]);
        V.set_comp( i, kin_info.V[i]);
        OM.set_comp(i, kin_info.OM[i]);

        for(int j=0; j<3; j++)
        {
            R.set_comp(i, j, kin_info.R[i][j]);
        }
    }
}

/*! \brief update the kinematics values of the rigid body
 */
void RigidShape::update_kinematics()
{
    // only for moving shape
    if (isens > 0)
    {
        // update all basic shapes kinematics
        for(unsigned int i=0; i<list_geom.size(); i++)
        {
            static_cast<BasicShape*>(list_geom[i])->update_kinematics(P, V, OM, R);
        }

        // check_flag
        update_check();
    }
}

/*! \brief reset the forces and torques to 0
 */
void RigidShape::reset_F_T()
{
    reset_F_T_tot();

    UnionShape::reset_F_T();
}

/*! \brief gather all the forces and torques of the basic shapes at the ExtForce sensor point
 */
void RigidShape::gather_F_T()
{
    // reset F_tot and T_tot
    reset_F_T_tot();

    // iterate on all the basic shapes
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        static_cast<BasicShape*>(list_geom[i])->update_F_T_rigid(F_tot, T_tot);
    }
}

/*! \brief fill Robotran force-torque variables
 * 
 * \param[out] Fx force in the x direction
 * \param[out] Fy force in the y direction
 * \param[out] Fz force in the z direction
 * \param[out] Mx moment in the x direction
 * \param[out] My moment in the y direction
 * \param[out] Mz moment in the z direction
 */
void RigidShape::apply_F_T(double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz)
{
    // force
    *Fx += F_tot.get_x();
    *Fy += F_tot.get_y();
    *Fz += F_tot.get_z();

    // moment
    *Mx += T_tot.get_x();
    *My += T_tot.get_y();
    *Mz += T_tot.get_z();
}

/*! \brief update the user states
 */
void RigidShape::user_states()
{
    // iterate on all the basic shapes
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        static_cast<BasicShape*>(list_geom[i])->user_states(MBSdata);
    }
}

}
