#include "MainUnionShape.hh"

extern "C" {
    #include "MBSfun.h"
}

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 */
MainUnionShape::MainUnionShape(MBSdataStruct *MBSdata): UnionShape(MBSdata, NULL)
{
    nb_joints = MBSdata->njoint;
}

/*! \brief destructor
 */
MainUnionShape::~MainUnionShape()
{
    for(unsigned int i=0; i<kin_info_list.size(); i++)
    {
        free_sensor(&(kin_info_list[i].sens));
    }
}

/*! \brief add a new Rigid Shape with S sensor
 * 
 * \param[in] new_rigid new rigid shape
 */
void MainUnionShape::add_rigid_S(RigidShape *new_rigid)
{
    rigid_S_list.push_back(new_rigid);
    add_kin_info(new_rigid);
}

/*! \brief add a new Rigid Shape with F sensor
 * 
 * \param[in] new_rigid new rigid shape
 */
void MainUnionShape::add_rigid_F(RigidShape *new_rigid)
{
    rigid_F_list.push_back(new_rigid);
    add_kin_info(new_rigid);
}

/*! \brief add a new ID used to get sensors kinematics
 * 
 * \param[in] rigid_shape rigid shape added
 */
void MainUnionShape::add_kin_info(RigidShape *rigid_shape)
{
    // increment list
    kin_info_list.push_back(KinInfo());

    // initialize with 0 values
    for(int i=0; i<3; i++)
    {
        kin_info_list.back().P[i]  = 0.0;
        kin_info_list.back().V[i]  = 0.0;
        kin_info_list.back().OM[i] = 0.0;

        for(int j=0; j<3; j++)
        {
            kin_info_list.back().R[i][j] = 0.0;
        }
    }

    // isens related
    kin_info_list.back().isens = rigid_shape->get_isens();
    kin_info_list.back().rigid_shape = rigid_shape;

    // Robotran sensor
    allocate_sensor(&(kin_info_list.back().sens), nb_joints);
    init_sensor(&(kin_info_list.back().sens), nb_joints);
}

/*! \brief apply the computed force on the Fsensor if its id is in the list of rigid F shapes
 * 
 * \param[in] ixF ID of the Fsensor
 * \param[out] Fx force in the x direction
 * \param[out] Fy force in the y direction
 * \param[out] Fz force in the z direction
 * \param[out] Mx moment in the x direction
 * \param[out] My moment in the y direction
 * \param[out] Mz moment in the z direction
 */
void MainUnionShape::apply_F_T(int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz)
{
    RigidShape *cur_rigid;

    for(unsigned int i=0; i<rigid_F_list.size(); i++)
    {
        cur_rigid = rigid_F_list[i];

        if ( (cur_rigid->get_flag() == RIGID_F_SENS) && (cur_rigid->get_Fsens() == ixF) )
        {
            cur_rigid->apply_F_T(Fx, Fy, Fz, Mx, My, Mz);

            break;
        }
    }
}

/*! \brief fill the kinematics info with the Robotran 'sensor' function
 */
void MainUnionShape::mbs_sensor_compute()
{
    int cur_isens;

    for(unsigned int i=0; i<kin_info_list.size(); i++)
    {
        cur_isens = kin_info_list[i].isens;

        if (cur_isens <= 0)
        {
            std::cout << "Error, S sens ID " << cur_isens << " should be strictly positive !" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Robotran kinematics computation
        init_sensor(&(kin_info_list[i].sens), nb_joints);

        sensor(&(kin_info_list[i].sens), MBSdata, cur_isens);

        // copy values computed from Robotran
        for(int j=0; j<3; j++)
        {
            kin_info_list[i].P[j]  = kin_info_list[i].sens.P[1+j];
            kin_info_list[i].V[j]  = kin_info_list[i].sens.V[1+j];
            kin_info_list[i].OM[j] = kin_info_list[i].sens.OM[1+j];

            for(int k=0; k<3; k++)
            {
                kin_info_list[i].R[j][k] = kin_info_list[i].sens.R[1+j][1+k];
            }
        }
    }
}

/*! \brief apply the kinematics info computed in kin_info_list
 */
void MainUnionShape::kin_info_apply()
{
    for(unsigned int i=0; i<kin_info_list.size(); i++)
    {
        kin_info_list[i].rigid_shape->update_kinematics(kin_info_list[i]);
    }
}

}
