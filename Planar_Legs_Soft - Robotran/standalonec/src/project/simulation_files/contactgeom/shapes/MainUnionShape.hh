/*! 
 * \author Nicolas Van der Noot
 * \file MainUnionShape.hh
 * \brief MainUnionShape class
 */

#ifndef _MAIN_UNION_SHAPE_HH_
#define _MAIN_UNION_SHAPE_HH_

#include "UnionShape.hh"
#include "RigidShape.hh"
#include "user_contact_kinematics.hh"
#include <vector>

namespace ContactGeom{

/*! \brief main union of shapes
 */
class MainUnionShape: public UnionShape
{
    public:
        MainUnionShape(MBSdataStruct *MBSdata);
        virtual ~MainUnionShape();

        void add_rigid_F(RigidShape *new_rigid);
        void add_rigid_S(RigidShape *new_rigid);
        void add_kin_info(RigidShape *rigid_shape);

        void apply_F_T(int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);
        void mbs_sensor_compute();
        void kin_info_apply();

        std::vector<KinInfo>& get_kin_info_list() { return kin_info_list; }

    private:
        int nb_joints; ///< number of joints in Robotran

        std::vector<RigidShape*> rigid_F_list; ///< list of all rigid shapes with Fsensor
        std::vector<RigidShape*> rigid_S_list; ///< list of all rigid shapes with Ssensor
        std::vector<KinInfo> kin_info_list; ///< kinematic info computed
};

}
#endif
