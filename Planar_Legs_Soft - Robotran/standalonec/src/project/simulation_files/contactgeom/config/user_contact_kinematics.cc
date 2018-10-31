
#include "MainUnionShape.hh"
#include "user_contact_kinematics.hh"
#include "RigidShape.hh"

namespace ContactGeom{

/*! \brief kinematics update
 * 
 * \param[out] kin_info_list kinematic information of all moving bodies
 * \param[in] main_union main union of shapes
 * \param[in] MBSdata Robotran structure
 *
 * This function is used to compute the kinematics of all bodies involved in the contact model.
 * Using the line 'main_union->mbs_sensor_compute();' is enough to automatically get the correct result.
 * Unfortunately, this function can be quite slow because it uses the 'sensor' function, which recomputes
 * all the kinematics of each body, each time starting from scratch.
 *
 * For more advance users who want to improve the computational efficiency, you can remove this line and
 * perform yourself the computation. The purpose of this function is to fill 'kin_info_list', as can
 * be seen in the 'update_kin_info' function (see bottom of this file).
 *
 * An easy solution to handle this computation in an efficient way is to do as follows:
 * - Create a new .mbs file (similar to the main one), but where all bodies with a changing kinematics
 *   (i.e. bodies added with the lines 'add_rigid_Ssens' or 'add_rigid_Fsens' in 'user_shapes.cc') are
 *   assigned to a F sensor (even the ones which should be with a S sensor, due to 'add_rigid_Ssens').
 *   Like in 'user_shapes.cc', check only the fields 'Position', 'Rotation Matrix' and 'Velocity' for
 *   the F sensors. Also, remove all the other F sensors in this special .mbs file.
 * - Generate the accelred symbolic equations and open the function 'mbs_accelred'
 * - Copy these two lines: 'q = s->q;' and 'qd = s->qd;'
 * - Copy the sine and cosine equations involved in the requested kinematics computations from 'mbs_accelred'.
 * - Copy the whole block 'Sensor Kinematics' from 'mbs_accelred' (stop just before 'SWr1 = user_ExtForces(...);')
 * - Add the requested declarations found in 'mbs_accelred_xxx.c' and 'mbs_accelred_xxx.h'
 * - For each body where and 'F sensor' was added in the special .mbs file, add this line:
 *         update_kin_info(PxFi, RxFi, VxFi, OMxFi, kin_info_list, j)
 *   where 'i' is the index of the F sensor (i.e. in the order of definition in the special .mbs file, starting with '1')
 *   abd 'j' is the index of the body in the contact model (i.e. the order of calls of 'add_rigid_Ssens'
 *   and 'add_rigid_Fsens' in 'user_shapes.cc', starting with '0'). To be sure, you can print the value
 *   'kin_info_list[j].isens' to know the related index, which would be called in the 'sensor' function.
 */
void user_contact_kinematics(std::vector<KinInfo> &kin_info_list, MainUnionShape *main_union, MBSdataStruct *MBSdata)
{
	// Robotran normal computation
	main_union->mbs_sensor_compute();
}

/*! \brief update 'kin_info_list'
 * 
 * \param[in] PxF position [m]
 * \param[in] RxF rotation matrix [-]
 * \param[in] VxF velocity [m/s]
 * \param[in] OMxF angular velocity [rad/s]
 * \param[out] kin_info_list kinematic information of all moving bodies
 * \param[in] kin_info_id ID of the body in the contact model (i.e. order of the rigid shapes added in 'user_shapes.cc')
 */
void update_kin_info(double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], std::vector<KinInfo> &kin_info_list, int kin_info_id)
{
	for(int i=0; i<3; i++)
	{
		kin_info_list[kin_info_id].P[i]  = PxF[1+i];  ///< position [m]
		kin_info_list[kin_info_id].V[i]  = VxF[1+i];  ///< velocity [m/s]
		kin_info_list[kin_info_id].OM[i] = OMxF[1+i]; ///< angular velocity [rad/s]

		for(int j=0; j<3; j++)
		{
			kin_info_list[kin_info_id].R[i][j] = RxF[1+i][1+j]; ///< rotation matrix [-]
		}
	}
}

}
