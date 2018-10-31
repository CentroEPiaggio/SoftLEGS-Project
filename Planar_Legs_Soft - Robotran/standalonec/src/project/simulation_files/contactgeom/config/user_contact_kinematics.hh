/*! 
 * \author Nicolas Van der Noot
 * \file user_contact_kinematics.hh
 * \brief kinematics of the contact bodies
 */

#ifndef _USER_CONTACT_KINEMATICS_HH_
#define _USER_CONTACT_KINEMATICS_HH_

extern "C" {
	#include "MBSsensorStruct.h"
}
#include <vector>
#include "MBSdataStruct.h"

namespace ContactGeom{

// forward declaration
class RigidShape;
class MainUnionShape;

/*! \brief kinematics information
 */ 
typedef struct KinInfo
{
	double P[3]; ///< position [m]
	double V[3]; ///< velocity [m/s]
	double OM[3]; ///< angular velocity [rad/s]
	double R[3][3]; ///< rotation matrix [-]

	int isens; ///< Robotran ID in 'sensor'
	RigidShape *rigid_shape; ///< RigidShape pointer

	MBSsensorStruct sens; ///< Robotran sensor

} KinInfo;

//  function prototype
void user_contact_kinematics(std::vector<KinInfo> &kin_info_list, MainUnionShape *main_union, MBSdataStruct *MBSdata);
void update_kin_info(double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], std::vector<KinInfo> &kin_info_list, int kin_info_id);

}
#endif
