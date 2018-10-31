/*! 
 * \author Nicolas Van der Noot
 * \file contact_interface.h
 * \brief interface between C++ and C to use the contact library with Robotran
 */

#ifndef _CONTACT_INTERFACE_H_
#define _CONTACT_INTERFACE_H_

#include "MBSdataStruct.h"

#ifdef __cplusplus
extern "C" {
#endif
    void init_contact_geom(MBSdataStruct *MBSdata);
    void user_state_contact_geom(MBSdataStruct *MBSdata);
    void close_contact_geom(MBSdataStruct *MBSdata);
    void update_contact_geom_kinematics(MBSdataStruct *MBSdata);
    void update_contact_geom_F_T(MBSdataStruct *MBSdata);
    void update_contact_geom(MBSdataStruct *MBSdata);
    void apply_contact_geom(MBSdataStruct *MBSdata, int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz);
#ifdef __cplusplus
}
#endif


#endif
