/*! 
 * \author Nicolas Van der Noot
 * \file gcm_interface.h
 * \brief C-C++ interface for the groud contacr model
 */

#ifndef _GCM_INTERFACE_H_
#define _GCM_INTERFACE_H_

#include "user_sf_IO.h"
#include "MBSdataStruct.h"

#ifdef __cplusplus
extern "C" {
#endif
	void init_mesh_gcm(MBSdataStruct *MBSdata);
	void free_mesh_gcm(MBSdataStruct *MBSdata);

	void gcm_mesh_compute_F_T(MBSdataStruct *MBSdata, double *SWr, double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index);
	void gcm_mesh_state_compute(MBSdataStruct *MBSdata);
#ifdef __cplusplus
}
#endif

#endif
