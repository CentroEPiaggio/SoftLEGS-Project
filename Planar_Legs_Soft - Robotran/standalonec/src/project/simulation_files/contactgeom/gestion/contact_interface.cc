#ifndef STANDALONE
#include <cmath>
#ifdef _CHAR16T // math.h version
#define CHAR16_T // visual version
#endif
#endif

#include "contact_interface.h"
#include "ContactGestion.hh"
#include "user_sf_IO.h"

/*! \brief extract ContactGestion from MBSdata
 * 
 * \param[in] MBSdata Robotran structure
 * \return ContactGestion
 */
ContactGeom::ContactGestion* get_ContactGestion(MBSdataStruct *MBSdata)
{
    return static_cast<ContactGeom::ContactGestion*>(MBSdata->user_IO->contactGestion);
}

/*! \brief initialize ContactGestion
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void init_contact_geom(MBSdataStruct *MBSdata)
{
    MBSdata->user_IO->contactGestion = new ContactGeom::ContactGestion(MBSdata);
}

void user_state_contact_geom(MBSdataStruct *MBSdata)
{
    get_ContactGestion(MBSdata)->update_user_state();
}

/*! \brief delete ContactGestion
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void close_contact_geom(MBSdataStruct *MBSdata)
{
    delete get_ContactGestion(MBSdata);
}

/*! \brief update the kinematics of the shapes in ContactGestion
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void update_contact_geom_kinematics(MBSdataStruct *MBSdata)
{
    get_ContactGestion(MBSdata)->update_kinematics();
}

/*! \brief update the forces with ContactGestion
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void update_contact_geom_F_T(MBSdataStruct *MBSdata)
{
    get_ContactGestion(MBSdata)->update_F_T();
}

/*! \brief update the kinematics and force-torque computation
 * 
 * \param[in,out] MBSdata Robotran structure
 */
void update_contact_geom(MBSdataStruct *MBSdata)
{
    update_contact_geom_kinematics(MBSdata);
    update_contact_geom_F_T(MBSdata);
}

/*! \brief apply the forces and torques computed by the force-contact model
 * 
 * \param[in,out] MBSdata Robotran structure
 * \param[in] ixF ID of the Fsensor
 * \param[out] Fx force in the x direction
 * \param[out] Fy force in the y direction
 * \param[out] Fz force in the z direction
 * \param[out] Mx moment in the x direction
 * \param[out] My moment in the y direction
 * \param[out] Mz moment in the z direction
 */
void apply_contact_geom(MBSdataStruct *MBSdata, int ixF, double *Fx, double *Fy, double *Fz, double *Mx, double *My, double *Mz)
{
    get_ContactGestion(MBSdata)->get_main_union()->apply_F_T(ixF, Fx, Fy, Fz, Mx, My, Mz);
}
