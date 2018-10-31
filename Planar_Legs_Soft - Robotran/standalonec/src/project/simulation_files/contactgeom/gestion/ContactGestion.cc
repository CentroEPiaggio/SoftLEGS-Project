
#include "user_shapes.hh"
#include "ContactGestion.hh"

namespace ContactGeom{

/*! \brief constructor
 * 
 * \param[in] MBSdata Robotran structure
 */
ContactGestion::ContactGestion(MBSdataStruct *MBSdata)
{
    this->MBSdata = MBSdata;

    // create global list of shapes (main_union)
    main_union = new MainUnionShape(MBSdata);

    // user main_union configuration
    config_shapes(main_union, MBSdata);

    // update the check_flag flags
    main_union->update_check_flag();

    // check for correct lists
    main_union->check_list();
}

/*! \brief destructor
 */
ContactGestion::~ContactGestion()
{
    delete main_union;
}

/*! \brief update the bodies kinematics
 */
void ContactGestion::update_kinematics()
{
    user_contact_kinematics(main_union->get_kin_info_list(), main_union, MBSdata);

    main_union->kin_info_apply();

    main_union->update_kinematics();
}

/*! \brief update the user possible states
 */
void ContactGestion::update_user_state()
{
    main_union->user_states();
}

/*! \brief update the forces and torques of all bodies
 */
void ContactGestion::update_F_T()
{
    main_union->reset_F_T();
    main_union->update_F_T_main();
    main_union->gather_F_T();
}

}
