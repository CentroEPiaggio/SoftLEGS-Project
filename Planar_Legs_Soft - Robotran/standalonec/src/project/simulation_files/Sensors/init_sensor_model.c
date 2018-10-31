#include "MBSdataStructR7.h"

#include "user_all_id.h"
#include "MBSfun.h"
#include "simu_def.h"

void init_sensor_model(MBSdataStruct *MBSdata)
{

    IMU_Struct* IMU_waist;
    IMU_Struct* IMU_head;
    FT_sensor_Struct* RFoot_FT;
    FT_sensor_Struct* LFoot_FT;

    // waist IMU sensor
    IMU_waist = MBSdata->user_IO->cvs->Inputs->IMU_list[IMU_WAIST_ID];  // this index should be in a macro

    allocate_sensor(IMU_waist->robotran_sensor,NB_JOINTS);
    init_sensor(IMU_waist->robotran_sensor,NB_JOINTS);
    IMU_waist->sensor_id = WaistCg_id; // a sensor for IMU should be add in mbs. For now we can use WaistCg for testing.
    sensor(IMU_waist->robotran_sensor, MBSdata, IMU_waist->sensor_id );

    // head IMU sensor
    IMU_head = MBSdata->user_IO->cvs->Inputs->IMU_list[IMU_HEAD_ID];  // this index should be in a macro

    allocate_sensor(IMU_head->robotran_sensor,NB_JOINTS);
    init_sensor(IMU_head->robotran_sensor,NB_JOINTS);
    IMU_head->sensor_id = DWYTorsoCg_id; // a sensor for IMU should be add in mbs. For now we can use DWYTorsoCg_id for testing.
    sensor(IMU_head->robotran_sensor, MBSdata, IMU_head->sensor_id );

    // right foot Force torque sensor
    RFoot_FT = MBSdata->user_IO->cvs->Inputs->FT_sensor_list[FT_RFOOT_ID];  // this index should be in a macro

    RFoot_FT->id_joint_F[0] = FT_RFoot_X_id;  // Fx
    RFoot_FT->id_joint_F[1] = FT_RFoot_Y_id;  // Fy
    RFoot_FT->id_joint_F[2] = FT_RFoot_Z_id;  // Fz

    RFoot_FT->id_joint_T[0] = FT_RFoot_R1_id;  // R1
    RFoot_FT->id_joint_T[1] = FT_RFoot_R2_id;  // R2
    RFoot_FT->id_joint_T[2] = FT_RFoot_R3_id;  // R3

    // left foot Force torque sensor
    LFoot_FT = MBSdata->user_IO->cvs->Inputs->FT_sensor_list[FT_LFOOT_ID];  // this init of FT should be in a function
    LFoot_FT->id_joint_F[0] = FT_LFoot_X_id;  // Fx
    LFoot_FT->id_joint_F[1] = FT_LFoot_Y_id;  // Fy
    LFoot_FT->id_joint_F[2] = FT_LFoot_Z_id;  // Fz

    LFoot_FT->id_joint_T[0] = FT_LFoot_R1_id;  // R1
    LFoot_FT->id_joint_T[1] = FT_LFoot_R2_id;  // R2
    LFoot_FT->id_joint_T[2] = FT_LFoot_R3_id;  // R3


}

void update_sensor_model(MBSdataStruct *MBSdata)
{
    ControllerInputs* ivs;
    ActuatorsStruct* actuatorsStruct;
    int i;

    ivs = MBSdata->user_IO->cvs->Inputs;
    actuatorsStruct = MBSdata->user_IO->actuatorsStruct;

    // clock
	ivs->t = MBSdata->tsim;

    // Actuators sensors
    for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
        // link position
        ivs->q[i + 1] = MBSdata->q[actuatorsStruct->JointIds[i]];

        // link velocity
        ivs->qd[i + 1] = MBSdata->qd[actuatorsStruct->JointIds[i]];

        // link torque
        ivs->Qq[i + 1] = MBSdata->Qq[actuatorsStruct->JointIds[i]];

    }

    switch (Act_order) {
        case 1:  // justElectrical
            break;

        case 2: // Motor (Mechanical) ODE
            // ux:motor position, velocity
            for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
            {
                ivs->q_mot[i + 1]  = MBSdata->ux[i + 1];
                ivs->qd_mot[i + 1] = MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED];
            }
            break;

        case 3:  // Motor (Electrical+Mechanical) ODE
            // ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative
            for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
            {
                ivs->q_mot[i + 1]  = MBSdata->ux[i + 1];
                ivs->qd_mot[i + 1] = MBSdata->ux[i + 1 + COMAN_NB_JOINT_ACTUATED];
            }
            break;
    }

    // IMU sensors
    update_IMU_data(ivs->IMU_list[IMU_WAIST_ID], MBSdata); // waist IMU sensors
    update_IMU_data(ivs->IMU_list[IMU_HEAD_ID], MBSdata);  // head IMU sensors

    // Force-Torque sensors
    update_FT_data(ivs->FT_sensor_list[FT_RFOOT_ID], MBSdata); // right foot FT
    update_FT_data(ivs->FT_sensor_list[FT_LFOOT_ID], MBSdata); // left foot FT

}
