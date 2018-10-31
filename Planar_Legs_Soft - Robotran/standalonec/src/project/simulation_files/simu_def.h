//---------------------------
// Nicolas Van der Noot
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Simulation files main header
//
//---------------------------

#ifndef simulation_def_h
#define simulation_def_h
//--------------------*/

#include "MBSdataStruct.h"
// struct MBSdataStruct;
struct IMU_Struct;
struct FT_sensor_Struct;

#include "ActuatorsDefinitions.h"
#include "user_all_id.h"

#ifdef SIMBODY
#include "simbody_cpp_functions.h"
#include "simbody_functions.h"
#endif

#include "user_all_id.h"

#define RFOOT_FSENS_ID RFoot_force_id
#define LFOOT_FSENS_ID LFoot_force_id

// Control Type
#define POSITION_CTRL 0  // uvs->servo_type is inti to 0 
#define TORQUE_CTRL 1
//#define POSITION_DIRECT_CTRL 2
#define IDLE_CTRL 3
//#define VELOCITY_CTRL 4
#define IMPEDANCE_POS_CTRL 5
#define IMPEDANCE_VEL_CTRL 6
#define OPEN_LOOP_CTRL 7

// controller calls
#define PERIOD_CTRL 1.0e-3
#define TIME_EPSILON 1.0e-5

// FT list id
#define FT_RFOOT_ID     0
#define FT_LFOOT_ID     1

// IMU list id
#define IMU_WAIST_ID    0
#define IMU_HEAD_ID     1

// ---- Custom Functions ---- //

// useful functions
double limit_function(double value, double this_min, double this_max);

// actuators
void init_actuator_model(MBSdataStruct *MBSdata);

// joint controller
void init_joints_control(MBSdataStruct *MBSdata);
void joints_control(MBSdataStruct *MBSdata);

// sensor (IMU, ...)
void init_sensor_model(MBSdataStruct *MBSdata);
void update_sensor_model(MBSdataStruct *MBSdata);
void update_IMU_data(IMU_Struct* Imu, MBSdataStruct *MBSdata);
void update_FT_data(FT_sensor_Struct* FT_sens, MBSdataStruct *MBSdata);

// GCM (Ground Contact Model)
double get_ground_height(double x, double y, double tsim, MBSdataStruct *MBSdata);
void ground_mesh_model(double PxF[4], double RxF[4][4],
                       double VxF[4], double OMxF[4],
                       MBSdataStruct *MBSdata, double tsim,
                       int ixF, double *dxF, double *SWr);
void init_GCM(MBSdataStruct *MBSdata);
double z_left_foot(double x, double y);
double z_right_foot(double x, double y);

// floating base reset
void init_floatingBase(MBSdataStruct *MBSdata);

/*--------------------*/
#endif
