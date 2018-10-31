//---------------------------
// Timothee Habra, Houman Dallali
//
// Creation : 9-7-2014
// Last update : 5-8-2015
//---------------------------

#ifndef ACTUATOR_STRUCT_h
#define ACTUATOR_STRUCT_h

// The Big motors are current controlled while medium and small motors are voltage controlled.
#define TYPE_MEDIUM 1  // SEA-MEDIUM: TBMS7615: Torso Roll and Ankle Roll
#define TYPE_SMALL  2  // SEA-SMALL: wrist joints
#define TYPE_BIG    3  // SEA-BIG: Hip Pitch, Hip Roll, Knee, Ankle Pitch, Ankle Yaw, torso Pitch.
#define TYPE_BIG2    4  // SEA-BIG2: Ankle Yaw
#define TYPE_MEDIUM2 5  // SEA-MEDIUM2 RBE1810: Shoulder joints, Elbows, Torso Yaw and Hip Yaw

// Series Elastic Actuator Structure
typedef struct SEActuatorStruct
{
    int    type;
    double Resistance;
    double Inductance;
    double TrqConst;
    double Kbemf;
    double Inertia;
    double Damping;
    double GearRatio;
    double SeriesSpring;
    double SeriesDamping;
    double Isaturation;
    double Vsaturation;

} SEActuatorStruct;

// Parallel Elastic Actuator Structure
typedef struct PActuatorStruct
{
    char   type[50];
    double Resistance;
    double Inductance;
    double TrqConst;
    double Kbemf;
    double Inertia;
    double Damping;
    double BallScrewRatio;
    double PSpring;
    double PDamping;
    double saturation;

} PActuatorStruct;

// Actuators Structure
typedef struct ActuatorsStruct
{
    //int* MotorIds;
	int* JointIds;
    SEActuatorStruct** acs;
    
} ActuatorsStruct;

// ---- Init and free functions: declarations ---- //
ActuatorsStruct* init_ActuatorsStruct(void);
void free_ActuatorsStruct(ActuatorsStruct* actuatorsStruct);

//SEActuatorStruct * init_SEActuatorStruct();
void init_SEActuatorStruct(SEActuatorStruct *act);
//SEActuatorStruct** init_SEActuatorStruct(void);
void free_SEActuatorStruct(SEActuatorStruct *acs[]);

PActuatorStruct * init_PStruct(void);
void free_PActuatorStruct(PActuatorStruct *pacs);

/*--------------------*/
#endif



