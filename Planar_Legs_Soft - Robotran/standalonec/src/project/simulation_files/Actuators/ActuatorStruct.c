//---------------------------
//
// Timothee Habra, Houman Dallali
//
// Creation : 09.07.2014
// Last update : 09.07.2014
//---------------------------

#include <stdlib.h>
#include <stdio.h>

#include "ActuatorStruct.h"
#include "ActuatorsDefinitions.h"
//#include "user_all_id.h"

// ---- Actuator initialization ---- //

ActuatorsStruct* init_ActuatorsStruct(void)
{
    int i;

    ActuatorsStruct* actuatorsStruct;
    actuatorsStruct = (ActuatorsStruct*) malloc(sizeof(ActuatorsStruct));

    actuatorsStruct->acs = (SEActuatorStruct**)malloc(COMAN_NB_JOINT_ACTUATED*sizeof(SEActuatorStruct*));

    for (i=0; i < COMAN_NB_JOINT_ACTUATED; i++)
    {
        actuatorsStruct->acs[i] = (SEActuatorStruct*) malloc(sizeof(SEActuatorStruct));
    }

    actuatorsStruct->JointIds = (int*)malloc(COMAN_NB_JOINT_ACTUATED*sizeof(int));

    //actuatorsStruct->MotorIds = (int*)malloc((NB_JOINTS+1)*sizeof(int));

    return actuatorsStruct;
}

void free_ActuatorsStruct(ActuatorsStruct* actuatorsStruct)
{

    free_SEActuatorStruct(actuatorsStruct->acs);
	free(actuatorsStruct->JointIds);
    //free(actuatorsStruct->MotorIds);

    free(actuatorsStruct);

}

// SEActuatorStruct
void init_SEActuatorStruct(SEActuatorStruct *act)
//SEActuatorStruct** init_SEActuatorStruct(void)
{
        switch (act->type) {
          case TYPE_MEDIUM: //TBMS 7615 (Values reflected after gearbox)
            (*act).Resistance=0.356;
            (*act).GearRatio=100.0;
            (*act).Damping= 385.1575; // 6.3503e-05*N^2+(Kbemf*Ktor)/R*(N^2);
            (*act).Inductance= 0.00032;// to be checked;
            (*act).Inertia=  0.416;
            (*act).Kbemf= 0.117;
            (*act).SeriesDamping = 1;
            (*act).SeriesSpring = 5000;
            (*act).TrqConst=0.117;
            (*act).Isaturation=50;
            (*act).Vsaturation=40;
            //sprintf((*acs[i]).type,"Medium");
            //printf("actuator type is 1 \n");
            break;
        case TYPE_MEDIUM2: //RBE1810 (Values reflected after gearbox)
          (*act).Resistance=1.22;
          (*act).GearRatio=80.0;
          (*act).Damping= 38.3489; // 0+(Kbemf*Ktor)/R*(N^2);
          (*act).Inductance= 0.00032;// to be checked;
          (*act).Inertia=  0.30208;
          (*act).Kbemf= 0.0855;
          (*act).SeriesDamping = 1;
          (*act).SeriesSpring = 1300;
          (*act).TrqConst=0.0855;
          (*act).Isaturation=50;
          (*act).Vsaturation=40;
          //sprintf((*acs[i]).type,"Medium");
          //printf("actuator type is 1 \n");
          break;
          case TYPE_SMALL:
            (*act).Resistance = 0.664;
            (*act).GearRatio=100.0;
            (*act).Damping = 25.52;
            (*act).Inductance = 0.00032;
            (*act).Inertia = 0.1387;
            (*act).Kbemf = 0.041;
            (*act).SeriesDamping = 1;
            (*act).SeriesSpring=500.0;
            (*act).TrqConst = 0.041;
            (*act).Isaturation=10;
            (*act).Vsaturation=24;
            //sprintf((*acs[i]).type,"Small");
            //printf("actuator type is 2 \n");
            break;
        case TYPE_BIG2:
          (*act).Resistance = 0.664; //This parameter is not needed (This motor is current controlled)
          (*act).GearRatio=80.0;
          (*act).Damping =  19.9542; //to be updated
          (*act).Inductance = 0.00032;
          (*act).Inertia =   0.51904;
          (*act).Kbemf = 0.0455;
          (*act).SeriesDamping = 1.0;
          (*act).SeriesSpring = 2300.0;
          (*act).TrqConst = 0.0455;
          (*act).Isaturation=50;
          (*act).Vsaturation=80;
            break;
          case TYPE_BIG:
            (*act).Resistance = 0.664; // This parameter is not needed (This motor is current controlled)
            (*act).GearRatio=100.0;
            (*act).Damping =  19.9542; //to be updated
            (*act).Inductance = 0.00032;
            (*act).Inertia =   0.8110;
            (*act).Kbemf = 0.0455;
            (*act).SeriesDamping = 1.0;
            (*act).SeriesSpring = 10000.0;
            (*act).TrqConst = 0.0455;
            (*act).Isaturation=50;
            (*act).Vsaturation=80;
            //sprintf((*acs[i]).type,"Big");
            //printf("actuator type is 3 \n");
          }
    //return acs;
}


// ParallelActuatorStruct
PActuatorStruct * init_PStruct(void)
{
    PActuatorStruct *pacs;

    pacs = (PActuatorStruct*) malloc(sizeof(PActuatorStruct));
        pacs->Resistance=0.5;
        pacs->BallScrewRatio = 29;
        pacs->Damping=1;
        pacs->Inductance=0.002;
        pacs->Inertia= 0.01;
        pacs->Kbemf= 0.04;
        pacs->PDamping=0.1;
        pacs->PSpring=2000.0;
        pacs->TrqConst=0.04;
        pacs->saturation=24;
        printf("default parallel actuator selected \n");

    return pacs;
}

// ActuatorStruct
void free_SEActuatorStruct(SEActuatorStruct *acs[])
{

    int i;

    for (i=0; i < COMAN_NB_JOINT_ACTUATED; i++)
    {
        free(acs[i]);
    }

}

// ActuatorStruct
void free_PActuatorStruct(PActuatorStruct *pacs)
{
    free(pacs);
}
