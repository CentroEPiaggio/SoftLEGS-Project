

#include "IMU_Struct.h"
#include "MBSfun.h"
#include "MBSdataStruct.h"
#include <stdlib.h>


FT_sensor_Struct* init_FT_sensor_Struct(void)
{
	int i;

	FT_sensor_Struct* FT_sens;
	FT_sens = (FT_sensor_Struct*) malloc(sizeof(FT_sensor_Struct));

    for (i=0; i < 3; i++)
    {
        FT_sens->id_joint_F[i] = 0;
        FT_sens->id_joint_T[i] = 0;
        FT_sens->force[i] = 0.;
        FT_sens->torque[i] = 0.;
    }

    return FT_sens;

}

void free_FT_sensor_Struct(FT_sensor_Struct* FT_sens)
{
	free(FT_sens);
}

void update_FT_data(FT_sensor_Struct* FT_sens, MBSdataStruct *MBSdata)
{
    int i;

    for(i=0;i<3;i++)
    {
        FT_sens->force[i] = MBSdata->Qq[FT_sens->id_joint_F[i]];
        FT_sens->torque[i] = MBSdata->Qq[FT_sens->id_joint_T[i]];
    }
}
