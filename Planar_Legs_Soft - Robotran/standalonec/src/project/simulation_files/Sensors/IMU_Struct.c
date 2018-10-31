

#include "IMU_Struct.h"
#include "MBSfun.h"
#include "MBSdataStruct.h"
#include <stdlib.h>

IMU_Struct* init_IMU_Struct(void)
{
	int i;

	IMU_Struct* Imu;
	Imu = (IMU_Struct*) malloc(sizeof(IMU_Struct));
    Imu->robotran_sensor = (MBSsensorStruct*) malloc(sizeof(MBSsensorStruct));
    Imu->sensor_id = 0;

	for (i=0; i < 9; i++)
    {
    	Imu->Orientation[i] = 0.;
    }

    for (i=0; i < 3; i++)
    {
    	Imu->Angular_Rate[i] = 0.;
    }

    for (i=0; i < 3; i++)
    {
    	Imu->Acceleration[i] = 0.;
    }

    return Imu;

}

void free_IMU_Struct(IMU_Struct* Imu)
{
    if(Imu->robotran_sensor != NULL)
        free_sensor(Imu->robotran_sensor);

	free(Imu);
}

void update_IMU_data(IMU_Struct* Imu, MBSdataStruct *MBSdata)
{
    int i;

    sensor(Imu->robotran_sensor, MBSdata, Imu->sensor_id );

    // IMU orientation
    Imu->Orientation[0] = Imu->robotran_sensor->R[1][1];
    Imu->Orientation[1] = Imu->robotran_sensor->R[1][2];
    Imu->Orientation[2] = Imu->robotran_sensor->R[1][3];
    Imu->Orientation[3] = Imu->robotran_sensor->R[2][1];
    Imu->Orientation[4] = Imu->robotran_sensor->R[2][2];
    Imu->Orientation[5] = Imu->robotran_sensor->R[2][3];
    Imu->Orientation[6] = Imu->robotran_sensor->R[3][1];
    Imu->Orientation[7] = Imu->robotran_sensor->R[3][2];
    Imu->Orientation[8] = Imu->robotran_sensor->R[3][3];

    // IMU absolute velocity and acceleration
    for (i=0; i<3; i++)
    {
        Imu->Angular_Rate[i] = Imu->robotran_sensor->OM[i+1];  // angulare rate -> velocity [rad/s]
        Imu->Acceleration[i] = Imu->robotran_sensor->OMP[i+1]; // acceleration [rad/s^2]
    }
}
