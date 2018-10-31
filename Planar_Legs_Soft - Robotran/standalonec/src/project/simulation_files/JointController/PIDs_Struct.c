
#include "PIDs_Struct.h"

void free_ControllerPIDs(ControllerPIDs *PIDs)
{
    free(PIDs);
}

ControllerPIDs * init_ControllerPIDs(void)
{
     ControllerPIDs *PIDs;
	 int i;

     PIDs = (ControllerPIDs*) malloc(sizeof(ControllerPIDs));

     for (i=0;i<COMAN_NB_JOINT_ACTUATED+1;i++)
     {
         PIDs->p[i] = 0.0;
         PIDs->d[i] = 0.0;
         PIDs->i[i] = 0.0;
         PIDs->maxInt[i] = 0.0;
         PIDs->maxOut[i]= 0.0;
         PIDs->int_err[i]= 0.0;
     }

     return PIDs;
}
