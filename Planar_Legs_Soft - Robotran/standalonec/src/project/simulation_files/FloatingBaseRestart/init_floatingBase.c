
#include "MBSdataStructR7.h"

#include "user_all_id.h"
#include "MBSfun.h"
#include "simu_def.h"


void init_floatingBase(MBSdataStruct *MBSdata)
{

    int i;

    for ( i=1; i<7; i++ )
    {
     MBSdata->user_IO->FB_state[i]=0.0;
    }

    // only set the pelvis height at a suitable value.
    MBSdata->user_IO->FB_state[3]=1.1;

}
