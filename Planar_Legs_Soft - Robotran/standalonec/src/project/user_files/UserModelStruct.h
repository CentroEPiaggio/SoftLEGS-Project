//---------------------------
// C-code automatically generated from Gen_mds_user project
//
//
// Last update : Sat Nov  7 15:01:04 2015
//---------------------------



#ifndef USERMODELSTRUCT_h
#define USERMODELSTRUCT_h

#include "lut.h"
#include "useful_functions.h"


// ============================================================ //


typedef struct UserModelStruct 
{
    struct Actuator{
        int* Motor;
        // index of the corresponding values in MBSdataStruct->ux/uxd/ux0
    } Actuator;
 
} UserModelStruct;

UserModelStruct* init_UserModelStruct();
void free_UserModelStruct(UserModelStruct* ums);

// ============================================================ //
 
# endif
