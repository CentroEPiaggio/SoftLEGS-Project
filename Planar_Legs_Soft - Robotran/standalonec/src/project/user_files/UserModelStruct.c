//---------------------------
// C-code automatically generated from Gen_mds_user project
//
//
// Last update : Sat Nov  7 15:01:04 2015
//---------------------------




#include "UserModelStruct.h"

#ifdef STANDALONE
#include "mbs_xml_reader.h"
#endif

// ============================================================ //


UserModelStruct* init_UserModelStruct() 
{
    UserModelStruct* ums;
    ums = (UserModelStruct*)malloc(sizeof(UserModelStruct));
    ums->Actuator.Motor = get_int_vec(93+1);
    ums->Actuator.Motor[0] = 93+1;
 
    return ums;
}

void free_UserModelStruct(UserModelStruct* ums) 
{
    free_int_vec(ums->Actuator.Motor);
    free(ums);
}

#ifdef STANDALONE
 void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums)
{

    int ind;
    int ind_state_value = 0;

    for(ind=0; ind<gen->user_models->user_model_list[0]->parameter_list[0]->n_value; ind++)
    {
        ums->Actuator.Motor[ind] = ind_state_value;
        ind_state_value++;
    }
 
}
#endif
// ============================================================ //
 
 #ifndef STANDALONE

 UserModelStruct * loadUserModel(mxArray *usm_ptr)
{
 
	UserModelStruct* ums;
	int ind;
    int ind_state_value = 0;

    ums = (UserModelStruct*)malloc(sizeof(UserModelStruct));
    ums->Actuator.Motor = get_int_vec(93+1);
    

	for(ind=0; ind<93; ind++)
    {
        ums->Actuator.Motor[ind] = ind_state_value;
        ind_state_value++;
    }
 
    return ums;
}

void freeUserModel(UserModelStruct *ums)
{
	free_int_vec(ums->Actuator.Motor);
    free(ums);

}
storeUserModel(UserModelStruct *ums, mxArray * ump)
{
 
}

#endif
