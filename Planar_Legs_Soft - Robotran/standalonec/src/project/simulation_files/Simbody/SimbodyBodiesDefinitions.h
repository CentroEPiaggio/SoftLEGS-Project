/*
 * Simbody bodies definition
 *
 * authors: Alexandra Zobova & Timothee Habra
 */

#ifdef SIMBODY

#include "user_all_id.h"

//#define VIZ

// path to the .obj mesh files
#define BODIES_OBJ_PATH PROJECT_ABS_PATH,"/src/project/simulation_files/Simbody/mesh_obj"

// number of bodies with contact
#define NB_CONTACT_BODIES 2

// sensors tabulars
#define S_SENSORS_ARRAY {LFoot_sens_id, RFoot_sens_id}
#define F_SENSORS_ARRAY {LFoot_force_id, RFoot_force_id}

#endif
