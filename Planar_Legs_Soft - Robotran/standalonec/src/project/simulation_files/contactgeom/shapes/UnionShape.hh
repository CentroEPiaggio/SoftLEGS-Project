/*! 
 * \author Nicolas Van der Noot
 * \file UnionShape.hh
 * \brief UnionShape class
 */

#ifndef _UNION_SHAPE_HH_
#define _UNION_SHAPE_HH_

#include "GeometryShape.hh"
#include "MBSdataStruct.h"
#include <vector>

namespace ContactGeom{

class UnionShape: public GeometryShape
{
    public:
        UnionShape(MBSdataStruct *MBSdata, GeometryShape *parent_shape);
        virtual ~UnionShape();
    
        UnionShape* add_union();

        virtual void update_kinematics();
        virtual void update_check();
        virtual void update_check_flag();
        virtual void check_list();
        virtual void user_states();

        void update_F_T_main();
        virtual void reset_F_T();
        virtual void update_F_T(GeometryShape *other_shape);
        virtual void add_child_recursively(GeometryShape *child);

        GeometryShape* add_geometry(GeometryShape *geom);

        MBSdataStruct* get_mbs_data() const { return MBSdata; }

        virtual void set_all_contact_default();
        virtual void set_no_contact_default();

        virtual void gather_F_T();
        virtual void list_genealogy();

    protected:
        MBSdataStruct *MBSdata; ///< Robotran structure

        std::vector<GeometryShape*> list_geom; ///< list with geometry shapes
};

}
#endif
