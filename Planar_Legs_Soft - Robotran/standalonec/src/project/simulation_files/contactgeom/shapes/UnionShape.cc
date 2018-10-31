#include "UnionShape.hh"
#include <iostream>

namespace ContactGeom{

/*! \brief constructor
 *
 * \param[in] parent_shape parent shape
 */
UnionShape::UnionShape(MBSdataStruct *MBSdata, GeometryShape *parent_shape): GeometryShape(parent_shape)
{
    type = GEOM_UNION;

    this->MBSdata = MBSdata;
}

/*! \brief destructor
 */
UnionShape::~UnionShape()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        delete list_geom[i];
    }
}

/*! \brief add a new union shape
 * 
 * \return initialized union shape
 */
UnionShape* UnionShape::add_union()
{
    UnionShape *new_union;

    new_union = new UnionShape(MBSdata, this);

    return static_cast<UnionShape*>(add_geometry(new_union));
}

/*! \brief add a new geometry and returns the corresponding pointer
 */
GeometryShape* UnionShape::add_geometry(GeometryShape *geom)
{
    // all_contact_flag default: same as parent
    if (all_contact_flag)
    {
        geom->set_all_contact_default();
    }
    else
    {
        geom->set_no_contact_default();
    }

    // prohibited (or not) lists
    for(unsigned int i=0; i<prohibited_list.size(); i++)
    {
        geom->add_prohibited_simple(prohibited_list[i]);
        prohibited_list[i]->add_prohibited_simple(geom);
    }

    for(unsigned int i=0; i<no_prohibited_list.size(); i++)
    {
        geom->add_no_prohibited_simple(no_prohibited_list[i]);
        no_prohibited_list[i]->add_no_prohibited_simple(geom);
    }

    // list of children
    list_geom.push_back(geom);

    add_child_recursively(geom);

    return geom;
}

/*! \brief add a child recursively to the ancestors
 * 
 * \param[in] child child shape to add
 */
void UnionShape::add_child_recursively(GeometryShape *child)
{
    genealogy.push_back(child);

    if (parent_shape != NULL)
    {
        parent_shape->add_child_recursively(child);
    }
}

/*! \brief print the genealogy of this body
 */
void UnionShape::list_genealogy()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->list_genealogy();
    }
    
    GeometryShape::list_genealogy();
}

/*! \brief check if all elements are union shapes or rigid shapes
 * 
 * also check list members
 */
void UnionShape::check_list()
{
    GeometryShape *cur_geom;

    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        cur_geom = list_geom[i];

        switch (cur_geom->get_type())
        {
            case GEOM_UNION : cur_geom->check_list(); break;
            case GEOM_RIGID : cur_geom->check_list(); break;
        
            default:
                std::cout << "Error: not a UnionShape !" << std::endl;
                exit(EXIT_FAILURE);
                break;
        }
    }
}

/*! \brief update the bodies kinematics
 */
void UnionShape::update_kinematics()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->update_kinematics();
    }

    // check_flag
    update_check();
}

/*! \brief update the user state
 */
void UnionShape::user_states()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->user_states();
    }
}

/*! \brief update the global check_flag
 */
void UnionShape::update_check_flag()
{
    check_flag = 1;

    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->update_check_flag();

        if (!list_geom[i]->get_check_flag())
        {
            check_flag = 0;
        }
    }
}

/*! \brief update the check related variables
 */
void UnionShape::update_check()
{
    double cur_radius;
    Vector3D cur_center;

    // check sphere not applicable
    if ( (!check_flag) || (list_geom.size() < 1) )
    {
        return;
    }

    // copy min-max of the first element
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<2; j++)
        {
            check_min_max[i][j] = list_geom[0]->get_check_min_max(i, j);
        }
    }

    // copy check sphere of the first element
    check_radius = list_geom[0]->get_check_radius();
    check_center = list_geom[0]->get_check_center();

    // loop on all the remaining bodies
    for(unsigned int i=1; i<list_geom.size(); i++)
    {
        // check min and max on all the shapes
        for(int j=0; j<3; j++)
        {
            // new minimum
            if (list_geom[i]->get_check_min_max(j, 0) < check_min_max[j][0])
            {
                check_min_max[j][0] = list_geom[i]->get_check_min_max(j, 0);
            }

            // new maximum
            if (list_geom[i]->get_check_min_max(j, 1) > check_min_max[j][1])
            {
                check_min_max[j][1] = list_geom[i]->get_check_min_max(j, 1);
            }
        }

        // check warpping sphere of all shapes
        wrapping_spheres(check_radius, check_center, list_geom[i]->get_check_radius(), list_geom[i]->get_check_center(), cur_radius, cur_center);

        check_radius = cur_radius;
        check_center = cur_center;
    }
}

/*! \brief set 'all_contact_flag' to 1 (recursively)
 */
void UnionShape::set_all_contact_default()
{
    GeometryShape::set_all_contact_default();

    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->set_all_contact_default();
    }
}

/*! \brief set 'all_contact_flag' to 0 (recursively)
 */
void UnionShape::set_no_contact_default()
{
    GeometryShape::set_no_contact_default();

    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->set_no_contact_default();
    }
}

/*! \brief main update of F and T (in main UnionShape)
 */
void UnionShape::update_F_T_main()
{
    GeometryShape *geom_1, *geom_2;

    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        geom_1 = list_geom[i];

        for(unsigned int j=i+1; j<list_geom.size(); j++)
        {
            geom_2 = list_geom[j];

            if (geom_1->contact_possible(geom_2))
            {
                geom_1->update_F_T(geom_2);
            }
        }
    }
}

/*! \brief reset the forces and torques to 0
 */
void UnionShape::reset_F_T()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->reset_F_T();
    }
}

/*! \brief update the forces and torques
 */
void UnionShape::update_F_T(GeometryShape *other_shape)
{
    GeometryShape *cur_geom;

    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        cur_geom = list_geom[i];

        if (cur_geom->contact_possible(other_shape))
        {
            cur_geom->update_F_T(other_shape);
        }
    }
}

/*! \brief gather all the forces and torques of the basic shapes at the ExtForce sensor point
 */
void UnionShape::gather_F_T()
{
    for(unsigned int i=0; i<list_geom.size(); i++)
    {
        list_geom[i]->gather_F_T();
    }
}

}
