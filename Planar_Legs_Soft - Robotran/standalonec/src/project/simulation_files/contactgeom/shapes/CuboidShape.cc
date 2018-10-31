#include "CuboidShape.hh"
#include "PlaneShape.hh"
#include "SphereShape.hh"

namespace ContactGeom{

/// norm in 3D
inline double norm_3D(double x, double y, double z) { return sqrt(x*x + y*y + z*z); }

/// maximum of three values
inline double max_three(double x0, double x1, double x2) { return ( x0 < x1 ? ( x1 < x2 ? x2 : x1 ) : ( x0 < x2 ? x2 : x0 )); }

/// index of the maximum of three values
inline int max_three_index(double x0, double x1, double x2) { return ( x0 < x1 ? ( x1 < x2 ? 2 : 1 ) : ( x0 < x2 ? 2 : 0 )); }

/// index of the medium value
inline int medium_three_index(double x0, double x1, double x2) { return ( x0 < x1 ? ( x1 < x2 ? 1 : (x0 < x2 ? 2 : 0) ) : ( x0 < x2 ? 0 : ( x1 < x2 ? 2 : 1 ) ) ); }

/// index of the minimum of three values
inline int min_three_index(double x0, double x1, double x2) { return ( x0 < x1 ? ( x0 < x2 ? 0 : 2 ) : ( x1 < x2 ? 1 : 2 )); }

/// check if the signs are the same
inline int same_sign(double x, double y) { return ( (x <= 0.0) && (y <= 00) ) || ( (x >= 0.0) && (y >= 0.0) ); }

/// check if a value is in a given range
inline int is_in_range(double x, double x_min, double x_max) { return ((x_min <= x) && (x <= x_max)); }

/// limit the range
inline double limit_range(double x, double x_min, double x_max) { return ( x < x_min ? x_min : ( x > x_max ? x_max : x ) ); }

/// square
inline double square(double x) { return x*x; }

/*! \brief constructor
 *
 * \param[in] d cuboid depth [m]
 * \param[in] w cuboid width [m]
 * \param[in] h cuboid height [m]
 * \param[in] rel_position relative position vector from the reference position of RigidShape to the shape reference position [m]
 * \param[in] rel_matrix relative rotation matrix from the RigidShape to the current BasicShape [-]
 * \param[in] parent_shape parent shape
 */
CuboidShape::CuboidShape(double d, double w, double h, Vector3D const& rel_position, RotMatrix const& rel_matrix, int rigid_flag, GeometryShape *parent_shape): BasicShape(rel_position, 1, norm_3D(0.5*d, 0.5*w, 0.5*h), get_nul_vector_3D(), rigid_flag, parent_shape)
{
    type = GEOM_CUBOID;

    this->d = d;
    this->w = w;
    this->h = h;

    max_dim = max_three(d, w, h);

    semi_d = 0.5 * d;
    semi_w = 0.5 * w;
    semi_h = 0.5 * h;

    this->rel_matrix = rel_matrix;

    flag_rel_identity = rel_matrix.check_identity();

    R = rel_matrix; // will be corrected when updating the kinematics
    R_T = get_trans_rot_matrix(R);

    set_vertex_seg_face();
    set_touch_lists();
}

/*! \brief destructor
 */
CuboidShape::~CuboidShape()
{

}

/*! \brief update the check related variables
 */
void CuboidShape::update_check()
{
    set_check_center(P);
}

/*! \brief update the vertex position
 */
void CuboidShape::update_vertex_pos()
{
    for(int i=0; i<8; i++)
    {
        rel_vertex_inertial[i] = R_T*rel_vertex[i];
        vertex_pos[i] = P + rel_vertex_inertial[i];
    }
}

/*! \brief update the minimal and maximal x,y,z values of the check sphere (if applicable)
 */
void CuboidShape::update_check_min_max()
{
    double cur_comp;

    for(int i=0; i<3; i++)
    {
        check_min_max[i][0] = vertex_pos[0].get_comp(i);
        check_min_max[i][1] = vertex_pos[0].get_comp(i);
    }

    for(int i=1; i<8; i++)
    {
        for(int j=0; j<3; j++)
        {
            cur_comp = vertex_pos[i].get_comp(j);

            if (cur_comp < check_min_max[j][0])
            {
                check_min_max[j][0] = cur_comp;
            }
            if (cur_comp > check_min_max[j][1])
            {
                check_min_max[j][1] = cur_comp;
            }
        }
    }
}

/*! \brief update the kinematics values of the basic body
 * 
 * \param[in] parent_P  absolute position vector of parent RigidShape [m]
 * \param[in] parent_V  absolute velocity vector of parent RigidShape [m/s]
 * \param[in] parent_OM absolute angle velocity vector of parent RigidShape [rad/s]
 * \param[in] parent_R  absolute rotation matrix of parent RigidShape [-]
 */
void CuboidShape::update_kinematics(Vector3D const& parent_P, Vector3D const& parent_V, Vector3D const& parent_OM, RotMatrix const& parent_R)
{
    flag_vertex_seg_face = 0;

    if (flag_rel_identity)
    {
        R = parent_R;
    }
    else
    {
        R = rel_matrix * parent_R;
    }
    
    R_T = get_trans_rot_matrix(R);

    BasicShape::update_kinematics(parent_P, parent_V, parent_OM, parent_R);

    update_vertex_pos();

    update_check_min_max();
}

/*! \brief update the forces and torques with another basic shape
 * 
 * \param[in,out] other_shape other basic shape
 * \return 1 if contact, 0 otherwise
 */
int CuboidShape::update_F_T_basic(BasicShape *other_shape)
{
    CuboidShape *other_cuboid;
    PlaneShape *other_plane;

    switch (other_shape->get_type())
    {
        case GEOM_PLANE :

            other_plane = static_cast<PlaneShape*>(other_shape);

            // check if contact possible
            if (other_plane->get_flag_z_norm())
            {
                if (check_min_max[2][0] > -other_plane->get_d())
                {
                    return 0;
                }
            }
            else if (other_plane->dist_point(P) > check_radius)
            {
                return 0;
            }

            if (!flag_vertex_seg_face)
            {
                update_vertex_seg_face();
                flag_vertex_seg_face = 1;
            }

            return update_F_T_plane(other_shape);

        case GEOM_SPHERE :

            if (!flag_vertex_seg_face)
            {
                update_vertex_seg_face();
                flag_vertex_seg_face = 1;
            }

            return update_F_T_sphere(other_shape);

        case GEOM_CUBOID :

            other_cuboid = static_cast<CuboidShape*>(other_shape);

            if (!flag_vertex_seg_face)
            {
                update_vertex_seg_face();
                flag_vertex_seg_face = 1;
            }

            if (!other_cuboid->get_flag_vertex_seg_face())
            {
                other_cuboid->update_vertex_seg_face();
                other_cuboid->set_flag_vertex_seg_face(1);
            }

            return update_F_T_cuboid(other_shape);
    
        default:
            std::cout << "Error: unknown basic shape !" << std::endl;
            exit(EXIT_FAILURE);
            break;
    }
}

/*! \brief clear 'contact_face_vertices'
 */
void CuboidShape::contact_face_vertices_clear()
{
    for(int i=0; i<7; i++)
    {
        contact_face_vertices[i].clear();
    }
}

/*! \brief update the forces and torques with another plane shape
 * 
 * \param[in,out] other_shape other plane shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int CuboidShape::update_F_T_plane(BasicShape *other_shape)
{
    int cur_size, flag_appex, nb_inter, nb_inside;
    double cur_volume, cur_volume_dot;
    double volume_tot, volume_dot_tot;
    std::vector<int> order_list;
    Vector3D centroid, cur_centroid, cur_normal_base, cur_normal_base_dot;
    Polygon3D *cur_polygon;
    PlaneShape *other_plane;
    Point3D cur_intersect;
    Point3D *cur_point, *main_appex;

    other_plane = static_cast<PlaneShape*>(other_shape);

    main_appex = NULL;

    // reset interest points
    nb_inter  = 0;
    nb_inside = 0;
    intersect_points.clear();
    contact_face_vertices_clear();

    // list all segments to detect contacts with the plane
    for(int i=0; i<12; i++)
    {
        if ( segment[i].intersect_plane(other_plane->get_norm_vec(), other_plane->get_norm_vec_dot(), 
            other_plane->get_d(), other_plane->get_d_dot(), other_plane->get_flag_z_norm(), cur_intersect) )
        {
            intersect_points_storage[nb_inter] = cur_intersect;
            cur_point = intersect_points_storage[nb_inter].get_ptr();
            
            // copy references
            intersect_points.push_back(cur_point);

            // add faces in contact
            contact_face_vertices[6].push_back(cur_point);

            for(int j=0; j<2; j++)
            {
                contact_face_vertices[seg_face[i][j]].push_back(cur_point);
            }

            // contact points for tangential forces
            add_tg_point_speed(other_shape, cur_intersect.get_point());

            nb_inter++;
        }   
    }

    // ordering contact list
    order_vertex_polygon_3D(intersect_points, order_list, other_plane->get_norm_vec());
    apply_order_polygon_3D(intersect_points, order_list);

    // normal vector
    contact.set_norm_vec(other_plane->get_norm_vec());

    // list all vertices to detect vertices in contact
    for(int i=0; i<8; i++)
    {
        if (other_plane->point_inside(vertex[i].get_point()))
        {
            // main appex is first point inside
            if (!nb_inside)
            {
                main_appex = &vertex[i];
            }

            nb_inside++;

            // add faces in contact
            for(int j=0; j<3; j++)
            {
                contact_face_vertices[vertex_face[i][j]].push_back(&vertex[i]);
            }
        }
    }   

    // safety
    if ( (!nb_inside) || (nb_inter < 3) ) { return 0; }

    volume_tot = 0.0;
    volume_dot_tot = 0.0;
    centroid.reset();

    // list all possible contact faces
    for(int i=0; i<7; i++)
    {
        cur_size = contact_face_vertices[i].size();

        if (cur_size >= 3)
        {
            flag_appex = 0;

            for(int j=0; j<cur_size; j++)
            {
                if (contact_face_vertices[i][j] == main_appex)
                {
                    flag_appex = 1;
                    break;
                }
            }

            // face generating a volume
            if (!flag_appex)
            {
                if (i == 6)
                {
                    cur_normal_base = other_plane->get_norm_vec();
                    cur_normal_base_dot = other_plane->get_norm_vec_dot();
                }
                else
                {
                    cur_polygon = &face[i];
                    cur_normal_base = cur_polygon->get_normal_vec();
                    cur_normal_base_dot = cur_polygon->get_normal_vec_dot();
                }

                // ordering vertices list
                order_vertex_polygon_3D(contact_face_vertices[i], order_list, cur_normal_base);
                apply_order_polygon_3D(contact_face_vertices[i], order_list);

                volume_pyramid_3D(contact_face_vertices[i], cur_normal_base, cur_normal_base_dot, main_appex, cur_volume, cur_volume_dot);          
                centroid_pyramid_3D(contact_face_vertices[i], main_appex, cur_centroid);

                volume_tot += cur_volume;
                volume_dot_tot += cur_volume_dot;

                centroid += cur_volume * cur_centroid;      
            }
        }
    }

    // safety
    if (!volume_tot) { return 0; }

    centroid /= volume_tot;

    // center force
    contact.set_center(centroid);

    // volume
    contact.set_volume(volume_tot);
    contact.set_volume_dot(volume_dot_tot);

    // copy result to other shape
    contact.copy_reverse(other_shape->get_contact());

    return 1;
}

/*! \brief compute h and its derivative for the distance between sphere and cuboid
 * 
 * \param[in] out_seg vector out of the cuboid
 * \param[in] diff_center distance vector bewteen the sphere center and the main contact point
 * \param[in] diff_center_dot derivative of 'diff_center'
 * \param[in] radius radius of the sphere [m]
 * \param[out] h h quantity (penetration) [m]
 * \param[out] h_dot derivative of 'h' [m/s]
 */
void CuboidShape::compute_h_hdot(double out_seg, double diff_center, double diff_center_dot, double radius, double &h, double &h_dot)
{
    if (same_sign(out_seg, diff_center)) // normal case, penetration less than half the distance
    {
        if (diff_center >= 0.0)
        {
            h = radius - diff_center;
            h_dot = -diff_center_dot;
        }
        else
        {
            h = radius + diff_center;
            h_dot = diff_center_dot;
        }
    }
    else // penetration more than half the distance
    {
        if (diff_center >= 0.0)
        {
            h = radius + diff_center;
            h_dot = diff_center_dot;
        }
        else
        {
            h = radius - diff_center;
            h_dot = -diff_center_dot;
        }
    }

    // safety
    if ((h < 0.0) || (h > 2.0*radius))
    {
        h = 0.0;
        h_dot = 0.0;
    }
}

/*! \brief ordering of the heights and their derivatives
 * 
 * \param[in] ha first height [m]
 * \param[in] hb second height [m]
 * \param[in] ha_dot derivative of 'ha' [m/s]
 * \param[in] hb_dot derivative of 'hb' [m/s]
 * \param[out] h1 smaller value between 'ha' and 'hb' [m]
 * \param[out] h2 medium value between 'ha' and 'hb' [m]
 * \param[out] h1_dot derivative of 'h1' [m/s]
 * \param[out] h2_dot derivative of 'h2' [m/s]
 */
void CuboidShape::order_h_hdot(double ha, double hb, double ha_dot, double hb_dot, double &h1, double &h2, double &h1_dot, double &h2_dot)
{
    if (ha < hb)
    {
        h1 = ha;
        h2 = hb;
        h1_dot = ha_dot;
        h2_dot = hb_dot;
    }
    else
    {
        h1 = hb;
        h2 = ha;
        h1_dot = hb_dot;
        h2_dot = ha_dot;
    }
}

/*! \brief ordering of the heights and their derivatives
 * 
 * \param[in] hx height in x [m]
 * \param[in] hy height in y [m]
 * \param[in] hz height in z [m]
 * \param[in] hx_dot derivative of 'hx' [m/s]
 * \param[in] hy_dot derivative of 'hy' [m/s]
 * \param[in] hz_dot derivative of 'hz' [m/s]
 * \param[out] h1 smaller value between 'hx', 'hy' and 'hz' [m]
 * \param[out] h2 medium value between 'hx', 'hy' and 'hz' [m]
 * \param[out] h3 larger value between 'hx', 'hy' and 'hz' [m]
 * \param[out] h1_dot derivative of 'h1' [m/s]
 * \param[out] h2_dot derivative of 'h2' [m/s]
 * \param[out] h3_dot derivative of 'h3' [m/s]
 */
void CuboidShape::order_h_hdot(double hx, double hy, double hz, double hx_dot, double hy_dot, double hz_dot, 
    double &h1, double &h2, double &h3, double &h1_dot, double &h2_dot, double &h3_dot)
{
    int min_index;

    min_index = min_three_index(hx, hy, hz);

    switch (min_index)
    {
        case 0:
            h1 = hx;
            h1_dot = hx_dot;

            if (hy < hz)
            {
                h2 = hy;
                h3 = hz;
                h2_dot = hy_dot;
                h3_dot = hz_dot;
            }
            else
            {
                h2 = hz;
                h3 = hy;
                h2_dot = hz_dot;
                h3_dot = hy_dot;
            }
            break;

        case 1:
            h1 = hy;
            h1_dot = hy_dot;

            if (hx < hz)
            {
                h2 = hx;
                h3 = hz;
                h2_dot = hx_dot;
                h3_dot = hz_dot;
            }
            else
            {
                h2 = hz;
                h3 = hx;
                h2_dot = hz_dot;
                h3_dot = hx_dot;
            }
            break;

        case 2:
            h1 = hz;
            h1_dot = hz_dot;

            if (hx < hy)
            {
                h2 = hx;
                h3 = hy;
                h2_dot = hx_dot;
                h3_dot = hy_dot;
            }
            else
            {
                h2 = hy;
                h3 = hx;
                h2_dot = hy_dot;
                h3_dot = hx_dot;
            }
            break;
    
        default:
            std::cout << "Error: unknown index !" << std::endl;
            exit(EXIT_FAILURE);
            break;
    }
}

/*! \brief update the forces and torques with another sphere shape
 * 
 * \param[in,out] other_shape other sphere shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int CuboidShape::update_F_T_sphere(BasicShape *other_shape)
{
    int cur_nb_inter, nb_inter, nb_inside, cur_size, flag_appex, flag_face, nb_cut_points, bad_index;
    double volume_tot, volume_dot_tot, cur_volume, cur_volume_dot, semi_length, ratio, cur_val;
    double cur_dist, cur_dist_dot, h, h_dot, h_abs, radius, r_surf;
    double hx, hy, hz, hx_dot, hy_dot, hz_dot, h1, h2, h1_dot, h2_dot;
    double cur_val_dot, semi_length_dot, ratio_dot, cur_num, cur_denom;
    std::vector<int> order_list;
    SphereShape *other_sphere;
    Point3D cur_inter_1, cur_inter_2, mid_inter, cut_cap_1, cut_cap_2, cur_ext_surf;
    Point3D *main_appex, *cur_point;
    Vector3D centroid, cur_centroid, normal_vec, normal_vec_dot, centroid_base;
    Vector3D closest_point, cur_normal_base, cur_normal_base_dot;
    Vector3D diff_center_inertial, diff_center_inertial_dot, diff_center, diff_center_dot;
    Vector3D axis_1, axis_2, out_seg, cut_1, cut_2, cut_norm_1, cut_norm_2;
    Polygon3D *cur_polygon;
    Segment3D cur_seg;

    other_sphere = static_cast<SphereShape*>(other_shape);

    main_appex = NULL;

    nb_inter = 0;
    nb_inside = 0;
    intersect_points.clear();
    contact_face_vertices_clear();

    // loop on all segments
    for(unsigned int i=0; i<12; i++)
    {
        cur_nb_inter = segment[i].intersect_sphere(other_sphere->get_P(), other_sphere->get_V(), other_sphere->get_radius(), cur_inter_1, cur_inter_2);

        if (cur_nb_inter >= 1)
        {
            intersect_points_storage[nb_inter] = cur_inter_1;
            cur_point = intersect_points_storage[nb_inter].get_ptr();

            intersect_points.push_back(cur_point);
            contact_face_vertices[6].push_back(cur_point);

            for(int j=0; j<2; j++)
            {
                contact_face_vertices[seg_face[i][j]].push_back(cur_point);
            }

            add_tg_point_speed(other_shape, cur_inter_1.get_point());

            if (cur_nb_inter == 2)
            {
                intersect_points_storage[nb_inter+1] = cur_inter_2;
                cur_point = intersect_points_storage[nb_inter+1].get_ptr();

                intersect_points.push_back(cur_point);
                contact_face_vertices[6].push_back(cur_point);

                for(int j=0; j<2; j++)
                {
                    contact_face_vertices[seg_face[i][j]].push_back(cur_point);
                }

                add_tg_point_speed(other_shape, cur_inter_2.get_point());
            }

            nb_inter += cur_nb_inter;
        }   
    }

    // safety
    if (nb_inter == 1) { return 0; }

    volume_tot = 0.0;
    volume_dot_tot = 0.0;

    radius = other_sphere->get_radius();

    if (!nb_inter)
    {
        flag_face = 0;

        // loop on all the cuboid faces
        for(unsigned int i=0; i<6; i++)
        {
            face[i].dist_point_plane(other_sphere->get_P(), other_sphere->get_V(), closest_point, cur_dist, cur_dist_dot);

            if (fabs(cur_dist) <= radius)
            {
                if (face[i].check_point_inside(closest_point))
                {
                    flag_face = 1;  

                    // center of the sphere inside the cuboid
                    if ( point_inside(other_sphere->get_P()) )
                    {
                        if (cur_dist > 0.0)
                        {
                            cur_dist = -cur_dist;
                            cur_dist_dot = -cur_dist_dot;
                        }
                    }
                    else // center of the sphere outside the cuboid
                    {
                        if (cur_dist < 0.0)
                        {
                            cur_dist = -cur_dist;
                            cur_dist_dot = -cur_dist_dot;
                        }
                    }

                    h = radius - cur_dist;
                    h_dot = -cur_dist_dot;

                    volume_tot += ( M_PI*h*h*(3.0*radius-h) ) / 3.0;
                    volume_dot_tot += M_PI*h*(2*radius*h_dot - h*h_dot);

                    // safety
                    if(!volume_tot) { return 0; }

                    // center force
                    contact.set_center(closest_point);

                    // normal vector
                    normal_vec = face[i].get_normal_vec();

                    if (scalar_vector_3D(P - other_sphere->get_P(), normal_vec) < 0.0)
                    {
                        normal_vec = -normal_vec;
                    }
                    
                    contact.set_norm_vec(normal_vec);

                    h_abs = fabs(h);

                    // radius of the contact surface
                    r_surf = sqrt(h_abs*(2.0*radius-h_abs));

                    // normalized tangential plane vectors (orthogonal)
                    normal_vec.ortho_matrix_3D(axis_1, axis_2);

                    // contact points for tangential forces
                    add_tg_point_speed(other_shape, closest_point + r_surf*axis_1);
                    add_tg_point_speed(other_shape, closest_point - r_surf*axis_1);
                    add_tg_point_speed(other_shape, closest_point + r_surf*axis_2);
                    add_tg_point_speed(other_shape, closest_point - r_surf*axis_2);
                }
            }

            if (flag_face)
            {
                break;
            }
        }

        // safety
        if(!flag_face) { return 0; }
    }
    else if (nb_inter == 2)
    {
        nb_cut_points = 0;

        mid_inter = get_mid_point(intersect_points_storage[0], intersect_points_storage[1]);

        for(int i=0; i<6; i++)
        {
            if (contact_face_vertices[i].size() == 2)
            {
                normal_vec = face[i].get_normal_vec();
                normal_vec_dot = face[i].get_normal_vec_dot();

                cur_ext_surf.update_kinematics(mid_inter.get_point() - max_dim*normal_vec, mid_inter.get_point_dot() - max_dim*normal_vec_dot);

                cur_seg.renew_seg(&mid_inter, &cur_ext_surf);

                if (cur_seg.intersect_sphere(other_sphere->get_P(), other_sphere->get_V(), radius, cur_inter_1, cur_inter_2) == 1)
                {
                    add_tg_point_speed(other_shape, cur_inter_1.get_point());

                    if (!nb_cut_points)
                    {
                        cut_1 = cur_inter_1.get_point();
                        cut_norm_1 = normal_vec;
                    }
                    else if (nb_cut_points == 1)
                    {
                        cut_2 = cur_inter_1.get_point();
                        cut_norm_2 = normal_vec;
                    }

                    nb_cut_points++;
                }
            }
        }

        // safety
        if (nb_cut_points != 2) { return 0; }

        diff_center_inertial = other_sphere->get_P() - mid_inter.get_point();
        diff_center_inertial_dot = other_sphere->get_V() - mid_inter.get_point_dot();

        diff_center = R * diff_center_inertial;
        diff_center_dot = R*diff_center_inertial_dot - RotMatrix(OM)*R*diff_center_inertial;

        // vector out of the cuboid, going through the segment 
        out_seg = R * (mid_inter.get_point() - P);

        bad_index = min_three_index(fabs(diff_center.get_x()), fabs(diff_center.get_y()), fabs(diff_center.get_z()));

        switch (bad_index)
        {
            case 0:
                compute_h_hdot(out_seg.get_y(), diff_center.get_y(), diff_center_dot.get_y(), radius, hy, hy_dot);
                compute_h_hdot(out_seg.get_z(), diff_center.get_z(), diff_center_dot.get_z(), radius, hz, hz_dot);
                order_h_hdot(hy, hz, hy_dot, hz_dot, h1, h2, h1_dot, h2_dot);
                break;

            case 1:
                compute_h_hdot(out_seg.get_x(), diff_center.get_x(), diff_center_dot.get_x(), radius, hx, hx_dot);
                compute_h_hdot(out_seg.get_z(), diff_center.get_z(), diff_center_dot.get_z(), radius, hz, hz_dot);
                order_h_hdot(hx, hz, hx_dot, hz_dot, h1, h2, h1_dot, h2_dot);
                break;

            case 2:
                compute_h_hdot(out_seg.get_x(), diff_center.get_x(), diff_center_dot.get_x(), radius, hx, hx_dot);
                compute_h_hdot(out_seg.get_y(), diff_center.get_y(), diff_center_dot.get_y(), radius, hy, hy_dot);
                order_h_hdot(hx, hy, hx_dot, hy_dot, h1, h2, h1_dot, h2_dot);
                break;
        
            default:
                break;
        }

        cur_volume = ( M_PI*h1*h1*(3.0*radius-h1) ) / 3.0;
        cur_volume_dot = M_PI*h1*(2*radius*h1_dot - h1*h1_dot);

        // divide by ratio
        cur_val = 2.0*radius*h1 - h1*h1;

        if (cur_val > 0.0)
        {
            cur_val_dot = 2.0*radius*h1_dot - 2.0*h1*h1_dot;

            semi_length = sqrt(cur_val);
            semi_length_dot = cur_val_dot / (2.0 * semi_length);

            cur_num = semi_length + h2 - radius;
            cur_denom = 2.0 * semi_length;

            ratio = cur_num / cur_denom;
            ratio_dot = ((semi_length_dot + h2_dot) * cur_denom - cur_num * 2.0 * semi_length_dot) / square(cur_denom);

            cur_volume_dot = ratio_dot * cur_volume + ratio * cur_volume_dot;
            cur_volume = ratio * cur_volume;
        }
        
        volume_tot += cur_volume;
        volume_dot_tot += cur_volume_dot;

        // center force
        contact.set_center(0.5 * (cut_1 + cut_2));

        // normal vector
        normal_vec = cross_vector_3D(cross_vector_3D(cut_norm_1, cut_norm_2), cut_1 - cut_2);
        normal_vec.normalize();

        if (scalar_vector_3D(P - other_sphere->get_P(), normal_vec) < 0.0)
        {
            normal_vec = -normal_vec;
        }

        contact.set_norm_vec(normal_vec);
    }
    else if (nb_inter >= 3)
    {
        centroid.reset();

        // loop on all the vertices
        for(unsigned int i=0; i<8; i++)
        {
            if (other_sphere->point_inside(vertex[i].get_point()))
            {
                if (!nb_inside)
                {
                    main_appex = &vertex[i];
                }

                nb_inside++;

                // add faces in contact
                for(int j=0; j<3; j++)
                {
                    contact_face_vertices[vertex_face[i][j]].push_back(&vertex[i]);
                }
            }
        }

        // safety
        if (!nb_inside) { return 0; }

        // normal vector
        normal_polygon_3D(intersect_points, normal_vec, normal_vec_dot);

        // centroid of the base
        centroid_polygon_3D(intersect_points, centroid_base);

        // ordering contact list
        order_vertex_polygon_3D(intersect_points, order_list, normal_vec);
        apply_order_polygon_3D(intersect_points, order_list);

        // correct normal vector sign
        if (scalar_vector_3D(normal_vec, P - centroid_base) < 0.0)
        {
            normal_vec = -normal_vec;
            normal_vec_dot = -normal_vec_dot;
        }
        contact.set_norm_vec(normal_vec);

        // list all possible contact faces
        for(int i=0; i<7; i++)
        {
            cur_size = contact_face_vertices[i].size();

            if (cur_size >= 3)
            {
                flag_appex = 0;

                for(int j=0; j<cur_size; j++)
                {
                    if (contact_face_vertices[i][j] == main_appex)
                    {
                        flag_appex = 1;
                        break;
                    }
                }

                // face generating a volume
                if (!flag_appex)
                {
                    if (i == 6)
                    {
                        cur_normal_base = normal_vec;
                        cur_normal_base_dot = normal_vec_dot;
                    }
                    else
                    {
                        cur_polygon = &face[i];
                        cur_normal_base = cur_polygon->get_normal_vec();
                        cur_normal_base_dot = cur_polygon->get_normal_vec_dot();
                    }

                    // ordering vertices list
                    order_vertex_polygon_3D(contact_face_vertices[i], order_list, cur_normal_base);
                    apply_order_polygon_3D(contact_face_vertices[i], order_list);

                    volume_pyramid_3D(contact_face_vertices[i], cur_normal_base, cur_normal_base_dot, main_appex, cur_volume, cur_volume_dot);          
                    centroid_pyramid_3D(contact_face_vertices[i], main_appex, cur_centroid);

                    volume_tot += cur_volume;
                    volume_dot_tot += cur_volume_dot;

                    centroid += cur_volume * cur_centroid;      
                }               
            }
        }

        // safety
        if (!volume_tot) { return 0; }

        centroid /= volume_tot;

        // center force
        contact.set_center(centroid);
    }
    else
    {
        return 0; // safety
    }

    // volume
    contact.set_volume(volume_tot);
    contact.set_volume_dot(volume_dot_tot);

    // copy result to other shape
    contact.copy_reverse(other_shape->get_contact());

    return 1;
}

/*! \brief update the forces and torques with another cuboid shape
 * 
 * \param[in,out] other_shape other cuboid shape (BasicShape form)
 * \return 1 if contact, 0 otherwise
 */
int CuboidShape::update_F_T_cuboid(BasicShape *other_shape)
{
    int cur_size, flag_appex;
    int nb_inter, nb_inside_1, nb_inside_2;
    double cur_volume, cur_volume_dot;
    double volume_tot, volume_dot_tot;
    std::vector<int> order_list;
    CuboidShape *other_cuboid;
    Vector3D centroid, cur_centroid, centroid_base;
    Vector3D normal_vec, normal_vec_dot, cur_normal_base, cur_normal_base_dot;
    Polygon3D *cur_polygon;
    Point3D cur_intersect;
    Point3D *cur_point, *main_appex_1, *main_appex_2;

    other_cuboid = static_cast<CuboidShape*>(other_shape);

    main_appex_1 = NULL;
    main_appex_2 = NULL;

    // reset interest points
    nb_inter    = 0;
    nb_inside_1 = 0;
    nb_inside_2 = 0;

    intersect_points.clear();

    contact_face_vertices_clear();
    other_cuboid->contact_face_vertices_clear();

    // loop on all segments of this cuboid
    for(int i=0; i<12; i++)
    {
        // loop on all faces of the other cuboid
        for(int j=0; j<6; j++)
        {
            if (other_cuboid->get_face(j).contact_segment(segment[i], cur_intersect))
            {
                intersect_points_storage[nb_inter] = cur_intersect;
                cur_point = intersect_points_storage[nb_inter].get_ptr();
                
                // copy references
                intersect_points.push_back(cur_point);

                // add faces in contact
                contact_face_vertices[6].push_back(cur_point);
                other_cuboid->add_contact_face_vertices(6, cur_point);

                for(int k=0; k<2; k++)
                {
                    contact_face_vertices[seg_face[i][k]].push_back(cur_point);
                }

                other_cuboid->add_contact_face_vertices(j, cur_point);

                // contact points for tangential forces
                add_tg_point_speed(other_shape, cur_intersect.get_point());

                nb_inter++;
            }
        }
    }

    // loop on all segments of the other cuboid
    for(int i=0; i<12; i++)
    {
        // loop on all faces of this cuboid
        for(int j=0; j<6; j++)
        {
            if (face[j].contact_segment(other_cuboid->get_segment(i), cur_intersect))
            {
                intersect_points_storage[nb_inter] = cur_intersect;
                cur_point = intersect_points_storage[nb_inter].get_ptr();
                
                // copy references
                intersect_points.push_back(cur_point);

                // add faces in contact
                contact_face_vertices[6].push_back(cur_point);
                other_cuboid->add_contact_face_vertices(6, cur_point);

                for(int k=0; k<2; k++)
                {
                    other_cuboid->add_contact_face_vertices(other_cuboid->get_seg_face(i, k), cur_point);
                }

                contact_face_vertices[j].push_back(cur_point);

                // contact points for tangential forces
                add_tg_point_speed(other_shape, cur_intersect.get_point());

                nb_inter++;
            }
        }
    }

    // normal vector
    normal_polygon_3D(intersect_points, normal_vec, normal_vec_dot);

    // centroid of the base
    centroid_polygon_3D(intersect_points, centroid_base);

    // ordering contact list
    order_vertex_polygon_3D(intersect_points, order_list, normal_vec);
    apply_order_polygon_3D(intersect_points, order_list);

    // correct normal vector sign
    if (scalar_vector_3D(normal_vec, P - centroid_base) < 0.0)
    {
        normal_vec = -normal_vec;
        normal_vec_dot = -normal_vec_dot;
    }
    contact.set_norm_vec(normal_vec);

    // list all vertices of this cuboid
    for(int i=0; i<8; i++)
    {
        if (other_cuboid->point_inside(vertex[i].get_point()))
        {
            // main appex is first point inside
            if (!nb_inside_1)
            {
                main_appex_1 = &vertex[i];
            }

            nb_inside_1++;

            // add faces in contact
            for(int j=0; j<3; j++)
            {
                contact_face_vertices[vertex_face[i][j]].push_back(&vertex[i]);
            }
        }
    }

    // list all vertices of the other cuboid
    for(int i=0; i<8; i++)
    {
        if (point_inside(other_cuboid->get_vertex(i).get_point()))
        {
            // main appex is first point inside
            if (!nb_inside_2)
            {
                main_appex_2 = other_cuboid->get_vertex_ptr(i);
            }

            nb_inside_2++;

            // add faces in contact
            for(int j=0; j<3; j++)
            {
                other_cuboid->add_contact_face_vertices(other_cuboid->get_vertex_face(i,j), other_cuboid->get_vertex_ptr(i));
            }
        }
    }

    // safety
    if ( ((!nb_inside_1) && (!nb_inside_2)) || (nb_inter < 3) ) { return 0; }

    volume_tot = 0.0;
    volume_dot_tot = 0.0;
    centroid.reset();

    // list all possible contact faces of this cuboid
    if (nb_inside_1)
    {
        for(int i=0; i<7; i++)
        {
            cur_size = contact_face_vertices[i].size();

            if (cur_size >= 3)
            {
                flag_appex = 0;

                for(int j=0; j<cur_size; j++)
                {
                    if (contact_face_vertices[i][j] == main_appex_1)
                    {
                        flag_appex = 1;
                        break;
                    }
                }

                // face generating a volume
                if (!flag_appex)
                {
                    if (i == 6)
                    {
                        cur_normal_base = normal_vec;
                        cur_normal_base_dot = normal_vec_dot;
                    }
                    else
                    {
                        cur_polygon = &face[i];
                        cur_normal_base = cur_polygon->get_normal_vec();
                        cur_normal_base_dot = cur_polygon->get_normal_vec_dot();
                    }

                    // ordering vertices list
                    order_vertex_polygon_3D(contact_face_vertices[i], order_list, cur_normal_base);
                    apply_order_polygon_3D(contact_face_vertices[i], order_list);

                    volume_pyramid_3D(contact_face_vertices[i], cur_normal_base, cur_normal_base_dot, main_appex_1, cur_volume, cur_volume_dot);            
                    centroid_pyramid_3D(contact_face_vertices[i], main_appex_1, cur_centroid);

                    volume_tot += cur_volume;
                    volume_dot_tot += cur_volume_dot;

                    centroid += cur_volume * cur_centroid;      
                }
            }
        }       
    }

    // list all possible contact faces of the other cuboid
    if (nb_inside_2)
    {
        for(int i=0; i<7; i++)
        {
            cur_size = other_cuboid->get_contact_face_vertices_size(i);

            if (cur_size >= 3)
            {
                flag_appex = 0;

                for(int j=0; j<cur_size; j++)
                {
                    if (other_cuboid->get_contact_face_vertices(i,j) == main_appex_2)
                    {
                        flag_appex = 1;
                        break;
                    }
                }

                // face generating a volume
                if (!flag_appex)
                {
                    if (i == 6)
                    {
                        cur_normal_base = -normal_vec;
                        cur_normal_base_dot = -normal_vec_dot;
                    }
                    else
                    {
                        cur_polygon = other_cuboid->get_face_ptr(i);
                        cur_normal_base = cur_polygon->get_normal_vec();
                        cur_normal_base_dot = cur_polygon->get_normal_vec_dot();
                    }

                    // ordering vertices list
                    order_vertex_polygon_3D(other_cuboid->get_contact_face_vertices(i), order_list, cur_normal_base);
                    apply_order_polygon_3D(other_cuboid->get_contact_face_vertices(i), order_list);

                    volume_pyramid_3D(other_cuboid->get_contact_face_vertices(i), cur_normal_base, cur_normal_base_dot, main_appex_2, cur_volume, cur_volume_dot);          
                    centroid_pyramid_3D(other_cuboid->get_contact_face_vertices(i), main_appex_2, cur_centroid);

                    volume_tot += cur_volume;
                    volume_dot_tot += cur_volume_dot;

                    centroid += cur_volume * cur_centroid;      
                }
            }
        }
    }   

    // safety
    if (!volume_tot) { return 0; }

    centroid /= volume_tot;

    // center force
    contact.set_center(centroid);

    // volume
    contact.set_volume(volume_tot);
    contact.set_volume_dot(volume_dot_tot);

    // copy result to other shape
    contact.copy_reverse(other_shape->get_contact());

    return 1;
}

/*! \brief check if a point is inside the corresponding shape
 * 
 * \param[in] point point to check
 * \return 1 if inside, 0 otherwise
 */
int CuboidShape::point_inside(Vector3D const& point)
{
    Vector3D vec = R * (point - P);

    return (is_in_range(vec.get_x(), -semi_d, semi_d) && is_in_range(vec.get_y(), -semi_w, semi_w) && is_in_range(vec.get_z(), -semi_h, semi_h));
}

/*! \brief update the vertices, segments and faces of the cuboid 
 */
void CuboidShape::update_vertex_seg_face()
{
    // vertices
    for(int i=0; i<8; i++)
    {
        vertex[i].update_kinematics(vertex_pos[i], V + cross_vector_3D(OM, rel_vertex_inertial[i]));
    }

    // segments
    for(int i=0; i<12; i++)
    {
        segment[i].update_kinematics();
    }

    // faces
    for(int i=0; i<6; i++)
    {
        face[i].update_kinematics(P);
    }
}

/*! \brief set the vertices, segments and faces
 */
void CuboidShape::set_vertex_seg_face()
{
    flag_vertex_seg_face = 0;

    // vertices
    rel_vertex[0] = get_vector_3D( semi_d, -semi_w, -semi_h);
    rel_vertex[1] = get_vector_3D( semi_d,  semi_w, -semi_h);
    rel_vertex[2] = get_vector_3D( semi_d,  semi_w,  semi_h);
    rel_vertex[3] = get_vector_3D( semi_d, -semi_w,  semi_h);
    rel_vertex[4] = get_vector_3D(-semi_d, -semi_w, -semi_h);
    rel_vertex[5] = get_vector_3D(-semi_d,  semi_w, -semi_h);
    rel_vertex[6] = get_vector_3D(-semi_d,  semi_w,  semi_h);
    rel_vertex[7] = get_vector_3D(-semi_d, -semi_w,  semi_h);

    for(int i=0; i<8; i++)
    {
        rel_vertex_inertial[i] = rel_vertex[i];
        vertex_pos[i] = rel_vertex_inertial[i];

        vertex[i].update_kinematics(vertex_pos[i], get_nul_vector_3D());
    }

    // segments
    segment[0].add_point(&vertex[0]);
    segment[0].add_point(&vertex[1]);
    segment[1].add_point(&vertex[1]);
    segment[1].add_point(&vertex[2]);
    segment[2].add_point(&vertex[2]);
    segment[2].add_point(&vertex[3]);
    segment[3].add_point(&vertex[3]);
    segment[3].add_point(&vertex[0]);

    segment[4].add_point(&vertex[5]);
    segment[4].add_point(&vertex[4]);
    segment[5].add_point(&vertex[6]);
    segment[5].add_point(&vertex[5]);
    segment[6].add_point(&vertex[7]);
    segment[6].add_point(&vertex[6]);
    segment[7].add_point(&vertex[4]);
    segment[7].add_point(&vertex[7]);

    segment[8].add_point(&vertex[0]);
    segment[8].add_point(&vertex[4]);
    segment[9].add_point(&vertex[5]);
    segment[9].add_point(&vertex[1]);
    segment[10].add_point(&vertex[2]);
    segment[10].add_point(&vertex[6]);
    segment[11].add_point(&vertex[7]);
    segment[11].add_point(&vertex[3]);

    // faces
    face[0].add_segment(&segment[0], P);
    face[0].add_segment(&segment[1], P);
    face[0].add_segment(&segment[2], P);
    face[0].add_segment(&segment[3], P);

    face[0].add_point(&vertex[0]);
    face[0].add_point(&vertex[1]);
    face[0].add_point(&vertex[2]);
    face[0].add_point(&vertex[3]);

    face[1].add_segment(&segment[7], P);
    face[1].add_segment(&segment[6], P);
    face[1].add_segment(&segment[5], P);
    face[1].add_segment(&segment[4], P);

    face[1].add_point(&vertex[4]);
    face[1].add_point(&vertex[7]);
    face[1].add_point(&vertex[6]);
    face[1].add_point(&vertex[5]);

    face[2].add_segment(&segment[0], P);
    face[2].add_segment(&segment[8], P);
    face[2].add_segment(&segment[4], P);
    face[2].add_segment(&segment[9], P);

    face[2].add_point(&vertex[1]);
    face[2].add_point(&vertex[0]);
    face[2].add_point(&vertex[4]);
    face[2].add_point(&vertex[5]);

    face[3].add_segment(&segment[1] , P);
    face[3].add_segment(&segment[9] , P);
    face[3].add_segment(&segment[5] , P);
    face[3].add_segment(&segment[10], P);

    face[3].add_point(&vertex[2]);
    face[3].add_point(&vertex[1]);
    face[3].add_point(&vertex[5]);
    face[3].add_point(&vertex[6]);

    face[4].add_segment(&segment[2] , P);
    face[4].add_segment(&segment[10], P);
    face[4].add_segment(&segment[6] , P);
    face[4].add_segment(&segment[11], P);

    face[4].add_point(&vertex[3]);
    face[4].add_point(&vertex[2]);
    face[4].add_point(&vertex[6]);
    face[4].add_point(&vertex[7]);

    face[5].add_segment(&segment[3] , P);
    face[5].add_segment(&segment[11], P);
    face[5].add_segment(&segment[7] , P);
    face[5].add_segment(&segment[8] , P);

    face[5].add_point(&vertex[0]);
    face[5].add_point(&vertex[3]);
    face[5].add_point(&vertex[7]);
    face[5].add_point(&vertex[4]);
}

/*! \brief set the touch lists
 */
void CuboidShape::set_touch_lists()
{
    // vertex_face
    vertex_face[0][0] = 0;
    vertex_face[0][1] = 2;
    vertex_face[0][2] = 5;

    vertex_face[1][0] = 0;
    vertex_face[1][1] = 2;
    vertex_face[1][2] = 3;

    vertex_face[2][0] = 0;
    vertex_face[2][1] = 3;
    vertex_face[2][2] = 4;

    vertex_face[3][0] = 0;
    vertex_face[3][1] = 4;
    vertex_face[3][2] = 5;

    vertex_face[4][0] = 1;
    vertex_face[4][1] = 2;
    vertex_face[4][2] = 5;

    vertex_face[5][0] = 1;
    vertex_face[5][1] = 2;
    vertex_face[5][2] = 3;

    vertex_face[6][0] = 1;
    vertex_face[6][1] = 3;
    vertex_face[6][2] = 4;

    vertex_face[7][0] = 1;
    vertex_face[7][1] = 4;
    vertex_face[7][2] = 5;
    
    // vertex_seg
    vertex_seg[0][0] = 0;
    vertex_seg[0][1] = 3;
    vertex_seg[0][2] = 8;

    vertex_seg[1][0] = 0;
    vertex_seg[1][1] = 1;
    vertex_seg[1][2] = 9;

    vertex_seg[2][0] = 1;
    vertex_seg[2][1] = 2;
    vertex_seg[2][2] = 10;

    vertex_seg[3][0] = 2;
    vertex_seg[3][1] = 3;
    vertex_seg[3][2] = 11;

    vertex_seg[4][0] = 4;
    vertex_seg[4][1] = 7;
    vertex_seg[4][2] = 8;

    vertex_seg[5][0] = 4;
    vertex_seg[5][1] = 5;
    vertex_seg[5][2] = 9;

    vertex_seg[6][0] = 5;
    vertex_seg[6][1] = 6;
    vertex_seg[6][2] = 10;

    vertex_seg[7][0] = 6;
    vertex_seg[7][1] = 7;
    vertex_seg[7][2] = 11;

    // seg_vertex
    seg_vertex[0][0] = 0;
    seg_vertex[0][1] = 1;

    seg_vertex[1][0] = 1;
    seg_vertex[1][1] = 2;

    seg_vertex[2][0] = 2;
    seg_vertex[2][1] = 3;

    seg_vertex[3][0] = 0;
    seg_vertex[3][1] = 3;

    seg_vertex[4][0] = 4;
    seg_vertex[4][1] = 5;

    seg_vertex[5][0] = 5;
    seg_vertex[5][1] = 6;

    seg_vertex[6][0] = 6;
    seg_vertex[6][1] = 7;

    seg_vertex[7][0] = 4;
    seg_vertex[7][1] = 7;

    seg_vertex[8][0] = 0;
    seg_vertex[8][1] = 4;

    seg_vertex[9][0] = 1;
    seg_vertex[9][1] = 5;

    seg_vertex[10][0] = 2;
    seg_vertex[10][1] = 6;

    seg_vertex[11][0] = 3;
    seg_vertex[11][1] = 7;

    // seg_face
    seg_face[0][0] = 0;
    seg_face[0][1] = 2;

    seg_face[1][0] = 0;
    seg_face[1][1] = 3;

    seg_face[2][0] = 0;
    seg_face[2][1] = 4;

    seg_face[3][0] = 0;
    seg_face[3][1] = 5;

    seg_face[4][0] = 1;
    seg_face[4][1] = 2;

    seg_face[5][0] = 1;
    seg_face[5][1] = 3;

    seg_face[6][0] = 1;
    seg_face[6][1] = 4;

    seg_face[7][0] = 1;
    seg_face[7][1] = 5;

    seg_face[8][0] = 2;
    seg_face[8][1] = 5;

    seg_face[9][0] = 2;
    seg_face[9][1] = 3;

    seg_face[10][0] = 3;
    seg_face[10][1] = 4;

    seg_face[11][0] = 4;
    seg_face[11][1] = 5;

    // face_vertex
    face_vertex[0][0] = 0;
    face_vertex[0][1] = 1;
    face_vertex[0][2] = 2;
    face_vertex[0][3] = 3;

    face_vertex[1][0] = 4;
    face_vertex[1][1] = 5;
    face_vertex[1][2] = 6;
    face_vertex[1][3] = 7;

    face_vertex[2][0] = 0;
    face_vertex[2][1] = 1;
    face_vertex[2][2] = 5;
    face_vertex[2][3] = 4;

    face_vertex[3][0] = 1;
    face_vertex[3][1] = 5;
    face_vertex[3][2] = 6;
    face_vertex[3][3] = 2;

    face_vertex[4][0] = 3;
    face_vertex[4][1] = 2;
    face_vertex[4][2] = 6;
    face_vertex[4][3] = 7;

    face_vertex[5][0] = 0;
    face_vertex[5][1] = 4;
    face_vertex[5][2] = 7;
    face_vertex[5][3] = 3;

    // face_seg
    face_seg[0][0] = 0;
    face_seg[0][1] = 1;
    face_seg[0][2] = 2;
    face_seg[0][3] = 3;

    face_seg[1][0] = 4;
    face_seg[1][1] = 5;
    face_seg[1][2] = 6;
    face_seg[1][3] = 7;

    face_seg[2][0] = 0;
    face_seg[2][1] = 9;
    face_seg[2][2] = 4;
    face_seg[2][3] = 8;

    face_seg[3][0] = 9;
    face_seg[3][1] = 5;
    face_seg[3][2] = 10;
    face_seg[3][3] = 1;

    face_seg[4][0] = 2;
    face_seg[4][1] = 10;
    face_seg[4][2] = 6;
    face_seg[4][3] = 11;

    face_seg[5][0] = 11;
    face_seg[5][1] = 3;
    face_seg[5][2] = 8;
    face_seg[5][3] = 7;
}

}
