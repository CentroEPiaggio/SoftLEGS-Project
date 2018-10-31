/*! 
 * \author Nicolas Van der Noot
 * \file CuboidShape.hh
 * \brief CuboidShape class
 */

#ifndef _CUBOID_SHAPE_HH_
#define _CUBOID_SHAPE_HH_

#include "BasicShape.hh"
#include "Point3D.hh"
#include "Segment3D.hh"
#include "Rectangle3D.hh"

namespace ContactGeom{

#define MAX_INTERSECT_CUBOID 144

/*! \brief basic shape: cuboid
 */
class CuboidShape: public BasicShape
{
    public:
        CuboidShape(double d, double w, double h, Vector3D const& rel_position, RotMatrix const& rel_matrix, int rigid_flag, GeometryShape *parent_shape);
        virtual ~CuboidShape();

        virtual void update_check();

        virtual void update_kinematics(Vector3D const& parent_P, Vector3D const& parent_V, Vector3D const& parent_OM, RotMatrix const& parent_R);

        // update the check_flag flag
        virtual void update_check_flag() { check_flag = 1; }
        virtual int point_inside(Vector3D const& point);

        virtual void update_check_min_max();
        void update_vertex_pos();

        virtual int update_F_T_basic(BasicShape *other_shape);

        void set_vertex_seg_face();
        void set_touch_lists();
        void update_vertex_seg_face();

        Point3D& get_vertex(int i) { return vertex[i]; }
        Segment3D& get_segment(int i) { return segment[i]; }
        Rectangle3D& get_face(int i) { return face[i]; }

        Point3D* get_vertex_ptr(int i) { return &vertex[i]; }
        Segment3D* get_segment_ptr(int i) { return &segment[i]; }
        Rectangle3D* get_face_ptr(int i) { return &face[i]; }

        int get_vertex_face(int i, int j) const { return vertex_face[i][j]; }
        int get_seg_face(int i, int j) const { return seg_face[i][j]; }
        int get_flag_vertex_seg_face() const { return flag_vertex_seg_face; }

        void set_flag_vertex_seg_face(int i) { flag_vertex_seg_face = i; }

        void contact_face_vertices_clear();

        std::vector<Point3D*>& get_contact_face_vertices(int i) { return contact_face_vertices[i]; }
        Point3D* get_contact_face_vertices(int i, int j) { return contact_face_vertices[i][j]; }
        int get_contact_face_vertices_size(int i) const { return contact_face_vertices[i].size(); }

        /// add a new contact point to a face
        void add_contact_face_vertices(int i, Point3D *new_point) { contact_face_vertices[i].push_back(new_point); }

        void compute_h_hdot(double out_seg, double diff_center, double diff_center_dot, double radius, double &h, double &h_dot);
        void order_h_hdot(double ha, double hb, double ha_dot, double hb_dot, double &h1, double &h2, double &h1_dot, double &h2_dot);
        void order_h_hdot(double hx, double hy, double hz, double hx_dot, double hy_dot, double hz_dot,
            double &h1, double &h2, double &h3, double &h1_dot, double &h2_dot, double &h3_dot);

    private:
        double d; ///< cuboid depth (along the x axis) [m]
        double w; ///< cuboid width (along the y axis) [m]
        double h; ///< cuboid height (along the z axis) [m]
        double max_dim; ///< max between d, w and h

        double semi_d; ///< cuboid semi depth [m]
        double semi_w; ///< cuboid semi width [m]
        double semi_h; ///< cuboid semi height [m]

        int flag_rel_identity;    ///< 1 if rel_matrix is the identity matrix, 0 otherwise
        int flag_vertex_seg_face; ///< 1 if 'update_vertex_seg_face' performed, 0 otherwise

        /*! \brief relative rotation matrix from the RigidShape to the current CuboidShape [-]
         *
         * This rotation matrix is expressed in the frame local to the body
         * (i.e. the one given by the ExtForce sensor of the corresponding RigidShape).
         */
        RotMatrix rel_matrix;

        RotMatrix R; ///< absolute rotation matrix (in the inertial frame) [-]
        RotMatrix R_T; ///< transpose of 'R'

        // elements
        Vector3D rel_vertex[8];          ///< position of the 8 vertices in the cuboid frame
        Vector3D rel_vertex_inertial[8]; ///< 'rel_vertex' in the inertial frame
        Vector3D vertex_pos[8];          ///< vertex position in the inertial frame

        Point3D vertex[8];      ///< cuboid vertices
        Segment3D segment[12];  ///< cuboid segments
        Rectangle3D face[6];    ///< cuboid faces

        Point3D intersect_points_storage[MAX_INTERSECT_CUBOID]; ///< points delimiting the contact between the two shapes
        std::vector<Point3D*> intersect_points; ///< 'intersect_points_alloc', but with pointers

        /*! \brief points at the edge of the surface
         *
         * each index of the tab corresponds to one of the six cuboid face, 
         *      except the last one (6), corresponding to the contact surface
         */
        std::vector<Point3D*> contact_face_vertices[7]; 

        // touch table
        int vertex_face[8][3];
        int vertex_seg[8][3];

        int seg_vertex[12][2];
        int seg_face[12][2];

        int face_vertex[6][4];
        int face_seg[6][4];

        // functions prototypes
        int update_F_T_plane(BasicShape *other_shape);
        int update_F_T_sphere(BasicShape *other_shape);
        int update_F_T_cuboid(BasicShape *other_shape);
};

}
#endif
