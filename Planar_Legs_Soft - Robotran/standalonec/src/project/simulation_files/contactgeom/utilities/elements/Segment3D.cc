#include "Segment3D.hh"
#include <iostream>

namespace ContactGeom{

inline double square(double x) { return x*x; }
inline int is_in_range(double x, double x_min, double x_max) { return ((x_min <= x) && (x <= x_max)); }

/*! \brief constructor
 */
Segment3D::Segment3D()
{
    nb_points = 0;
}

/*! \brief constructor
 *
 * \pre point_A and point_B must already be allocated
 * \param[in] point_A first extremity point
 * \param[in] point_B second extremity point
 */
Segment3D::Segment3D(Point3D *point_A, Point3D *point_B)
{
    renew_seg(point_A, point_B);
}

/*! \brief destructor
 */
Segment3D::~Segment3D()
{

}

/*! \brief renew the segment
 *
 * \pre point_A and point_B must already be allocated
 * \param[in] point_A first extremity point
 * \param[in] point_B second extremity point
 */
void Segment3D::renew_seg(Point3D *point_A, Point3D *point_B)
{
    nb_points = 2;

    point[0] = point_A;
    point[1] = point_B;

    update_kinematics();
}

/*! \brief add a new point
 * 
 * \param[in] new_point new point to add
 */
void Segment3D::add_point(Point3D *new_point)
{
    switch (nb_points)
    {
        case 0:
            point[0] = new_point;
            nb_points++;
            break;

        case 1:
            point[1] = new_point;
            nb_points++;
            break;
    
        default:
            std::cout << "Warning: already two points in the segment !" << std::endl;
            break;
    }
}

/*! \brief update the kinematics
 */
void Segment3D::update_kinematics()
{
    diff_vec     = point[1]->get_point()     - point[0]->get_point();
    diff_vec_dot = point[1]->get_point_dot() - point[0]->get_point_dot();

    diff_vec_norm = diff_vec.get_norm();

    if (diff_vec_norm > 0.0)
    {
        diff_vec_norm_dot = scalar_vector_3D(diff_vec, diff_vec_dot) / diff_vec_norm;
    }   
}

/*! \brief compute the intersection point with a given plane
 * 
 * \param[in] normal_vec vector normal to the plane
 * \param[in] normal_vec_dot derivative of 'normal_vec'
 * \param[in] d d value in equation 'ax + by + cz + d = 0'
 * \param[in] d_dot derivative of 'd'
 * \param[in] flag_z_norm 1 if normal in +z direction, 0 otherwise
 * \param[out] result_point intersection point (if found)
 * \return 1 if contact found, 0 otherwise
 */
int Segment3D::intersect_plane(Vector3D const& normal_vec, Vector3D const& normal_vec_dot, double d, double d_dot, int flag_z_norm, Point3D &result_point)
{
    double num, num_dot;
    double denom, denom_dot;
    double lambda, lambda_dot;

    // denominator
    if (flag_z_norm)
    {
        denom = diff_vec.get_z();
    }
    else
    {
        denom = scalar_vector_3D(normal_vec, diff_vec);
    }

    // segment parallel to the plane
    if (!denom)
    {
        return 0;
    }

    // numerator
    if (flag_z_norm)
    {
        num = point[0]->get_point().get_z() + d;
    }
    else
    {
        num = scalar_vector_3D(normal_vec, point[0]->get_point()) + d;
    }

    lambda = - num / denom;

    if (!is_in_range(lambda, 0.0, 1.0))
    {
        return 0;
    }

    // derivatives
    if (flag_z_norm)
    {
        denom_dot = diff_vec_dot.get_z();
        num_dot   = point[0]->get_point_dot().get_z() + d_dot;
    }
    else
    {
        denom_dot = scalar_vector_3D_dot(normal_vec, diff_vec, normal_vec_dot, diff_vec_dot);
        num_dot   = scalar_vector_3D_dot(normal_vec, point[0]->get_point(), normal_vec_dot, point[0]->get_point_dot()) + d_dot;
    }

    lambda_dot = - (num_dot*denom - num*denom_dot) / square(denom);

    result_point.update_kinematics(point[0]->get_point()     + lambda * diff_vec,
                                   point[0]->get_point_dot() + lambda_dot * diff_vec + lambda * diff_vec_dot);

    return 1; // intersection found
}

/*! \brief compute the intersection point with a given sphere
 * 
 * \param[in] center center of the sphere
 * \param[in] center_dot derivative of the center of the sphere
 * \param[in] radius radius of the sphere
 * \param[out] result_point_1 first intersection point (if found)
 * \param[out] result_point_2 second intersection point (if found)
 * \return number of contact points found
 */
int Segment3D::intersect_sphere(Vector3D const& center, Vector3D const& center_dot, double radius, Point3D &result_point_1, Point3D &result_point_2)
{
    double alpha, beta, gamma;
    double alpha_dot, beta_dot, gamma_dot;
    double expr, expr_sqrt, expr_2;
    double square_R, square_alpha;
    double lambda, lambda_1, lambda_2;
    double lambda_dot, lambda_1_dot, lambda_2_dot;
    Vector3D diff_0_c, diff_0_c_dot;

    expr_sqrt = 0.0;
    lambda   = 0.0;
    lambda_1 = 0.0;
    lambda_2 = 0.0;
    lambda_1_dot = 0.0;
    lambda_2_dot = 0.0;

    diff_0_c = point[0]->get_point() - center;

    square_R = square(radius);

    alpha = diff_vec.get_square_norm();
    beta = scalar_vector_3D(diff_vec, diff_0_c);
    gamma = diff_0_c.get_square_norm() - square_R;

    expr = beta*beta - alpha*gamma;

    // no intersection possible
    if (expr < 0.0)
    {
        return 0;
    }
    
    // only one intersection possible
    if (!expr)
    {
        // safety
        if (!alpha)
        {
            return 0;
        }
        
        lambda = -beta / alpha;

        if (!is_in_range(lambda, 0.0, 1.0))
        {
            return 0;
        }
    }
    else // two intersections possible
    {
        expr_sqrt = sqrt(expr);

        if (!alpha)
        {
            // safety
            if (!beta)
            {
                return 0;
            }

            lambda_1 = -gamma / (2.0*beta);
            lambda_2 = -1.0;
        }
        else
        {
            lambda_1 = (-beta - expr_sqrt) / alpha;
            lambda_2 = (-beta + expr_sqrt) / alpha;
        }

        if ( (!is_in_range(lambda_1, 0.0, 1.0)) && (!is_in_range(lambda_2, 0.0, 1.0)) )
        {
            return 0;
        }
    }

    // derivatives
    diff_0_c_dot = point[0]->get_point_dot() - center_dot;

    alpha_dot = 2.0 * scalar_vector_3D(diff_vec, diff_vec_dot);
    beta_dot = scalar_vector_3D_dot(diff_vec, diff_0_c, diff_vec_dot, diff_0_c_dot);
    gamma_dot = 2.0 * scalar_vector_3D(diff_0_c, diff_0_c_dot);

    square_alpha = square(alpha);

    // one point found: first case
    if (!expr)
    {
        lambda_dot = -(beta_dot*alpha - beta*alpha_dot) / square_alpha;

        result_point_1.update_kinematics(point[0]->get_point()     + lambda * diff_vec,
                                         point[0]->get_point_dot() + lambda_dot * diff_vec + lambda * diff_vec_dot);

        result_point_2.clear();

        return 1;
    }

    // one point found: second case
    if (!alpha)
    {
        lambda_1_dot = -0.5 * (gamma_dot*beta - gamma*beta_dot) / square(beta);

        result_point_1.update_kinematics(point[0]->get_point()     + lambda_1 * diff_vec,
                                         point[0]->get_point_dot() + lambda_1_dot * diff_vec + lambda_1 * diff_vec_dot);

        result_point_2.clear();

        return 1;
    }

    expr_2 = (2.0*beta*beta_dot - alpha_dot*gamma - alpha*gamma_dot) / (2.0 * expr_sqrt);

    // first point derivative lambda
    if (is_in_range(lambda_1, 0.0, 1.0))
    {
        lambda_1_dot = ((-beta_dot - expr_2)*alpha - (-beta - expr_sqrt)*alpha_dot) / square_alpha;
    }

    // second point derivative lambda
    if (is_in_range(lambda_2, 0.0, 1.0))
    {
        lambda_2_dot = ((-beta_dot + expr_2)*alpha - (-beta + expr_sqrt)*alpha_dot) / square_alpha;
    }

    if (is_in_range(lambda_1, 0.0, 1.0))
    {
        result_point_1.update_kinematics(point[0]->get_point()     + lambda_1 * diff_vec,
                                         point[0]->get_point_dot() + lambda_1_dot * diff_vec + lambda_1 * diff_vec_dot);

        if (is_in_range(lambda_2, 0.0, 1.0))
        {
            result_point_2.update_kinematics(point[0]->get_point()     + lambda_2 * diff_vec,
                                             point[0]->get_point_dot() + lambda_2_dot * diff_vec + lambda_2 * diff_vec_dot);

            return 2;
        }
        else
        {
            result_point_2.clear();

            return 1;
        }       
    }
    else
    {
        if (is_in_range(lambda_2, 0.0, 1.0))
        {
            result_point_1.update_kinematics(point[0]->get_point()     + lambda_2 * diff_vec,
                                             point[0]->get_point_dot() + lambda_2_dot * diff_vec + lambda_2 * diff_vec_dot);

            result_point_2.clear();

            return 1;
        }
        else // safety, but normally never reached
        {
            return 0;
        }
    }
}

}
