#include "RotMatrix.hh"

namespace ContactGeom{

/*! \brief constructor
 */
RotMatrix::RotMatrix()
{
    reset();
}

/*! \brief constructor
 * 
 * \param[in] theta_x angle along the x axis [rad]
 * \param[in] theta_y angle along the y axis [rad]
 * \param[in] theta_z angle along the z axis [rad]
 *
 * The angles describing the rotation matrix are three consecutive angles
 * laong the x (R1), y (R2) and z (R3) axes
 */
RotMatrix::RotMatrix(double theta_x, double theta_y, double theta_z)
{
    set_from_R1_R2_R3(theta_x, theta_y, theta_z);
}

/*! \brief constructor to get the rotation matrix tilde
 * 
 * \param[in] vec vector defining the tilde matrix
 */
RotMatrix::RotMatrix(Vector3D const& vec)
{
    matrix[0][0] =  0.0;
    matrix[0][1] = -vec.get_z();
    matrix[0][2] =  vec.get_y();

    matrix[1][0] =  vec.get_z();
    matrix[1][1] =  0.0;
    matrix[1][2] = -vec.get_x();

    matrix[2][0] = -vec.get_y();
    matrix[2][1] =  vec.get_x();
    matrix[2][2] =  0.0;
}

/*! \brief destructor
 */
RotMatrix::~RotMatrix()
{

}

/*! \brief set the value of a rotation matrix, with rotational angles R1-R2-R3
 * 
 * \param[in] theta_x agnle along the x axis [rad]
 * \param[in] theta_y agnle along the y axis [rad]
 * \param[in] theta_z agnle along the z axis [rad]
 *
 * The angles describing the rotation matrix are three consecutive angles
 * laong the x (R1), y (R2) and z (R3) axes
 */
void RotMatrix::set_from_R1_R2_R3(double theta_x, double theta_y, double theta_z)
{
    double c_x, c_y, c_z;
    double s_x, s_y, s_z;

    c_x = cos(theta_x);
    c_y = cos(theta_y);
    c_z = cos(theta_z);

    s_x = sin(theta_x);
    s_y = sin(theta_y);
    s_z = sin(theta_z);

    matrix[0][0] =  c_y*c_z;
    matrix[0][1] =  s_x*s_y*c_z + c_x*s_z;
    matrix[0][2] = -c_x*s_y*c_z + s_x*s_z;

    matrix[1][0] = -c_y*s_z;
    matrix[1][1] = -s_x*s_y*s_z + c_x*c_z;
    matrix[1][2] =  c_x*s_y*s_z + s_x*c_z;

    matrix[2][0] =  s_y;
    matrix[2][1] = -s_x*c_y;
    matrix[2][2] =  c_x*c_y;
}

}
