/*! 
 * \author Nicolas Van der Noot
 * \file RotMatrix.hh
 * \brief RotMatrix class
 */

#ifndef _ROT_MATRIX_HH_
#define _ROT_MATRIX_HH_

#include <cmath>
#include "Vector3D.hh"

namespace ContactGeom{

/*! \brief rotation matrix
 */
class RotMatrix
{
    public:
        RotMatrix();
        RotMatrix(double theta_x, double theta_y, double theta_z);
        RotMatrix(Vector3D const& omega_vec);
        ~RotMatrix();

        /// access element
        double get_comp(int i, int j) const { return matrix[i][j]; }

        /// set value
        void set_comp(int i, int j, double value) { matrix[i][j] = value; }

        /// reset
        void reset()
        {
            matrix[0][0] = 1.0;
            matrix[0][1] = 0.0;
            matrix[0][2] = 0.0;
            matrix[1][0] = 0.0;
            matrix[1][1] = 1.0;
            matrix[1][2] = 0.0;
            matrix[2][0] = 0.0;
            matrix[2][1] = 0.0;
            matrix[2][2] = 1.0;
        }

        /// return 1 if it is the identity matrix, 0 otherwise
        int check_identity() const
        {
            return ((matrix[0][0] == 1.0) & (matrix[0][1] == 0.0) & (matrix[0][2] == 0.0) & 
                    (matrix[1][0] == 0.0) & (matrix[1][1] == 1.0) & (matrix[1][2] == 0.0) & 
                    (matrix[2][0] == 0.0) & (matrix[2][1] == 0.0) & (matrix[2][2] == 1.0));
        }

    private:
        double matrix[3][3]; ///< matrix 3x3

        // functions prototypes
        void set_from_R1_R2_R3(double theta_x, double theta_y, double theta_z);
};

/*! \brief sum two matrices with the + operation
 * 
 * \param[in] a first matrix to sum
 * \param[in] b second matrix to sum 
 * \return sum of the two matrices
 */
inline RotMatrix operator+(RotMatrix const& a, RotMatrix const& b)
{
    RotMatrix result;

    for (int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            result.set_comp(i, j, a.get_comp(i,j) + b.get_comp(i,j));
        }
    }

    return result;
}

/*! \brief difference of two matrices with the - operation
 * 
 * \param[in] a first matrix
 * \param[in] b second matrix 
 * \return difference of the two matrices
 */
inline RotMatrix operator-(RotMatrix const& a, RotMatrix const& b)
{
    RotMatrix result;

    for (int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            result.set_comp(i, j, a.get_comp(i,j) - b.get_comp(i,j));
        }
    }

    return result;
}

/*! \brief return a rotation matrix being the identity matrix
 * 
 * \return requested matrix
 */
inline RotMatrix get_identity_rot_matrix()
{
    RotMatrix result;

    return result;
}

/*! \brief get the transposed of a matrix
 * 
 * \param[in] a matrix to be transposed
 * \return transposed matrix
 */
inline RotMatrix get_trans_rot_matrix(RotMatrix const& a)
{
    RotMatrix result;

    result.set_comp(0, 0, a.get_comp(0,0));
    result.set_comp(0, 1, a.get_comp(1,0));
    result.set_comp(0, 2, a.get_comp(2,0));
    result.set_comp(1, 0, a.get_comp(0,1));
    result.set_comp(1, 1, a.get_comp(1,1));
    result.set_comp(1, 2, a.get_comp(2,1));
    result.set_comp(2, 0, a.get_comp(0,2));
    result.set_comp(2, 1, a.get_comp(1,2));
    result.set_comp(2, 2, a.get_comp(2,2));

    return result;
}

/*! \brief multiplication of two matrices with the * operation
 * 
 * \param[in] a first matrix  (pre-multiplication)
 * \param[in] b second matrix (post-multiplication)
 * \return matrix product of the two matrices
 */
inline RotMatrix operator*(RotMatrix const& a, RotMatrix const& b)
{
    RotMatrix result;

    double a00, a01, a02, a10, a11, a12, a20, a21, a22;
    double b00, b01, b02, b10, b11, b12, b20, b21, b22;

    a00 = a.get_comp(0,0);
    a01 = a.get_comp(0,1);
    a02 = a.get_comp(0,2);
    a10 = a.get_comp(1,0);
    a11 = a.get_comp(1,1);
    a12 = a.get_comp(1,2);
    a20 = a.get_comp(2,0);
    a21 = a.get_comp(2,1);
    a22 = a.get_comp(2,2);

    b00 = b.get_comp(0,0);
    b01 = b.get_comp(0,1);
    b02 = b.get_comp(0,2);
    b10 = b.get_comp(1,0);
    b11 = b.get_comp(1,1);
    b12 = b.get_comp(1,2);
    b20 = b.get_comp(2,0);
    b21 = b.get_comp(2,1);
    b22 = b.get_comp(2,2);

    result.set_comp(0, 0, a00*b00 + a01*b10 + a02*b20);
    result.set_comp(0, 1, a00*b01 + a01*b11 + a02*b21);
    result.set_comp(0, 2, a00*b02 + a01*b12 + a02*b22);
    result.set_comp(1, 0, a10*b00 + a11*b10 + a12*b20);
    result.set_comp(1, 1, a10*b01 + a11*b11 + a12*b21);
    result.set_comp(1, 2, a10*b02 + a11*b12 + a12*b22);
    result.set_comp(2, 0, a20*b00 + a21*b10 + a22*b20);
    result.set_comp(2, 1, a20*b01 + a21*b11 + a22*b21);
    result.set_comp(2, 2, a20*b02 + a21*b12 + a22*b22);

    return result;
}

/*! \brief multiplication of a matrix by a vector with the * operation
 * 
 * \param[in] a matrix
 * \param[in] b vector
 * \return resutling vector product
 */
inline Vector3D operator*(RotMatrix const& a, Vector3D const&b)
{
    Vector3D result;
    double b0, b1, b2;

    b0 = b.get_comp(0);
    b1 = b.get_comp(1);
    b2 = b.get_comp(2);

    result.set_comp(0, a.get_comp(0,0)*b0 + a.get_comp(0,1)*b1 + a.get_comp(0,2)*b2);
    result.set_comp(1, a.get_comp(1,0)*b0 + a.get_comp(1,1)*b1 + a.get_comp(1,2)*b2);
    result.set_comp(2, a.get_comp(2,0)*b0 + a.get_comp(2,1)*b1 + a.get_comp(2,2)*b2);

    return result;
}

/*! \brief multiplication of the transposed of a matrix by a vector
 * 
 * \param[in] a matrix
 * \param[in] b vector
 * \return resutling vector product
 */
inline Vector3D product_trans_matrix_vector_3d(RotMatrix const& a, Vector3D const&b)
{
    Vector3D result;
    double b0, b1, b2;

    b0 = b.get_comp(0);
    b1 = b.get_comp(1);
    b2 = b.get_comp(2);

    result.set_comp(0, a.get_comp(0,0)*b0 + a.get_comp(1,0)*b1 + a.get_comp(2,0)*b2);
    result.set_comp(1, a.get_comp(0,1)*b0 + a.get_comp(1,1)*b1 + a.get_comp(2,1)*b2);
    result.set_comp(2, a.get_comp(0,2)*b0 + a.get_comp(1,2)*b1 + a.get_comp(2,2)*b2);

    return result;
}

}
#endif
