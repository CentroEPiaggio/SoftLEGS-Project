/*! 
 * \author Nicolas Van der Noot
 * \file Vector3D.hh
 * \brief Vector3D class
 */

#ifndef _VECTOR_3D_HH_
#define _VECTOR_3D_HH_

#include <math.h>
#ifndef STANDALONE
#if (_MSC_VER >= 1600)
#define __STDC_UTF_16__
#endif
#ifndef M_PI
#  define M_PI 3.141592653589793238
#endif
#endif

namespace ContactGeom{

/*! \brief 3D vector
 */
class Vector3D
{
    public:
        Vector3D();
        Vector3D(double x, double y, double z);
        ~Vector3D();

        void ortho_matrix_3D(Vector3D &b, Vector3D &c);

        /// access element
        double get_x() const { return vector[0]; }
        double get_y() const { return vector[1]; }
        double get_z() const { return vector[2]; }

        double get_comp(int i) const { return vector[i]; }

        /// set value
        void set_x(double value) { vector[0] = value; }
        void set_y(double value) { vector[1] = value; }
        void set_z(double value) { vector[2] = value; }

        void set_comp(int i, double value) { vector[i] = value; }

        /// vector norm
        double get_norm()        const { return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]); }
        double get_square_norm() const { return      vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2];  }

        /// normalize the 3D vector
        void normalize()
        {
            double norm = get_norm();

            if (norm > 0.0)
            {
                for(int i=0; i<3; i++)
                {
                    vector[i] /= norm;
                }
            }
        }

        /// set all the vector components to 0
        void reset()
        {
            for (int i=0; i<3; i++)
            {
                vector[i] = 0.0;
            }
        }

        /*! \brief set new values for the vector
         * 
         * \param[in] vec3D new vector to copy
         */
        void copy_vector(Vector3D const& vec3D)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] = vec3D.get_comp(i);
            }
        }

        /*! \brief set function with a =
         * 
         * \param[in] vec3D new position
         */
        void operator=(Vector3D const& vec3D)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] = vec3D.get_comp(i);
            }
        }

        /*! \brief increment the vector with the += operation
         * 
         * \param[in] a vector to increment
         */
        void operator+=(Vector3D const& a)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] += a.get_comp(i);
            }
        }

        /*! \brief decrement the vector with the -= operation
         * 
         * \param[in] a vector to decrement
         */
        void operator-=(Vector3D const& a)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] -= a.get_comp(i);
            }
        }

        /*! \brief multiply the vector with the *= operation
         * 
         * \param[in] a vector to multiply
         */
        void operator*=(Vector3D const& a)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] *= a.get_comp(i);
            }
        }

        /*! \brief divide the vector with the /= operation
         * 
         * \param[in] a vector to divide
         */
        void operator/=(Vector3D const& a)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] /= a.get_comp(i);
            }
        }

        /*! \brief divide the vector with the /= operation
         * 
         * \param[in] a constant to divide
         */
        void operator/=(double a)
        {
            for (int i=0; i<3; i++)
            {
                vector[i] /= a;
            }
        }

        /*! \brief get a copy of the current vector
         * 
         * \return copy of the current 3D vector
         */
        Vector3D get_copy()
        {
            Vector3D result(vector[0], vector[1], vector[2]);
            return result;
        }

    private:
        double vector[3]; ///< 3 components for the vector: x, y, z
};

/*! \brief sum two vectors with the + operation
 * 
 * \param[in] a first vector to sum
 * \param[in] b second vector to sum 
 * \return sum of the two vectors
 */
inline Vector3D operator+(Vector3D const& a, Vector3D const& b)
{
    Vector3D result;

    for (int i=0; i<3; i++)
    {
        result.set_comp(i, a.get_comp(i) + b.get_comp(i));
    }

    return result;
}

/*! \brief difference of two vectors with the - operation
 * 
 * \param[in] a first vector
 * \param[in] b second vector 
 * \return difference of the two vectors
 */
inline Vector3D operator-(Vector3D const& a, Vector3D const& b)
{
    Vector3D result;

    for (int i=0; i<3; i++)
    {
        result.set_comp(i, a.get_comp(i) - b.get_comp(i));
    }

    return result;
}

/*! \brief opposite of a vector
 * 
 * \param[in] a vector
 * \return opposite of the vector
 */
inline Vector3D operator-(Vector3D const& a)
{
    Vector3D result;

    for (int i=0; i<3; i++)
    {
        result.set_comp(i, -a.get_comp(i));
    }

    return result;
}

/*! \brief multiply a vector by a constant with the * operation
 * 
 * \param[in] a vector
 * \param[in] b constant used to multiply 
 * \return requested multiplied vector
 */
inline Vector3D operator*(Vector3D const& a, double const b)
{
    Vector3D result;

    for (int i=0; i<3; i++)
    {
        result.set_comp(i, a.get_comp(i) * b);
    }

    return result;
}

/*! \brief multiply a vector by a constant with the * operation
 * 
 * \param[in] a constant used to multiply
 * \param[in] b vector
 * \return requested multiplied vector
 */
inline Vector3D operator*(double const a, Vector3D const& b)
{
    Vector3D result;

    for (int i=0; i<3; i++)
    {
        result.set_comp(i, a * b.get_comp(i));
    }

    return result;
}

/*! \brief divide a vector by a constant with the / operation
 * 
 * \param[in] a vector
 * \param[in] b constant used to divide 
 * \return requested divided vector
 */
inline Vector3D operator/(Vector3D const& a, double const b)
{
    Vector3D result;

    for (int i=0; i<3; i++)
    {
        result.set_comp(i, a.get_comp(i) / b);
    }

    return result;
}

/*! \brief return a 3D vector with only zero values
 * 
 * \return requested vector
 */
inline Vector3D get_nul_vector_3D()
{
    Vector3D result(0.0, 0.0, 0.0);

    return result;
}

/*! \brief return a 3D vector initialized
 * 
 * \param[in] x x component
 * \param[in] y y component
 * \param[in] z z component
 * \return requested vector with x, y and z components
 */
inline Vector3D get_vector_3D(double x, double y, double z)
{
    Vector3D result(x, y, z);

    return result;
}

/*! \brief compute the scalar product of two 3D vectors
 * 
 * \param[in] a first vector
 * \param[in] b second vector
 * \return scalar product of the two vectors
 */
inline double scalar_vector_3D(Vector3D const& a, Vector3D const& b)
{
    double result = 0.0;

    for(int i=0; i<3; i++)
    {
        result += a.get_comp(i) * b.get_comp(i);
    }

    return result;
}

/*! \brief compute the time derivative of the scalar product of two 3D vectors
 * 
 * \param[in] a first vector
 * \param[in] b second vector
 * \param[in] a_dot first vector derivative
 * \param[in] b_dot second vector derivative
 * \return derivative of scalar product of the two vectors
 */
inline double scalar_vector_3D_dot(Vector3D const& a, Vector3D const& b, Vector3D const& a_dot, Vector3D const& b_dot)
{
    double result = 0.0;

    for(int i=0; i<3; i++)
    {
        result += a_dot.get_comp(i) * b.get_comp(i) + a.get_comp(i) * b_dot.get_comp(i);
    }

    return result;
}

/*! \brief compute the cross product of two 3D vectors
 * 
 * \param[in] a first vector
 * \param[in] b second vector
 * \return cross product of the two vectors
 */
inline Vector3D cross_vector_3D(Vector3D const& a, Vector3D const& b)
{
    Vector3D result( a.get_y()*b.get_z() - a.get_z()*b.get_y(),
                    -a.get_x()*b.get_z() + a.get_z()*b.get_x(),
                     a.get_x()*b.get_y() - a.get_y()*b.get_x());

    return result;
}

/*! \brief compute the time derivative of the cross product of two 3D vectors
 * 
 * \param[in] a first vector
 * \param[in] b second vector
 * \param[in] a_dot first vector derivative
 * \param[in] b_dot second vector derivative
 * \return derivative of the cross product of the two vectors
 */
inline Vector3D cross_vector_3D_dot(Vector3D const& a, Vector3D const& b, Vector3D const& a_dot, Vector3D const& b_dot)
{
    Vector3D result( a_dot.get_y()*b.get_z() + a.get_y()*b_dot.get_z() - a_dot.get_z()*b.get_y() - a.get_z()*b_dot.get_y(),
                    -a_dot.get_x()*b.get_z() - a.get_x()*b_dot.get_z() + a_dot.get_z()*b.get_x() + a.get_z()*b_dot.get_x(),
                     a_dot.get_x()*b.get_y() + a.get_x()*b_dot.get_y() - a_dot.get_y()*b.get_x() - a.get_y()*b_dot.get_x());

    return result;
}

/*! \brief compute the distance between two points
 * 
 * \param[in] a first point position
 * \param[in] b second point position
 * \return distance between points
 */
inline double get_dist_vectors_3D(Vector3D const& a, Vector3D const& b)
{
    Vector3D diff_vec;

    diff_vec = a - b;

    return diff_vec.get_norm();
}

/*! \brief compute the square of the distance between two points
 * 
 * \param[in] a first point position
 * \param[in] b second point position
 * \return square of the distance between points
 */
inline double get_square_dist_vectors_3D(Vector3D const& a, Vector3D const& b)
{
    Vector3D diff_vec;

    diff_vec = a - b;

    return diff_vec.get_square_norm();
}

/*! \brief compute the middle of two 3D positions
 * 
 * \param[in] a first 3D position
 * \param[in] b second 3D position
 * \return requested mid-position
 */
inline Vector3D mid_vectors_3D(Vector3D const& a, Vector3D const& b)
{
    return 0.5 * (a + b); 
}

}
#endif
