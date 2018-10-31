#include "Vector3D.hh"

#include <iostream>
#include <stdlib.h>

namespace ContactGeom{

/// return the index of the max of three values
inline int max_three(double x0, double x1, double x2) { return (x0 < x1) ? ((x1 < x2) ? 2 : 1) : ((x0 < x2) ? 2 : 0); }

/*! \brief constructor
 */
Vector3D::Vector3D()
{
    for(int i=0; i<3; i++)
    {
        vector[i] = 0.0;
    }
}

/*! \brief constructor
 * 
 * \param[in] x x value
 * \param[in] y y value
 * \param[in] z z value
 */
Vector3D::Vector3D(double x, double y, double z)
{
    vector[0] = x;
    vector[1] = y;
    vector[2] = z;
}

/*! \brief destructor
 */
Vector3D::~Vector3D()
{

}

/*! \brief get the orthogonal basis when one normalized vector is given
 * 
 * \pre vector 3D must be normalized
 * \param[out] b second basis vector (normalized)
 * \param[out] c third basis vector (normalized)
 */
void Vector3D::ortho_matrix_3D(Vector3D &b, Vector3D &c)
{
    int index;

    index = max_three( fabs(vector[0]), fabs(vector[1]), fabs(vector[2]) );

    switch (index)
    {
        case 0:
            b.set_x(-vector[1]/vector[0]);
            b.set_y(1.0);
            b.set_z(0.0);
            break;

        case 1:
            b.set_x(1.0);
            b.set_y(-vector[0]/vector[1]);
            b.set_z(0.0);
            break;

        case 2:
            b.set_x(1.0);
            b.set_y(0.0);
            b.set_z(-vector[0]/vector[2]);
            break;
    
        default:
            std::cout << "Error: unkonwn max three case !" << std::endl;
            exit(EXIT_FAILURE);
            break;
    }

    b.normalize();

    c = cross_vector_3D(*this, b);
    c.normalize();
}

}
