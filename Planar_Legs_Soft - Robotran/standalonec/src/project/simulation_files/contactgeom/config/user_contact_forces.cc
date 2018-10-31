
#include "user_contact_forces.hh"
#include "ContactVolume.hh"

namespace ContactGeom{

// normal contribution
#define KP_N 1.0e10 ///< normal stiffnes coeff [N/(m^3)]
#define KD_N 1.0e2 ///< normal damping contribution [s/(m^3)]

#define MU_MAX 0.9   ///< maximal value for the friction coefficient [-]
#define BETA_TG 100.0 ///< tanh inner factor [-]

/// if negative, return 0, otherwise, return the input as output
inline double pos(double x) { return (x > 0.0) ? x : 0.0; }

/*! \brief get normal force
 * 
 * \param[in] shape basic shape on which the force is exerted
 * \param[in] other_shape other basic shape in contact with 'shape'
 * \param[in] volume penetration volume [m^3] (always positive)
 * \param[in] volume_dot penetration volume derivative [m^3/s]
 * \return normal force [N]
 *
 * The purpose of this function is to compute the normal force (scalar, supposed possitive)
 * resulting from the contact between the basic shape 'shape'and another basic shape 'other_shape'.
 * To this end, the penetration volume (i.e. intersection volume beween these two shapes) is provided,
 * together with its derivative.
 *
 * You can use 'shape->get_module()' to access this shape own module (defined in 'UserShapeModule.hh').
 * In a similar way, you can also use 'other_shape->get_module()' for the other shape.
 */
double normal_force(BasicShape *shape, BasicShape *other_shape, double volume, double volume_dot)
{
	return KP_N * volume * pos(1.0 + KD_N * volume_dot);
}

/*! \brief get tangential force norm on a point
 * 
 * \param[in] shape basic shape on which the force is exerted
 * \param[in] other_shape other basic shape in contact with 'shape'
 * \param[in] v relative tangetial speed on this point (norm, always positive) [m/s]
 * \param[in] F_N normal force applied on this point (norm, always positive) [N]
 * \return tangential force [N]
 *
 * The purpose of this function is to compute the tangential force (scalar, supposed positive).
 * To this end, the norm of the relative speed beween the two shapes involved ('shape' and 'other_shape')
 * is provided, as well as the normal force applied on this contact point.
 *
 * You can use 'shape->get_module()' to access this shape own module (defined in 'UserShapeModule.hh').
 * In a similar way, you can also use 'other_shape->get_module()' for the other shape.
 */
double tangentiel_force(BasicShape *shape, BasicShape *other_shape, double v, double F_N)
{
	return F_N * MU_MAX * tanh(BETA_TG * v);
}

}
