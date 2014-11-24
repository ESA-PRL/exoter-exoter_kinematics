/**\file ExoterWheelwalkingTypes.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date March 2014
* @version 1.0.
*/

#ifndef EXOTER_WHEELWALKING_TYPES_HPP
#define EXOTER_WHEELWALKING_TYPES_HPP

namespace exoter
{
    enum WheelwalkingMode {AXLE_BY_AXLE, SIDE_BY_SIDE, EVEN_ODD};
    enum WheelwalkingState {FIRST_AXLE = 0, SECOND_AXLE = 1, THIRD_AXLE = 2,
                            LEFT_SIDE = 0, RIGHT_SIDE = 1,
                            EVEN = 0, ODD = 1};
}

#endif
