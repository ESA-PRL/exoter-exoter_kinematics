/**\file ExoterWheelwalkingTypes.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@tum.de
* @date February 2015
* @version 1.1
*/

#ifndef EXOTER_WHEELWALKING_TYPES_HPP
#define EXOTER_WHEELWALKING_TYPES_HPP

namespace exoter
{
    enum WheelwalkingMode {AXLE_BY_AXLE, SIDE_BY_SIDE, EVEN_ODD, SINGLE_WHEEL};
    enum WheelwalkingState {FIRST_AXLE = 0, SECOND_AXLE = 1, THIRD_AXLE = 2,
                            LEFT_SIDE = 0, RIGHT_SIDE = 1,
                            EVEN = 0, ODD = 1,
							FRONT_LEFT = 0, FRONT_RIGHT = 1, MIDDLE_LEFT = 2, MIDDLE_RIGHT = 3, REAR_LEFT = 4, REAR_RIGHT = 5};
}

#endif
