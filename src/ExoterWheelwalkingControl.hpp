/**\file ExoterWheelwalkingControl.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date March 2014
* @version 1.0.
*/

#ifndef EXOTER_WHEELWALKING_CONTROL_HPP
#define EXOTER_WHEELWALKING_CONTROL_HPP

/* ExoTeR */
#include "Configuration.hpp"
#include "ExoterLocomotionControl.hpp"
#include "ExoterWheelwalkingTypes.hpp"

namespace exoter_kinematics
{
    class ExoterWheelwalkingControl : public ExoterLocomotionControl
    {
    public:
        ExoterWheelwalkingControl(const double wheel_radius);
        ~ExoterWheelwalkingControl();
        void selectNextGait();
        void selectMode(const unsigned int mode);
        void setOffsetSpeed(const double offset_speed);
        void setStepLength(const double step_length);
    private:
        std::vector<double> getTargetJointPositions();
    };
}

#endif
