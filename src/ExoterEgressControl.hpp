/**\file ExoterEgressControl.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date March 2014
* @version 1.0.
*/

#ifndef EXOTER_EGRESS_CONTROL_HPP
#define EXOTER_EGRESS_CONTROL_HPP

/* ExoTeR */
#include "Configuration.hpp"
#include "ExoterEgressTypes.hpp"
#include "ExoterLocomotionControl.hpp"

namespace exoter
{
    class ExoterEgressControl : public ExoterLocomotionControl
    {
    public:
        ExoterEgressControl(const double wheel_radius);
        ~ExoterEgressControl();

        void selectNextNominalEgressMode();
        void selectNextForwardEgressMode();
        void selectNextBackwardEgressMode();
        void setHeightOffset(const double height_offset);
    private:
    };
}

#endif
