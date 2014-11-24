#include "ExoterWheelwalkingControl.hpp"

#include "ExoterWheelwalkingKinematics.hpp"

#include <iostream>

using namespace exoter;

ExoterWheelwalkingControl::ExoterWheelwalkingControl(const double wheel_radius) : ExoterLocomotionControl(static_cast<unsigned int>(AXLE_BY_AXLE))
{
    kinematics = new ExoterWheelwalkingKinematics(AXLE_BY_AXLE, 0.05, wheel_radius);
}

ExoterWheelwalkingControl::~ExoterWheelwalkingControl()
{
    delete kinematics;
}

void ExoterWheelwalkingControl::selectNextGait() {
    switch(current_mode)
    {
    case AXLE_BY_AXLE:
        std::cout << "Switching to SIDE_BY_SIDE gait." << std::endl;
        selectMode(SIDE_BY_SIDE);
        break;
    case SIDE_BY_SIDE:
        std::cout << "Switching to EVEN_ODD gait." << std::endl;
        selectMode(EVEN_ODD);
        break;
    case EVEN_ODD:
        std::cout << "Switching to AXLE_BY_AXLE gait." << std::endl;
        selectMode(AXLE_BY_AXLE);
        break;
    }
}
