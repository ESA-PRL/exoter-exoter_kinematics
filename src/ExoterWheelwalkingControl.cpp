#include "ExoterWheelwalkingControl.hpp"

#include "ExoterWheelwalkingKinematics.hpp"

#include <iostream>

using namespace exoter_kinematics;

ExoterWheelwalkingControl::ExoterWheelwalkingControl(const double wheel_radius) : ExoterLocomotionControl(static_cast<unsigned int>(AXLE_BY_AXLE))
{
    kinematics = new ExoterWheelwalkingKinematics(AXLE_BY_AXLE, wheel_radius);
}

ExoterWheelwalkingControl::~ExoterWheelwalkingControl()
{
    delete kinematics;
}

void ExoterWheelwalkingControl::selectMode(const unsigned int mode)
{
    bool valid_gait = true;

    switch(mode)
    {
    case AXLE_BY_AXLE:
        std::cout << "Switching to AXLE_BY_AXLE gait." << std::endl;
        break;
    case SIDE_BY_SIDE:
        std::cout << "Switching to SIDE_BY_SIDE gait." << std::endl;
        break;
    case EVEN_ODD:
	std::cout << "Switching to EVEN_ODD gait." << std::endl;
	break;
    case SINGLE_WHEEL:
        std::cout << "Switching to SINGLE_WHEEL gait." << std::endl;
	break;
    case NORMAL_DRIVING:
        std::cout << "Switching to NORMAL_DRIVING gait." << std::endl;
	break;
    default:
	std::cout << "Invalid gait selected." << std::endl;
	valid_gait = false;
	break;
    }

    if (valid_gait)
    {
	current_mode = mode;
	kinematics->setMode(mode);
	mode_transition = true;
	std::cout << "Transitioning to initial configuration." << std::endl;
    }
}

void ExoterWheelwalkingControl::selectNextGait()
{
    switch(current_mode)
    {
    case AXLE_BY_AXLE:
        selectMode(SIDE_BY_SIDE);
        break;
    case SIDE_BY_SIDE:
        selectMode(EVEN_ODD);
        break;
    case EVEN_ODD:
	selectMode(SINGLE_WHEEL);
	break;
    case SINGLE_WHEEL:
        selectMode(NORMAL_DRIVING);
	break;
    case NORMAL_DRIVING:
        selectMode(AXLE_BY_AXLE);
	break;
    }
}

void ExoterWheelwalkingControl::setOffsetSpeed(const double offset_speed)
{
    dynamic_cast<ExoterWheelwalkingKinematics*>(kinematics)->setOffsetSpeed(offset_speed);
}

void ExoterWheelwalkingControl::setStepLength(const double step_length)
{
    dynamic_cast<ExoterWheelwalkingKinematics*>(kinematics)->setStepLength(step_length);
}
