#include "ExoterEgressControl.hpp"

#include "ExoterEgressKinematics.hpp"
#include <iostream>

using namespace exoter_kinematics;

ExoterEgressControl::ExoterEgressControl(const double wheel_radius) : ExoterLocomotionControl(static_cast<unsigned int>(NOMINAL))
{
    kinematics = new ExoterEgressKinematics(wheel_radius, NOMINAL);
}

ExoterEgressControl::~ExoterEgressControl()
{
}

void ExoterEgressControl::setHeightOffset(const double height_offset)    // Input between 0 and 1. 0 means no offset (nominal position), 1 means maximum offset.
{
    if (((ExoterEgressKinematics*) kinematics)->getHeightOffset() != height_offset && (current_mode == LOWER_COG_FW || current_mode == LOWER_COG_BW))
    {
        mode_transition = true;
        std::cout << "Height offset changed, is now " << height_offset << ".Transitioning to initial configuration..." << std::endl;
        ((ExoterEgressKinematics*) kinematics)->setHeightOffset(height_offset);
    }
}

void ExoterEgressControl::selectNextNominalEgressMode()
{
    switch (current_mode)
    {
    case STOWED:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    case LOWER_COG_FW:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    case STEP_DOWN_FW:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    case LOWER_COG_BW:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    case STEP_DOWN_BW:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    }
}

void ExoterEgressControl::selectNextForwardEgressMode()
{
    switch (current_mode)
    {
    case STOWED:
        std::cout << "Switching to LOWER_COG_FW mode." << std::endl;
        selectMode(LOWER_COG_FW);
        break;
    case NOMINAL:
        std::cout << "Switching to LOWER_COG_FW mode." << std::endl;
        selectMode(LOWER_COG_FW);
        break;
    case LOWER_COG_FW:
        std::cout << "Switching to STEP_DOWN_FW mode." << std::endl;
        selectMode(STEP_DOWN_FW);
        break;
    case STEP_DOWN_FW:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    case LOWER_COG_BW:
        std::cout << "Switching to LOWER_COG_FW mode." << std::endl;
        selectMode(LOWER_COG_FW);
        break;
    case STEP_DOWN_BW:
        std::cout << "Switching to LOWER_COG_FW mode." << std::endl;
        selectMode(LOWER_COG_FW);
        break;
    }
}

void ExoterEgressControl::selectNextBackwardEgressMode()
{
    switch (current_mode)
    {
    case STOWED:
        std::cout << "Switching to LOWER_COG_BW mode." << std::endl;
        selectMode(LOWER_COG_BW);
        break;
    case NOMINAL:
        std::cout << "Switching to LOWER_COG_BW mode." << std::endl;
        selectMode(LOWER_COG_BW);
        break;
    case LOWER_COG_FW:
        std::cout << "Switching to LOWER_COG_BW mode." << std::endl;
        selectMode(LOWER_COG_BW);
        break;
    case STEP_DOWN_FW:
        std::cout << "Switching to LOWER_COG_BW mode." << std::endl;
        selectMode(LOWER_COG_BW);
        break;
    case LOWER_COG_BW:
        std::cout << "Switching to STEP_DOWN_BW mode." << std::endl;
        selectMode(STEP_DOWN_BW);
        break;
    case STEP_DOWN_BW:
        std::cout << "Switching to NOMINAL mode." << std::endl;
        selectMode(NOMINAL);
        break;
    }
}
