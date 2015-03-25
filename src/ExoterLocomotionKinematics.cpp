#include "ExoterLocomotionKinematics.hpp"

using namespace exoter;

ExoterLocomotionKinematics::ExoterLocomotionKinematics(const double wheel_radius, const unsigned int mode) : mode(mode), wheel_radius(wheel_radius)
{
    kinematic_model = new ExoterKinematicModel(wheel_radius);
}

ExoterLocomotionKinematics::~ExoterLocomotionKinematics()
{
    delete kinematic_model;
}

void ExoterLocomotionKinematics::setMode(const unsigned int mode)
{
    this->mode = mode;
    initMode();
}
