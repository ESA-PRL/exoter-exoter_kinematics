#include "ExoterLocomotionControl.hpp"
#include <iostream>

using namespace exoter;

ExoterLocomotionControl::ExoterLocomotionControl(unsigned int mode) : current_mode(mode), mode_transition(true), stop_motion(false), speed(0.0d)
{
    positions.assign(EXOTER_JOINT_DOF, 0.0d);
    velocities.assign(EXOTER_JOINT_DOF, 0.0d);
    position_readings_old.assign(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS, 0.0d);
    velocity_readings_old.assign(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS, 0.0d);
}

ExoterLocomotionControl::~ExoterLocomotionControl()
{
}

void ExoterLocomotionControl::getJointCommands(std::vector<double>& position_commands, std::vector<double>& velocity_commands)
{
    if (mode_transition)
    {
        if (checkTargetJointPositionsReached())
        {
            mode_transition = false;
            std::cout << "Transition complete. Commence driving." << std::endl;
        }
        else
        {
            kinematics->computeConfigChangeJointCommands(walking_joints_status, positions, velocities, position_commands, velocity_commands);
            return;
        }
    }

	if (stop_motion)
	{
        position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());
        velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS, 0.0d);
	}
	else
	{
	    kinematics->computeMovementJointCommands(speed, walking_joints_status, positions, velocities, position_commands, velocity_commands);
	}
}

void ExoterLocomotionControl::setNewJointReadings(const std::vector<double>& position_readings, const std::vector<double>& velocity_readings)
{
    positions = assemblePositionVector(position_readings);
    velocities = assembleVelocityVector(velocity_readings);
    position_readings_old = position_readings;
    velocity_readings_old = velocity_readings;
}

void ExoterLocomotionControl::setSpeed(const double speed)
{
    this->speed = speed;
}

void ExoterLocomotionControl::stopMotion()
{
	stop_motion = true;
}

void ExoterLocomotionControl::startMotion()
{
	stop_motion = false;
}

void ExoterLocomotionControl::initJointConfiguration()
{
    mode_transition = true;
    std::cout << "Transitioning to initial configuration..." << std::endl;	
}

void ExoterLocomotionControl::selectMode(const unsigned int mode)
{
    current_mode = mode;
    kinematics->setMode(mode);

	initJointConfiguration();
}

bool ExoterLocomotionControl::checkTargetJointPositionsReached()
{
    std::vector<double> joint_positions(positions.begin() + NUMBER_OF_PASSIVE_JOINTS, positions.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS);
    std::vector<double> target_joint_positions = kinematics->getConfigChangeTargetJointPositions();

    for (unsigned int i = 0; i < joint_positions.size(); i++)
    {
		if (!walking_joints_status[i])
			continue;

        if (std::abs(joint_positions[i] - target_joint_positions[i]) > MAX_POS_OFFSET)
            return false;
    }

    return true;
}

std::vector<double> ExoterLocomotionControl::assemblePositionVector(const std::vector<double> position_readings)
{
    std::vector<double> positions;

    positions.insert(positions.end(), position_readings.begin(), position_readings.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS - 1);
//    positions.push_back(0.0d);  // Added temporarily for immovable left rear walking joint.
    positions.insert(positions.end(), position_readings.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS - 1, position_readings.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS);

    // Rolling angles as difference between new and old driving joint position minus change of ground contact angle (~= change of wheel walking angle)
    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        positions.push_back(position_readings[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i] - position_readings_old[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i]
        + position_readings[NUMBER_OF_PASSIVE_JOINTS + i] - position_readings_old[NUMBER_OF_PASSIVE_JOINTS + i]);

    // Slip currently assumed as zero
    positions.insert(positions.end(), NUMBER_OF_WHEELS * SLIP_VECTOR_SIZE, 0.0d);

    // Contact angles - currently assumed so that ground is parallel to rover base plate
    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        positions.push_back(-position_readings[NUMBER_OF_PASSIVE_JOINTS + i]);  // Contact angle set to the negative wheel walking angle

    return positions;
}

std::vector<double> ExoterLocomotionControl::assembleVelocityVector(const std::vector<double> velocity_readings)
{
    std::vector<double> velocities;

    velocities.insert(velocities.end(), velocity_readings.begin(), velocity_readings.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS - 1);
//    velocities.push_back(0.0d);  // Added temporarily for immovable left rear walking joint.
    velocities.insert(velocities.end(), velocity_readings.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS - 1, velocity_readings.begin() + NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS);
    
    // Rolling rates as difference between driving joint rates and contact angle rates (~= wheel walking joint rate)
    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        velocities.push_back(velocity_readings[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i] + velocity_readings[NUMBER_OF_PASSIVE_JOINTS + i]);

    // Slip rates currently assumed as zero
    velocities.insert(velocities.end(), NUMBER_OF_WHEELS * SLIP_VECTOR_SIZE, 0.0d);

    // Contact angle velocities - currently assumed so that ground stays parallel to rover base plate
    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        velocities.push_back(-velocity_readings[NUMBER_OF_PASSIVE_JOINTS + i]);  // Contact angle rate set to the negative wheel walking joint rate

    return velocities;
}
