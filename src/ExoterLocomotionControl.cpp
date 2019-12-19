#include "ExoterLocomotionControl.hpp"
#include <iostream>

using namespace exoter_kinematics;

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
    positions.resize(EXOTER_JOINT_DOF + NUMBER_OF_WHEELS);
    velocities.resize(EXOTER_JOINT_DOF + NUMBER_OF_WHEELS);

    positions[PASSIVE_LEFT] = position_readings[0];
    positions[PASSIVE_RIGHT] = position_readings[1];
    positions[PASSIVE_REAR] = position_readings[2];

    positions[WALKING_FL] = position_readings[3];
    positions[WALKING_FR] = position_readings[4];
    positions[WALKING_ML] = position_readings[5];
    positions[WALKING_MR] = position_readings[6];
    positions[WALKING_RL] = position_readings[7];
    positions[WALKING_RR] = position_readings[8];

    positions[STEERING_FL] = position_readings[9];
    positions[STEERING_FR] = position_readings[10];
    positions[STEERING_ML] = position_readings[11];
    positions[STEERING_MR] = position_readings[12];
    positions[STEERING_RL] = position_readings[13];
    positions[STEERING_RR] = position_readings[14];

    positions[CONTACT_FL] = atan2((-1)*cos(positions[STEERING_FL])*cos(positions[WALKING_FL])*sin(positions[PASSIVE_LEFT])+(-1)*cos(positions[PASSIVE_LEFT])*cos(positions[STEERING_FL])*sin(positions[WALKING_FL]),
            cos(positions[PASSIVE_LEFT])*cos(positions[WALKING_FL])+(-1)*sin(positions[PASSIVE_LEFT])*sin(positions[WALKING_FL]));
    positions[CONTACT_FR] = atan2((-1)*cos(positions[STEERING_FR])*cos(positions[WALKING_FR])*sin(positions[PASSIVE_RIGHT])+(-1)*cos(positions[PASSIVE_RIGHT])*cos(positions[STEERING_FR])*sin(positions[WALKING_FR]),
            cos(positions[PASSIVE_RIGHT])*cos(positions[WALKING_FR])+(-1)*sin(positions[PASSIVE_RIGHT])*sin(positions[WALKING_FR]));
    positions[CONTACT_ML] = atan2((-1)*cos(positions[STEERING_ML])*cos(positions[PASSIVE_LEFT])*sin(positions[WALKING_ML])+(-1)*cos(positions[STEERING_ML])*cos(positions[WALKING_ML])*sin(positions[PASSIVE_LEFT]),
            cos(positions[WALKING_ML])*cos(positions[PASSIVE_LEFT])+(-1)*sin(positions[WALKING_ML])*sin(positions[PASSIVE_LEFT]));
    positions[CONTACT_MR] = atan2((-1)*cos(positions[STEERING_MR])*cos(positions[PASSIVE_RIGHT])*sin(positions[WALKING_MR])+(-1)*cos(positions[STEERING_MR])*cos(positions[WALKING_MR])*sin(positions[PASSIVE_RIGHT]),
            cos(positions[WALKING_MR])*cos(positions[PASSIVE_RIGHT])+(-1)*sin(positions[WALKING_MR])*sin(positions[PASSIVE_RIGHT]));
    positions[CONTACT_RL] = atan2(sin(positions[PASSIVE_REAR])*sin(positions[STEERING_RL])+(-1)*cos(positions[PASSIVE_REAR])*cos(positions[STEERING_RL])*sin(positions[WALKING_RL]),
            cos(positions[PASSIVE_REAR])*cos(positions[WALKING_RL]));
    positions[CONTACT_RR] = atan2(sin(positions[PASSIVE_REAR])*sin(positions[STEERING_RR])+(-1)*cos(positions[PASSIVE_REAR])*cos(positions[STEERING_RR])*sin(positions[WALKING_RR]),
            cos(positions[PASSIVE_REAR])*cos(positions[WALKING_RR]));

    positions[ROLLING_FL] = position_readings[15] - positions[CONTACT_FL];
    positions[ROLLING_FR] = position_readings[16] - positions[CONTACT_FR];
    positions[ROLLING_ML] = position_readings[17] - positions[CONTACT_ML];
    positions[ROLLING_MR] = position_readings[18] - positions[CONTACT_MR];
    positions[ROLLING_RL] = position_readings[19] - positions[CONTACT_FL];
    positions[ROLLING_RR] = position_readings[20] - positions[CONTACT_RR];

    velocities[PASSIVE_LEFT] = velocity_readings[0];
    velocities[PASSIVE_RIGHT] = velocity_readings[1];
    velocities[PASSIVE_REAR] = velocity_readings[2];

    velocities[WALKING_FL] = velocity_readings[3];
    velocities[WALKING_FR] = velocity_readings[4];
    velocities[WALKING_ML] = velocity_readings[5];
    velocities[WALKING_MR] = velocity_readings[6];
    velocities[WALKING_RL] = velocity_readings[7];
    velocities[WALKING_RR] = velocity_readings[8];

    velocities[STEERING_FL] = velocity_readings[9];
    velocities[STEERING_FR] = velocity_readings[10];
    velocities[STEERING_ML] = velocity_readings[11];
    velocities[STEERING_MR] = velocity_readings[12];
    velocities[STEERING_RL] = velocity_readings[13];
    velocities[STEERING_RR] = velocity_readings[14];

    velocities[CONTACT_FL] = (tan(positions[PASSIVE_LEFT] + positions[WALKING_FL]) * sin(positions[STEERING_FL]) * velocities[STEERING_FL]
            - cos(positions[STEERING_FL]) / pow(cos(positions[PASSIVE_LEFT] + positions[WALKING_FL]), 2) * (velocities[PASSIVE_LEFT] + velocities[WALKING_FL]))
            / (1.0 + pow(tan(positions[PASSIVE_LEFT] + positions[WALKING_FL]), 2) * pow(cos(positions[STEERING_FL]), 2));
    velocities[CONTACT_FR] = (tan(positions[PASSIVE_RIGHT] + positions[WALKING_FR]) * sin(positions[STEERING_FR]) * velocities[STEERING_FR]
            - cos(positions[STEERING_FR]) / pow(cos(positions[PASSIVE_RIGHT] + positions[WALKING_FR]), 2) * (velocities[PASSIVE_RIGHT] + velocities[WALKING_FR]))
            / (1.0 + pow(tan(positions[PASSIVE_RIGHT] + positions[WALKING_FR]), 2) * pow(cos(positions[STEERING_FR]), 2));
    velocities[CONTACT_ML] = (tan(positions[PASSIVE_LEFT] + positions[WALKING_ML]) * sin(positions[STEERING_ML]) * velocities[STEERING_ML]
            - cos(positions[STEERING_ML]) / pow(cos(positions[PASSIVE_LEFT] + positions[WALKING_ML]), 2) * (velocities[PASSIVE_LEFT] + velocities[WALKING_ML]))
            / (1.0 + pow(tan(positions[PASSIVE_LEFT] + positions[WALKING_ML]), 2) * pow(cos(positions[STEERING_ML]), 2));
    velocities[CONTACT_MR] = (tan(positions[PASSIVE_RIGHT] + positions[WALKING_MR]) * sin(positions[STEERING_MR]) * velocities[STEERING_MR]
            - cos(positions[STEERING_MR]) / pow(cos(positions[PASSIVE_RIGHT] + positions[WALKING_MR]), 2) * (velocities[PASSIVE_RIGHT] + velocities[WALKING_MR]))
            / (1.0 + pow(tan(positions[PASSIVE_RIGHT] + positions[WALKING_MR]), 2) * pow(cos(positions[STEERING_MR]), 2));
    velocities[CONTACT_RL] = 1.0 / (1.0 + pow(tan(positions[PASSIVE_REAR]) * sin(positions[STEERING_RL]) / cos(positions[WALKING_RL]) - tan(positions[WALKING_RL]) * cos(positions[STEERING_RL]), 2))
            * (sin(positions[STEERING_RL]) / (pow(cos(positions[PASSIVE_REAR]), 2) * cos(positions[WALKING_RL])) * velocities[PASSIVE_REAR]
                + (tan(positions[PASSIVE_REAR]) * sin(positions[STEERING_RL]) * tan(positions[WALKING_RL]) / cos(positions[WALKING_RL]) - cos(positions[STEERING_RL]) / pow(cos(positions[WALKING_RL]), 2)) * velocities[WALKING_RL]
                + (tan(positions[PASSIVE_REAR]) * cos(positions[STEERING_RL]) / cos(positions[WALKING_RL]) + tan(positions[WALKING_RL]) * sin(positions[STEERING_RL])) * velocities[STEERING_RL]);
    velocities[CONTACT_RR] = 1.0 / (1.0 + pow(tan(positions[PASSIVE_REAR]) * sin(positions[STEERING_RR]) / cos(positions[WALKING_RR]) - tan(positions[WALKING_RR]) * cos(positions[STEERING_RR]), 2))
            * (sin(positions[STEERING_RR]) / (pow(cos(positions[PASSIVE_REAR]), 2) * cos(positions[WALKING_RR])) * velocities[PASSIVE_REAR]
                + (tan(positions[PASSIVE_REAR]) * sin(positions[STEERING_RR]) * tan(positions[WALKING_RR]) / cos(positions[WALKING_RR]) - cos(positions[STEERING_RR]) / pow(cos(positions[WALKING_RR]), 2)) * velocities[WALKING_RR]
                + (tan(positions[PASSIVE_REAR]) * cos(positions[STEERING_RR]) / cos(positions[WALKING_RR]) + tan(positions[WALKING_RR]) * sin(positions[STEERING_RR])) * velocities[STEERING_RR]);

    velocities[ROLLING_FL] = velocity_readings[15] - velocities[CONTACT_FL];
    velocities[ROLLING_FR] = velocity_readings[16] - velocities[CONTACT_FR];
    velocities[ROLLING_ML] = velocity_readings[17] - velocities[CONTACT_ML];
    velocities[ROLLING_MR] = velocity_readings[18] - velocities[CONTACT_MR];
    velocities[ROLLING_RL] = velocity_readings[19] - velocities[CONTACT_FL];
    velocities[ROLLING_RR] = velocity_readings[20] - velocities[CONTACT_RR];

    position_readings_old = position_readings;
    velocity_readings_old = velocity_readings;
}

void ExoterLocomotionControl::setWalkingJointsStatus(const std::vector<bool>& walking_joints_status)
{
    this->walking_joints_status = walking_joints_status;
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
    kinematics->initMode();

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
