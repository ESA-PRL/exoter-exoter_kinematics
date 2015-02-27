#include "ExoterWheelwalkingKinematics.hpp"

#include "ExoterWheelwalkingTypes.hpp"

#include <iostream>

using namespace exoter;

ExoterWheelwalkingKinematics::ExoterWheelwalkingKinematics(const unsigned int mode, const double wheel_radius) : ExoterLocomotionKinematics(wheel_radius, mode), step_length(0.02d), offset_speed(0.0d), state(0)
{
}

ExoterWheelwalkingKinematics::~ExoterWheelwalkingKinematics()
{
}

void ExoterWheelwalkingKinematics::setOffsetSpeed(const double offset_speed)
{
    this->offset_speed = offset_speed;
}

void ExoterWheelwalkingKinematics::setStepLength(const double step_length)
{
    this->step_length = step_length;
}

void ExoterWheelwalkingKinematics::computeConfigChangeJointCommands(const std::vector<double>& positions, const std::vector<double> &velocities,
                                                                    std::vector<double>& position_commands, std::vector<double> &velocity_commands)
{
    std::vector<double> target_positions = getConfigChangeTargetJointPositions();

    position_commands.resize(0);
    position_commands.insert(position_commands.end(), target_positions.begin(), target_positions.end());
    position_commands.insert(position_commands.end(), NUMBER_OF_WHEELS, std::numeric_limits<double>::quiet_NaN());

    velocity_commands.resize(0);
    velocity_commands.insert(velocity_commands.end(), NUMBER_OF_WALKING_WHEELS, std::numeric_limits<double>::quiet_NaN());
    velocity_commands.insert(velocity_commands.end(), NUMBER_OF_STEERABLE_WHEELS, std::numeric_limits<double>::quiet_NaN());

    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
    {
        double walking_joint_velocity = velocities[NUMBER_OF_PASSIVE_JOINTS + i];
        double walking_joint_position = positions[NUMBER_OF_PASSIVE_JOINTS + i];
        velocity_commands.push_back(-(walking_joint_velocity * LEG_LENGTH / wheel_radius * cos(walking_joint_position) + walking_joint_velocity));
    }
}

void ExoterWheelwalkingKinematics::computeMovementJointCommands(const double ww_speed, const std::vector<double>& positions, const std::vector<double>& velocities,
                                                                std::vector<double>& position_commands, std::vector<double>& velocity_commands)
{
    std::map<int,double> known_body_rates;

    known_body_rates.insert(std::pair<int,double>(X_DOT, ww_speed + offset_speed));
    known_body_rates.insert(std::pair<int,double>(Y_DOT, 0.0d));
    known_body_rates.insert(std::pair<int,double>(PHI_X_DOT, 0.0d));    // Removed temporarily because of immovable left rear walking joint.
    known_body_rates.insert(std::pair<int,double>(PHI_Y_DOT, 0.0d));    // Removed temporarily because of immovable left rear walking joint.
    known_body_rates.insert(std::pair<int,double>(PHI_Z_DOT, 0.0d));

    std::map<int,double> known_joint_rates;

//    known_joint_rates.insert(std::pair<int,double>(WALKING_RL, 0.0d));    // Added temporarily because of immovable left rear walking joint.

    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FL, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_ML, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_MR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RL, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RR, 0.0d));

    double ww_rolling_rate;
    double offset_rolling_rate = offset_speed / wheel_radius;

    switch (mode)
    {
    case AXLE_BY_AXLE:
        ww_rolling_rate = ww_speed * 3 / wheel_radius;

        switch (state)
        {
            case FIRST_AXLE:
                step_distance += wheel_radius * (positions[ROLLING_FL] + positions[ROLLING_FR]) / 2;

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
            case SECOND_AXLE:
                step_distance += wheel_radius * (positions[ROLLING_ML] + positions[ROLLING_MR]) / 2;

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
            case THIRD_AXLE:
                step_distance += wheel_radius * (positions[ROLLING_RL] + positions[ROLLING_RR]) / 2;    // "positions[ROLLING_RL] +" removed temporarily because of immovable left rear walking joint.

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, ww_rolling_rate + offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, ww_rolling_rate + offset_rolling_rate));

                break;
        }

        if ((ww_speed > 0 && step_distance >= step_length) || (ww_speed < 0 && step_distance <= 0))
        {
            if (ww_speed > 0)
            {
                ++state %= 3;
                step_distance = 0;
            }
            else
            {
                state = (state == 0) ? 2 : state - 1;
                step_distance = step_length;
            }
        }

        break;
    case SIDE_BY_SIDE:
        ww_rolling_rate = ww_speed * 2 / wheel_radius;

        switch (state)
        {
            case LEFT_SIDE:
                step_distance += wheel_radius * (positions[ROLLING_FL] + positions[ROLLING_ML] + positions[ROLLING_RL]) / 3;    // "+ positions[ROLLING_RL]" removed temporarily because of immovable left rear walking joint.

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, ww_rolling_rate + offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
            case RIGHT_SIDE:
                step_distance += wheel_radius * (positions[ROLLING_FR] + positions[ROLLING_MR] + positions[ROLLING_RR]) / 3;

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, ww_rolling_rate + offset_rolling_rate));

                break;
        }

        if ((ww_speed > 0 && step_distance >= step_length / 2) || (ww_speed < 0 && step_distance <= -step_length / 2))
        {
            state++;
            state %= 2;
            step_distance = (ww_speed > 0) ? -step_length / 2 : step_length / 2;
        }

        break;
    case EVEN_ODD:
        ww_rolling_rate = ww_speed * 2 / wheel_radius;

        switch (state)
        {
            case LEFT_SIDE:
                step_distance += wheel_radius * (positions[ROLLING_FL] + positions[ROLLING_MR] + positions[ROLLING_RL]) / 3;    // "+ positions[ROLLING_RL]" removed temporarily because of immovable left rear walking joint.

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, ww_rolling_rate + offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
            case RIGHT_SIDE:
                step_distance += wheel_radius * (positions[ROLLING_FR] + positions[ROLLING_ML] + positions[ROLLING_RR]) / 3;

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); // Removed temporarily because of immovable left rear walking joint.
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, ww_rolling_rate + offset_rolling_rate));


                break;
        }

        if ((ww_speed > 0 && step_distance >= step_length / 2) || (ww_speed < 0 && step_distance <= -step_length / 2))
        {
            state++;
            state %= 2;
            step_distance = (ww_speed > 0) ? -step_length / 2 : step_length / 2;
        }

        break;
    case SINGLE_WHEEL:
        ww_rolling_rate = ww_speed * 6 / wheel_radius;

        switch (state)
        {
            case FRONT_LEFT:
                step_distance += wheel_radius * positions[ROLLING_FL];

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
	    case FRONT_RIGHT:
                step_distance += wheel_radius * positions[ROLLING_FR];

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

		break;
            case MIDDLE_LEFT:
                step_distance += wheel_radius * positions[ROLLING_ML];

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
	    case MIDDLE_RIGHT:
                step_distance += wheel_radius * positions[ROLLING_MR];

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, ww_rolling_rate + offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
            case REAR_LEFT:
                step_distance += wheel_radius * positions[ROLLING_RL];

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, ww_rolling_rate + offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, offset_rolling_rate));

                break;
	    case REAR_RIGHT:
                step_distance += wheel_radius * positions[ROLLING_RR];

                known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, offset_rolling_rate));
                known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, offset_rolling_rate)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, ww_rolling_rate + offset_rolling_rate));

	        break;
        }

        if ((ww_speed > 0 && step_distance >= step_length) || (ww_speed < 0 && step_distance <= 0))
        {
            if (ww_speed > 0)
            {
                ++state %= 6;
                step_distance = 0;
            }
            else
            {
                state = (state == 0) ? 5 : state - 1;
                step_distance = step_length;
            }
        }

        break;
    }

    std::map<int,double> body_rates;
    std::map<int,double> joint_rates;

    kinematic_model->computeRates(positions, known_body_rates, known_joint_rates, body_rates, joint_rates);

    position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());

    velocity_commands.resize(0);

    velocity_commands.push_back(joint_rates.find(WALKING_FL)->second); velocity_commands.push_back(joint_rates.find(WALKING_FR)->second);
    velocity_commands.push_back(joint_rates.find(WALKING_ML)->second); velocity_commands.push_back(joint_rates.find(WALKING_MR)->second);
    velocity_commands.push_back(joint_rates.find(WALKING_RL)->second); velocity_commands.push_back(joint_rates.find(WALKING_RR)->second);

    velocity_commands.push_back(joint_rates.find(STEERING_FL)->second); velocity_commands.push_back(joint_rates.find(STEERING_FR)->second);
    velocity_commands.push_back(joint_rates.find(STEERING_RL)->second); velocity_commands.push_back(joint_rates.find(STEERING_RR)->second);

    velocity_commands.push_back(joint_rates.find(ROLLING_FL)->second + joint_rates.find(CONTACT_FL)->second); velocity_commands.push_back(joint_rates.find(ROLLING_FR)->second + joint_rates.find(CONTACT_FR)->second);
    velocity_commands.push_back(joint_rates.find(ROLLING_ML)->second + joint_rates.find(CONTACT_ML)->second); velocity_commands.push_back(joint_rates.find(ROLLING_MR)->second + joint_rates.find(CONTACT_MR)->second);
    velocity_commands.push_back(joint_rates.find(ROLLING_RL)->second + joint_rates.find(CONTACT_RL)->second); velocity_commands.push_back(joint_rates.find(ROLLING_RR)->second + joint_rates.find(CONTACT_RR)->second);
}

std::vector<double> ExoterWheelwalkingKinematics::getConfigChangeTargetJointPositions()
{
    std::vector<double> joint_positions;    //Walking and steering joint positions

    joint_positions.insert(joint_positions.end(), NUMBER_OF_WALKING_WHEELS, 0.0d);
    joint_positions.insert(joint_positions.end(), NUMBER_OF_STEERABLE_WHEELS, 0.0d);

    return joint_positions;
}

void ExoterWheelwalkingKinematics::setMode(unsigned int mode)
{
    this->mode = mode;
    this->state = 0;

    switch (mode)
    {
        case AXLE_BY_AXLE:
        case SINGLE_WHEEL:
            this->step_distance = 0;
            break;
        case SIDE_BY_SIDE:
        case EVEN_ODD:
            this->step_distance = 0;//this->step_length / 2;
            break;
    }
}
