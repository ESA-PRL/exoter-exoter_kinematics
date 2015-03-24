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

void ExoterWheelwalkingKinematics::computeConfigChangeJointCommands(const std::vector<bool>& walking_joints_status,
                                                                    const std::vector<double>& positions, const std::vector<double> &velocities,
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

void ExoterWheelwalkingKinematics::computeMovementJointCommands(const double ww_speed, const std::vector<bool>& walking_joints_status,
                                                                const std::vector<double>& positions, const std::vector<double>& velocities,
                                                                std::vector<double>& position_commands, std::vector<double>& velocity_commands)
{
    std::map<int,double> known_body_rates;

    known_body_rates.insert(std::pair<int,double>(X_DOT, ww_speed + offset_speed));
    known_body_rates.insert(std::pair<int,double>(Y_DOT, 0.0d));

    known_body_rates.insert(std::pair<int,double>(PHI_X_DOT, 0.0d));
    known_body_rates.insert(std::pair<int,double>(PHI_Y_DOT, 0.0d)); 
    known_body_rates.insert(std::pair<int,double>(PHI_Z_DOT, 0.0d));

    std::map<int,double> known_joint_rates;

    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FL, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_ML, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_MR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RL, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RR, 0.0d));

    std::vector<int> active_wheels;

    switch (mode)
    {
    case AXLE_BY_AXLE:
        switch (state)
        {
            case FIRST_AXLE:
                active_wheels.push_back(ROLLING_FL);
                active_wheels.push_back(ROLLING_FR);
                break;
            case SECOND_AXLE:
                active_wheels.push_back(ROLLING_ML);
                active_wheels.push_back(ROLLING_MR);
                break;
            case THIRD_AXLE:
                active_wheels.push_back(ROLLING_RL);
                active_wheels.push_back(ROLLING_RR);
                break;
        }

        break;
    case SIDE_BY_SIDE:
        switch (state)
        {
            case LEFT_SIDE:
                active_wheels.push_back(ROLLING_FL);
                active_wheels.push_back(ROLLING_ML);
                active_wheels.push_back(ROLLING_RL); 
                break;
            case RIGHT_SIDE:
                active_wheels.push_back(ROLLING_FR);
                active_wheels.push_back(ROLLING_MR);
                active_wheels.push_back(ROLLING_RR); 
                break;
        }

        break;
    case EVEN_ODD:
        switch (state)
        {
            case LEFT_SIDE:
                active_wheels.push_back(ROLLING_FL);
                active_wheels.push_back(ROLLING_MR);
                active_wheels.push_back(ROLLING_RL); 
                break;
            case RIGHT_SIDE:
                active_wheels.push_back(ROLLING_FR);
                active_wheels.push_back(ROLLING_ML);
                active_wheels.push_back(ROLLING_RR);
                break;
        }

        break;
    case SINGLE_WHEEL:
        switch (state)
        {
            case FRONT_LEFT:
                active_wheels.push_back(ROLLING_FL);    
                break;
            case FRONT_RIGHT:
                active_wheels.push_back(ROLLING_FR);
                break;
            case MIDDLE_LEFT:
                active_wheels.push_back(ROLLING_ML);
                break;
            case MIDDLE_RIGHT:
                active_wheels.push_back(ROLLING_MR);
                break;
            case REAR_LEFT:
                active_wheels.push_back(ROLLING_RL);
                break;
            case REAR_RIGHT:
                active_wheels.push_back(ROLLING_RR);
                break;
        }

        break;
    case NORMAL_DRIVING:
        break;
    }

    const int ROLLING_OFFSET = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS;

    double offset_rolling_rate = offset_speed / wheel_radius;

    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        known_joint_rates.insert(std::pair<int,double>(ROLLING_OFFSET + i, offset_rolling_rate));

    if (active_wheels.size() != 0)
    {
        double ww_rolling_rate = ww_speed / wheel_radius * NUMBER_OF_WHEELS / active_wheels.size();
        double step_sum = 0.0d;

        for (unsigned int i = 0; i < active_wheels.size(); i++)
        {
            known_joint_rates[active_wheels[i]] += ww_rolling_rate;

            if (walking_joints_status[active_wheels[i] - ROLLING_OFFSET])
            {
                step_sum += positions[active_wheels[i]];
            }
            else
            {
                // Virtual joint position to handle disabled walking joint
                step_sum += positions[active_wheels[i]] * NUMBER_OF_WHEELS / active_wheels.size();
            }
        }

        step_distance += wheel_radius * step_sum / active_wheels.size();
    }

    switch (mode)
    {
    case AXLE_BY_AXLE:
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
    case EVEN_ODD:
        if ((ww_speed > 0 && step_distance >= step_length / 2) || (ww_speed < 0 && step_distance <= -step_length / 2))
        {
            state++;
            state %= 2;
            step_distance = (ww_speed > 0) ? -step_length / 2 : step_length / 2;
        }

        break;
    case SINGLE_WHEEL:
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
    case NORMAL_DRIVING:
        break;
    }

    bool no_disabled_walking_joint = true;

    for (int i = 0; i < NUMBER_OF_WALKING_WHEELS; i++)
    {
        if (!walking_joints_status[i])
        {
            // Set desired joint rate of disabled walking joint to zero and remove constraint on corresponding rolling joint
            known_joint_rates.erase(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i);
            known_joint_rates.insert(std::pair<int,double>(NUMBER_OF_PASSIVE_JOINTS + i, 0.0d));

            no_disabled_walking_joint = false;
        }
    }

    if (!no_disabled_walking_joint)
    {
        known_body_rates.erase(PHI_X_DOT);
        known_body_rates.erase(PHI_Y_DOT); 
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
    this->step_distance = 0;
}
