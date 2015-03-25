#include "ExoterEgressKinematics.hpp"

#include "ExoterEgressTypes.hpp"

using namespace exoter;

ExoterEgressKinematics::ExoterEgressKinematics(const double wheel_radius, const unsigned int mode) : ExoterLocomotionKinematics(wheel_radius, mode), height_offset(0.0d)
{
}

ExoterEgressKinematics::~ExoterEgressKinematics()
{
}

void ExoterEgressKinematics::computeConfigChangeJointCommands(const std::vector<bool>& walking_joints_status,
                                                              const std::vector<double>& positions, const std::vector<double>& velocities,
                                                              std::vector<double>& position_commands, std::vector<double>& velocity_commands)
{
    std::vector<double> target_positions = getConfigChangeTargetJointPositions();

    position_commands.resize(0);
    position_commands.insert(position_commands.end(), target_positions.begin(), target_positions.end());
    position_commands.insert(position_commands.end(), NUMBER_OF_WHEELS, std::numeric_limits<double>::quiet_NaN());

    velocity_commands.resize(0);
    velocity_commands.insert(velocity_commands.end(), NUMBER_OF_WALKING_WHEELS, std::numeric_limits<double>::quiet_NaN());
    velocity_commands.insert(velocity_commands.end(), NUMBER_OF_STEERABLE_WHEELS, std::numeric_limits<double>::quiet_NaN());

    switch (mode)
    {
    case STOWED:
    case NOMINAL:
    case LOWER_COG_FW:
    case LOWER_COG_BW:
    case STEP_DOWN_FW:
    case STEP_DOWN_BW:
        for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        {
            double walking_joint_velocity = velocities[NUMBER_OF_PASSIVE_JOINTS + i];
            double walking_joint_position = positions[NUMBER_OF_PASSIVE_JOINTS + i];
            velocity_commands.push_back(-(walking_joint_velocity * LEG_LENGTH / wheel_radius * cos(walking_joint_position) + walking_joint_velocity));
        }

        break;
//    case STEP_DOWN_FW:
//        //Wheel velocities are only approximations! For accurate results with minimized slip, the kinematic model should be used.
//        for (int i = 0; i < 2; i++)
//        {
//            double walking_joint_velocity = velocities[NUMBER_OF_PASSIVE_JOINTS + i];
//            double walking_joint_velocity_2nd_axle = velocities[NUMBER_OF_PASSIVE_JOINTS + i + 2];
//            double walking_joint_angle_2nd_axle = positions[NUMBER_OF_PASSIVE_JOINTS + i + 2];
//            velocity_commands.push_back(walking_joint_velocity_2nd_axle * cos(walking_joint_angle_2nd_axle)
//                                        - (walking_joint_velocity * LEG_LENGTH / wheel_radius + walking_joint_velocity));
//        }

//        for (int i = 2; i < NUMBER_OF_WHEELS; i++)
//        {
//            double walking_joint_velocity = velocities[NUMBER_OF_PASSIVE_JOINTS + i];
//            velocity_commands.push_back(-walking_joint_velocity);
//        }

//        break;
//    case STEP_DOWN_BW:
//        //Wheel velocities are only approximations! For accurate results with minimized slip, the kinematic model should be used.
//        for (int i = 0; i < 4; i++)
//        {
//            double walking_joint_velocity = velocities[NUMBER_OF_PASSIVE_JOINTS + i];
//            velocity_commands.push_back(-walking_joint_velocity);
//        }

//        for (int i = 4; i < NUMBER_OF_WHEELS; i++)
//        {
//            double walking_joint_velocity = velocities[NUMBER_OF_PASSIVE_JOINTS + i];
//            double walking_joint_velocity_2nd_axle = velocities[NUMBER_OF_PASSIVE_JOINTS + i - 2];
//            double walking_joint_velocity_1st_axle = velocities[NUMBER_OF_PASSIVE_JOINTS + i - 4];
//            double walking_joint_angle_2nd_axle = positions[NUMBER_OF_PASSIVE_JOINTS + i - 2];
//            double walking_joint_angle_1st_axle = positions[NUMBER_OF_PASSIVE_JOINTS + i - 4];
//            velocity_commands.push_back((walking_joint_velocity_2nd_axle * cos(walking_joint_angle_2nd_axle) + walking_joint_velocity_1st_axle * cos(walking_joint_angle_1st_axle)) / 2
//                                        - (walking_joint_velocity * LEG_LENGTH / wheel_radius + walking_joint_velocity));
//        }

//        break;
    }
}

void ExoterEgressKinematics::computeMovementJointCommands(const double speed, const std::vector<bool>& walking_joints_status, const std::vector<double>& positions, const std::vector<double>& velocities,
                                                          std::vector<double>& position_commands, std::vector<double>& velocity_commands)
{
    switch (mode)
    {
    case STOWED:
        position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());
        velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS, 0.0d);
        break;
    case NOMINAL:
    case LOWER_COG_FW:
    case LOWER_COG_BW:
//    case STEP_DOWN_FW:
//    case STEP_DOWN_BW:
        position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());
        velocity_commands.resize(0);
        velocity_commands.insert(velocity_commands.end(), NUMBER_OF_WALKING_WHEELS, 0.0d);
        velocity_commands.insert(velocity_commands.end(), NUMBER_OF_STEERABLE_WHEELS, 0.0d);
        velocity_commands.insert(velocity_commands.end(), NUMBER_OF_WHEELS, speed / wheel_radius);
        break;
    case STEP_DOWN_FW:
    case STEP_DOWN_BW:
        if (mode == STEP_DOWN_FW && speed < 0)
        {
            position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());
            velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS, 0.0d);

            return;
        }
        else if (mode == STEP_DOWN_BW && speed > 0)
        {
            position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());
            velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS, 0.0d);

            return;
        }

        std::map<int,double> known_body_rates;

        known_body_rates.insert(std::pair<int,double>(X_DOT, speed));
        known_body_rates.insert(std::pair<int,double>(Y_DOT, 0.0d));
        known_body_rates.insert(std::pair<int,double>(PHI_X_DOT, 0.0d));
        known_body_rates.insert(std::pair<int,double>(PHI_Z_DOT, 0.0d));

        std::map<int,double> known_joint_rates;

        known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FL, 0.0d));
        known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FR, 0.0d));
        known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_ML, 0.0d));
        known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_MR, 0.0d));
        known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RL, 0.0d));
        known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RR, 0.0d));

        if (mode == STEP_DOWN_FW)
        {
            known_joint_rates.insert(std::pair<int,double>(WALKING_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(WALKING_FR, 0.0d));

            for (int i = 2; i < NUMBER_OF_WALKING_WHEELS; i++)
            {
                if (std::abs(positions[NUMBER_OF_PASSIVE_JOINTS + i]) <= std::abs(positions[NUMBER_OF_PASSIVE_JOINTS + (i%2)]) + MAX_POS_OFFSET)    // TOFIX
                    known_joint_rates.insert(std::pair<int,double>(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i, 0.0d));
                else
                    known_joint_rates.insert(std::pair<int,double>(NUMBER_OF_PASSIVE_JOINTS + i, 0.0d));
            }
        }
        else
        {
            known_joint_rates.insert(std::pair<int,double>(WALKING_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(WALKING_RR, 0.0d));

            for (int i = 0; i < NUMBER_OF_WALKING_WHEELS - 2; i++)
            {
                if (std::abs(positions[NUMBER_OF_PASSIVE_JOINTS + i]) <= std::abs(positions[NUMBER_OF_PASSIVE_JOINTS + 4 + (i%2)]) + MAX_POS_OFFSET)    // TOFIX
                    known_joint_rates.insert(std::pair<int,double>(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i, 0.0d));
                else
                    known_joint_rates.insert(std::pair<int,double>(NUMBER_OF_PASSIVE_JOINTS + i, 0.0d));
            }
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

        break;
    }
}

std::vector<double> ExoterEgressKinematics::getConfigChangeTargetJointPositions()
{
    std::vector<double> joint_positions;    //Walking and steering joint positions

    switch (mode)
    {
    case STOWED:
        joint_positions.insert(joint_positions.end(), 6, M_PI / 2);
        break;
    case NOMINAL:
        joint_positions.insert(joint_positions.end(), 6, 0.0d);
        break;
    case LOWER_COG_FW:
    case STEP_DOWN_FW:
        joint_positions.insert(joint_positions.end(), 6, -acos(1 - height_offset));
        break;
    case LOWER_COG_BW:
    case STEP_DOWN_BW:
        joint_positions.insert(joint_positions.end(), 6, acos(1 - height_offset));
        break;
//    case STEP_DOWN_FW:
//        joint_positions.insert(joint_positions.end(), 2, -acos(1 - height_offset));
//        joint_positions.insert(joint_positions.end(), 4, acos(1 - height_offset));
//        break;
//    case STEP_DOWN_BW:
//        joint_positions.insert(joint_positions.end(), 4, -acos(1 - height_offset));
//        joint_positions.insert(joint_positions.end(), 2, acos(1 - height_offset));
//        break;
    }

    joint_positions.insert(joint_positions.end(), NUMBER_OF_STEERABLE_WHEELS, 0.0d);

    return joint_positions;
}

double ExoterEgressKinematics::getHeightOffset()
{
    return height_offset;
}

void ExoterEgressKinematics::setHeightOffset(const double height_offset)
{
    this->height_offset = height_offset;
}

void ExoterEgressKinematics::initMode()
{
    return;
}
