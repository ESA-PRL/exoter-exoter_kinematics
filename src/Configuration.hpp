#ifndef _EXOTER_CONFIGURATION_HPP_
#define _EXOTER_CONFIGURATION_HPP_

/** This class should only contain configuration values for
 * Exoter than are only needed at compilation time due to
 * e.g.: template classes. Otherwise, the values should go
 * in configuration files (i.e.: xml,  yml, urdf...)
 */

namespace exoter_kinematics
{
    /** General constants values **/
    static const int NUMBER_OF_WHEELS = 6; /** Six wheels of the chassis **/
    static const int NUMBER_OF_STEERABLE_WHEELS = 4; /** Wheels that are steerable **/
    static const int NUMBER_OF_WALKING_WHEELS = 6; /** Number of walking wheels **/
    static const int NUMBER_OF_ACTIVE_JOINTS = NUMBER_OF_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WALKING_WHEELS; /** Number of active joints **/
    static const int NUMBER_OF_PASSIVE_JOINTS = 3; /** Number of passive joints **/
    static const int EXOTER_JOINT_DOF = NUMBER_OF_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_PASSIVE_JOINTS; /** All joints in the chassis (19 joints) **/
    static const int SLIP_VECTOR_SIZE = 3; /** 3D Slip vector model **/
    static const int CONTACT_POINT_DOF = 1; /** DoF of the contact point **/
	
    static const double LEG_LENGTH = 0.1253;
    static const double WALKING_AXES_DISTANCE = 0.265;
    static const double FRONT_PASSIVE_LINK_LENGTH = WALKING_AXES_DISTANCE;
    static const double REAR_PASSIVE_LINK_LENGTH = 0.478;
    static const double FRONT_PASSIVE_TO_VIRTUAL = FRONT_PASSIVE_LINK_LENGTH / 2;
    static const double REAR_PASSIVE_TO_VIRTUAL = REAR_PASSIVE_LINK_LENGTH / 2;
    static const double REAR_VIRTUAL_TO_STEERING = 0.071;
    static const double VIRTUAL_TO_WALKING = 0.005;
    static const double CENTERLINE_TO_STEERING = REAR_PASSIVE_LINK_LENGTH / 2 + REAR_VIRTUAL_TO_STEERING;
    static const double BODY_TO_FRONT = WALKING_AXES_DISTANCE / 2;
    static const double BODY_TO_REAR = WALKING_AXES_DISTANCE;

    enum Chains {FL, FR, ML, MR, RL, RR};
    enum Joints {PASSIVE, WALKING, STEERING, ROLLING, ROLL_SLIP, SIDE_SLIP, TURN_SLIP, CONTACT};
    enum Body_States {X_DOT, Y_DOT, Z_DOT, PHI_X_DOT, PHI_Y_DOT, PHI_Z_DOT};
    enum Joint_Order {PASSIVE_LEFT, PASSIVE_RIGHT, PASSIVE_REAR,
                      WALKING_FL, WALKING_FR, WALKING_ML, WALKING_MR, WALKING_RL, WALKING_RR,
                      STEERING_FL, STEERING_FR, STEERING_RL, STEERING_RR,
                      ROLLING_FL, ROLLING_FR, ROLLING_ML, ROLLING_MR, ROLLING_RL, ROLLING_RR,
                      ROLL_SLIP_FL, SIDE_SLIP_FL, TURN_SLIP_FL,
                      ROLL_SLIP_FR, SIDE_SLIP_FR, TURN_SLIP_FR,
                      ROLL_SLIP_ML, SIDE_SLIP_ML, TURN_SLIP_ML,
                      ROLL_SLIP_MR, SIDE_SLIP_MR, TURN_SLIP_MR,
                      ROLL_SLIP_RL, SIDE_SLIP_RL, TURN_SLIP_RL,
                      ROLL_SLIP_RR, SIDE_SLIP_RR, TURN_SLIP_RR,
                      CONTACT_FL, CONTACT_FR, CONTACT_ML, CONTACT_MR, CONTACT_RL, CONTACT_RR};

    static const double MAX_POS_OFFSET = 0.001d;
}

#endif
