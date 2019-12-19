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
    static const int NUMBER_OF_STEERABLE_WHEELS = 6; /** Wheels that are steerable **/
    static const int NUMBER_OF_WALKING_WHEELS = 6; /** Number of walking wheels **/
    static const int NUMBER_OF_ACTIVE_JOINTS = NUMBER_OF_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WALKING_WHEELS; /** Number of active joints **/
    static const int NUMBER_OF_PASSIVE_JOINTS = 3; /** Number of passive joints **/
    static const int EXOTER_JOINT_DOF = NUMBER_OF_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_PASSIVE_JOINTS; /** All joints in the chassis (21 joints) **/
    static const int SLIP_VECTOR_SIZE = 6; /** 3D Slip vector model **/
    static const int CONTACT_POINT_DOF = 1; /** DoF of the contact point **/
	
    static const double LEG_LENGTH = 0.260;
    static const double FRONT_PASSIVE_TO_VIRTUAL = 0.260;
    static const double REAR_PASSIVE_TO_VIRTUAL = 0.720;
    static const double REAR_VIRTUAL_TO_STEERING = 0.000;
    static const double VIRTUAL_TO_WALKING = 0.035;
    static const double CENTERLINE_TO_STEERING = 0.720;
    static const double BODY_TO_FRONT = 0.380;
    static const double BODY_TO_REAR = 0.710;

    enum Chains {FL, FR, ML, MR, RL, RR};
    enum Joints {PASSIVE, WALKING, STEERING, ROLLING, ROLL_SLIP, SIDE_SLIP, TURN_SLIP, CONTACT};
    enum Body_States {X_DOT, Y_DOT, Z_DOT, PHI_X_DOT, PHI_Y_DOT, PHI_Z_DOT};
    enum Joint_Order {PASSIVE_RIGHT, PASSIVE_REAR, PASSIVE_LEFT, 
                      WALKING_FR, WALKING_MR, WALKING_RR, WALKING_RL, WALKING_ML, WALKING_FL, 
                      STEERING_FR, STEERING_MR, STEERING_RR, STEERING_RL, STEERING_ML, STEERING_FL,
                      CONTACT_FR, CONTACT_MR, CONTACT_RR, CONTACT_RL, CONTACT_ML, CONTACT_FL,
                      ROLLING_FR, ROLLING_MR, ROLLING_RR, ROLLING_RL, ROLLING_ML, ROLLING_FL,
                      LONG_SLIP_FR, LONG_SLIP_MR, LONG_SLIP_RR, LONG_SLIP_RL, LONG_SLIP_ML, LONG_SLIP_FL,
                      LAT_SLIP_FR, LAT_SLIP_MR, LAT_SLIP_RR, LAT_SLIP_RL, LAT_SLIP_ML, LAT_SLIP_FL,
                      PERP_SLIP_FR, PERP_SLIP_MR, PERP_SLIP_RR, PERP_SLIP_RL, PERP_SLIP_ML, PERP_SLIP_FL,
                      ROLL_SLIP_FR, ROLL_SLIP_MR, ROLL_SLIP_RR, ROLL_SLIP_RL, ROLL_SLIP_ML, ROLL_SLIP_FL,
                      PITCH_SLIP_FR, PITCH_SLIP_MR, PITCH_SLIP_RR, PITCH_SLIP_RL, PITCH_SLIP_ML, PITCH_SLIP_FL,
                      YAW_SLIP_FR, YAW_SLIP_MR, YAW_SLIP_RR, YAW_SLIP_RL, YAW_SLIP_ML, YAW_SLIP_FL};

    static const double MAX_POS_OFFSET = 0.001d;
}

#endif
