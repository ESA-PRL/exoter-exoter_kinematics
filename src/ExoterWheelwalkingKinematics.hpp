/**\file ExoterWheelwalkingKinematics.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date February 2014
* @version 1.0.
*/

#ifndef EXOTER_WHEELWALKING_KINEMATICS_HPP
#define EXOTER_WHEELWALKING_KINEMATICS_HPP

/* ExoTeR */
#include "Configuration.hpp"
#include "ExoterLocomotionKinematics.hpp"
#include "ExoterKinematicModel.hpp"

namespace exoter
{
    class ExoterWheelwalkingKinematics : public ExoterLocomotionKinematics
    {
    public:
        ExoterWheelwalkingKinematics(const unsigned int mode, const double wheel_radius);
        ~ExoterWheelwalkingKinematics();

        void computeConfigChangeJointCommands(const std::vector<bool>& walking_joints_status,
					      const std::vector<double>& positions, const std::vector<double>& velocities,
                                              std::vector<double>& position_commands, std::vector<double>& velocity_commands);
        void computeMovementJointCommands(const double ww_speed, const std::vector<bool>& walking_joints_status,
					  const std::vector<double>& positions, const std::vector<double>& velocities,
                                          std::vector<double>& position_commands, std::vector<double>& velocity_commands);
        std::vector<double> getConfigChangeTargetJointPositions();
        void setMode(unsigned int mode);
        void initMode();
        void setOffsetSpeed(const double offset_speed);
        void setStepLength(const double step_length);
    private:
        double step_length;
        double offset_speed;
        unsigned int state;
        double step_distance;
    };
}

#endif
