/**\file ExoterLocomotionKinematics.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date March 2014
* @version 1.0.
*/

#ifndef EXOTER_LOCOMOTION_KINEMATICS_HPP
#define EXOTER_LOCOMOTION_KINEMATICS_HPP

/* ExoTeR */
#include "Configuration.hpp"
#include "ExoterKinematicModel.hpp"

namespace exoter_kinematics
{
    class ExoterLocomotionKinematics
    {
    public:
        ExoterLocomotionKinematics(const double wheel_radius, const unsigned int mode);
        virtual ~ExoterLocomotionKinematics();

        void setMode(const unsigned int mode);
        virtual void computeConfigChangeJointCommands(const std::vector<bool>& walking_joints_status,
						                              const std::vector<double>& positions, const std::vector<double>& velocities,
                                                      std::vector<double>& position_commands, std::vector<double>& velocity_commands) = 0;
        virtual void computeMovementJointCommands(const double speed, const std::vector<bool>& walking_joints_status,
                                                  const std::vector<double>& positions, const std::vector<double>& velocities,
                                                  std::vector<double>& position_commands, std::vector<double>& velocity_commands) = 0;
        virtual std::vector<double> getConfigChangeTargetJointPositions() = 0;
	virtual void initMode() = 0;
    protected:
        ExoterKinematicModel* kinematic_model;
        unsigned int mode;
        double wheel_radius;
    };
}

#endif
