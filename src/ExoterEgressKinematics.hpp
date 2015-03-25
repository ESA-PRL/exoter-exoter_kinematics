/**\file ExoterEgressKinematics.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date March 2014
* @version 1.0.
*/

#ifndef EXOTER_EGRESS_KINEMATICS_HPP
#define EXOTER_EGRESS_KINEMATICS_HPP

/* ExoTeR */
#include "Configuration.hpp"
#include "ExoterLocomotionKinematics.hpp"
#include "ExoterKinematicModel.hpp"

namespace exoter
{
    class ExoterEgressKinematics : public ExoterLocomotionKinematics
    {
    public:
        ExoterEgressKinematics(const double wheel_radius, const unsigned int mode);
        ~ExoterEgressKinematics();

        void computeConfigChangeJointCommands(const std::vector<bool>& walking_joints_status,
					      const std::vector<double>& positions, const std::vector<double>& velocities,
                                              std::vector<double>& position_commands, std::vector<double>& velocity_commands);
        void computeMovementJointCommands(const double speed, const std::vector<bool>& walking_joints_status,
					  const std::vector<double>& positions, const std::vector<double>& velocities,
                                          std::vector<double>& position_commands, std::vector<double>& velocity_commands);
        std::vector<double> getConfigChangeTargetJointPositions();

        double getHeightOffset();
        void setHeightOffset(const double height_offset);
	void initMode();
    private:
        double height_offset;
    };
}

#endif
