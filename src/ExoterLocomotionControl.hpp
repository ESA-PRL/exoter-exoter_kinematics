/**\file ExoterLocomotionControl.hpp
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date March 2014
* @version 1.0.
*/

#ifndef EXOTER_LOCOMOTION_CONTROL_HPP
#define EXOTER_LOCOMOTION_CONTROL_HPP

/* ExoTeR */
#include "Configuration.hpp"
#include "ExoterLocomotionKinematics.hpp"

namespace exoter
{
    class ExoterLocomotionControl
    {
    public:
        void getJointCommands(std::vector<double>& position_commands, std::vector<double>& velocity_commands);
        void setNewJointReadings(const std::vector<double>& position_readings, const std::vector<double>& velocity_readings);
        void selectMode(const unsigned int mode);
        void setSpeed(const double speed);
	void stopMotion();
	void startMotion();
	void initJointConfiguration();
    protected:
        ExoterLocomotionControl(unsigned int mode);
        ~ExoterLocomotionControl();

        ExoterLocomotionKinematics* kinematics;
        int current_mode;
        bool mode_transition;
	bool stop_motion;
        double speed;
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> position_readings_old;
        std::vector<double> velocity_readings_old;

        bool checkTargetJointPositionsReached();
        std::vector<double> assemblePositionVector(const std::vector<double> position_readings);
        std::vector<double> assembleVelocityVector(const std::vector<double> velocity_readings);
    };
}

#endif
