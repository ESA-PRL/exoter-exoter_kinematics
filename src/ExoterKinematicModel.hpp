/**\file ExoterKinematicKDL.hpp
*
* This class has the primitive methods for the Forward kinematics and Wheel Jacobian for ExoTer rover
* First computes the forward kinematics in order to have afterwards the wheel Jacobians using Analytical form.
*
* @author Tim Wiese | ESA/ESTEC | tim.wiese@esa.int
* @date February 2014
* @version 1.0.
*/

#ifndef EXOTER_KINEMATIC_MODEL_HPP
#define EXOTER_KINEMATIC_MODEL_HPP

/* Base includes */
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <sstream>
#include <string> /** This should be already in sstream **/
#include <map>

/* Odometry library */
#include <odometry/KinematicModel.hpp> /** For the Odometry class to inherit **/

/* ExoTer */
#include <exoter_kinematics/Configuration.hpp>


namespace exoter_kinematics
{

    class ExoterKinematicModel : public ::odometry::KinematicModel<double, NUMBER_OF_WHEELS, EXOTER_JOINT_DOF, SLIP_VECTOR_SIZE, CONTACT_POINT_DOF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        std::string model_name; /** Name of the model class **/
	unsigned int number_wheels;
        std::vector<int>  contact_idx;
	double wheelRadius; /** Wheel radius */

    public:

        ExoterKinematicModel(const double wheel_radius);//constructor
        virtual ~ExoterKinematicModel();//destructor

        std::string name();
        void contactPointsPerTree (std::vector<unsigned int> &contactPoints);
        void setPointsInContact (const std::vector<int> &pointsInContact);
        void fkBody2ContactPointt(const int chainIdx, const std::vector<double> &positions, Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov);
        void fkBody2ContactPoint(const int chainIdx, const std::vector<double> &positions,  Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov);
        void fkSolver(const std::vector<double> &positions, std::vector<Eigen::Affine3d> &fkRobot, std::vector<base::Matrix6d> &fkCov);
                      Eigen::Matrix<double, 6*NUMBER_OF_WHEELS,MODEL_DOF> jacobianSolver(const std::vector<double> &positions);
        void computeRates(const std::vector<double> &joint_positions,
                          const std::map<int,double> &known_body_rates, const std::map<int,double> &known_joint_rates,
                          std::map<int,double> &body_rates, std::map<int,double> &joint_rates);
    private:
        void rearrangeKinematicMatrices(const std::map<int,double> &known_body_rates, const std::map<int,double> &known_joint_rates,
                                        const Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, 6> &E, const Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, MODEL_DOF> &J,
                                        Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &A, Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &B);
        void removeMatrixColumns(Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &matrix, const std::map<int,double> columns_to_remove);
    };
};

#endif
