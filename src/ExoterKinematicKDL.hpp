/**\file ExoterKinematicKDL.hpp
*
* This class has the primitive methods for the Forward kinematics and Wheel Jacobian for ExoTer.
* First computes the forward kinematics in order to have afterwards the wheel Jacobians using KDL.
*
* It read a URDF file to create KDL trees using the kdl_parse.
*
* @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
* @date October 2013.
* @version 1.0.
*/


#ifndef EXOTER_KINEMATIC_KDL_HPP
#define EXOTER_KINEMATIC_KDL_HPP

/** Library logger **/
#include<base/Logging.hpp>

/* Base includes */
#include <base/time.h>
#include <base/eigen.h>
#include <Eigen/Core>
#include <sstream>
#include <string> /** This should be already in sstream **/

/* Odometry library */
#include <odometry/KinematicModel.hpp> /** For the Odometry class to inherit **/

/* KDL Parser */
#include <kdl_parser/kdl_parser.hpp>

/** KDL library **/
#include <kdl/frames_io.hpp> /** Defines KDL frames **/
#include <kdl/chainfksolver.hpp> /** Solve forward kinematics **/
#include <kdl/chainfksolverpos_recursive.hpp> /** Position of the end-effector **/
#include <kdl/chainjnttojacsolver.hpp> /** For jacobian matrix **/


/* URDF */
#include "urdf_parser/urdf_parser.h"
#include <urdf_model/model.h>


/* ExoTer */
#include <exoter/Configuration.hpp>

namespace exoter
{

    class ExoterKinematicKDL : public ::odometry::KinematicModel<double, NUMBER_OF_WHEELS, EXOTER_JOINT_DOF, SLIP_VECTOR_SIZE, CONTACT_POINT_DOF>
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        KDL::Tree tree; /** There are as many chains as number of wheels **/
        std::string model_name; /** Name of the model class **/
        double wheelRadius; /** Wheel radius */
        unsigned int number_wheels;

        /** Wheel Jacobian Matrix  (passive joint for Rear wheels/column of zeros for Front wheels, wheel rotation, (3 x 1)slip vector and contact_angle**/     
        std::vector< Eigen::Matrix <double, 2*3, MAX_CHAIN_DOF> , Eigen::aligned_allocator < Eigen::Matrix <double, 2*3, MAX_CHAIN_DOF> > > wheelJacobian;
        std::vector<int>  contact_idx; /** Foot index making the motion (0,...,4) of the wheelidx wheel **/

    public:

        ExoterKinematicKDL(std::string &urdf_file, const double wheel_radius);
        ~ExoterKinematicKDL();

        std::string name();
        void contactPointsPerTree (std::vector<unsigned int> &contactPoints);
        void setPointsInContact (const std::vector<int> &pointsInContact);
        void fkBody2ContactPointt(const int chainIdx, const std::vector<double> &positions, Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov);
        void fkSolver(const std::vector<double> &positions, std::vector<Eigen::Affine3d> &fkRobot, std::vector<base::Matrix6d> &fkCov);
        Eigen::Matrix<double, 6*NUMBER_OF_WHEELS,MODEL_DOF> jacobianSolver(const std::vector<double> &positions);

    };

    static void printLink(const KDL::SegmentMap::const_iterator& link, const std::string& prefix)
    {
        //std::cout << prefix << "- Segment " << link->second.segment.getName() << " has " << link->second.children.size() << " children" << std::endl;
        for (unsigned int i=0; i<link->second.children.size(); ++i)
            printLink(link->second.children[i], prefix + "  ");
    };

    static std::string getContactPoint (const KDL::SegmentMap::const_iterator& link, const int childrenIdx)
    {
        if (link->second.children.size() == 0)
            return link->second.segment.getName();

        //std::cout<<link->second.segment.getName() << " has "<<link->second.children.size()<<" children\n";
        return getContactPoint(link->second.children[childrenIdx], 0);
    };

};

#endif
