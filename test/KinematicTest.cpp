#include <exoter_kinematics/ExoterKinematicKDL.hpp>
#include <exoter_kinematics/ExoterKinematicModel.hpp>
#include <exoter_kinematics/Configuration.hpp>
#include <odometry/MotionModel.hpp>

#include <map>
#include <vector>
#include <string>

#define BOOST_TEST_MODULE KinematicModelTest
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace exoter_kinematics;

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif


BOOST_AUTO_TEST_CASE( ModelvsKDLTest )
{

    std::string urdf_file = boost::unit_test::framework::master_test_suite().argv[1];
    double wheelRadius = atof(boost::unit_test::framework::master_test_suite().argv[2]);
    std::vector<double> wheelDrive (exoter_kinematics::NUMBER_OF_WHEELS, 0);
    std::vector<double> passiveJoint (exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS, 0);
    std::vector<double> steerJoint (exoter_kinematics::NUMBER_OF_STEERABLE_WHEELS, 0);
    std::vector<double> wheelWalking (exoter_kinematics::NUMBER_OF_WALKING_WHEELS, 0);
    std::vector<double> contactAngle (exoter_kinematics::NUMBER_OF_WHEELS, 0);
    std::vector<double> slip (exoter_kinematics::SLIP_VECTOR_SIZE, 0);
    std::vector<double> joints (exoter_kinematics::ExoterKinematicKDL::MODEL_DOF, 0);
    std::vector<Eigen::Affine3d> fkRobotModel, fkRobotKDL;
    std::vector<base::Matrix6d> fkCovModel, fkCovKDL;

    std::cout<<"URDF FILE: "<<urdf_file<<"\n";
    std::cout<<"EXOTER WHEEL RADIUS: "<< wheelRadius <<"\n";
    std::cout<<"EXOTER MODEL_DOF: "<< exoter_kinematics::ExoterKinematicKDL::MODEL_DOF <<"\n";
    std::cout<<"EXOTER MODEL_DOF: "<< exoter_kinematics::ExoterKinematicModel::MODEL_DOF <<"\n";

    /** Wheel Drive **/
//    wheelDrive[0] = 45.00 * D2R; wheelDrive[2] = 45.00 * D2R; wheelDrive[4] = 45.00 * D2R;
//    wheelDrive[1] = 45.00 * D2R; wheelDrive[3] = 45.00 * D2R; wheelDrive[5] = 45.00 * D2R;

    /** Passive joints **/
//    passiveJoint[0] = 45.00 * D2R;
//    passiveJoint[1] = 45.00 * D2R;
//    passiveJoint[2] = 45.00 * D2R;

    /** Wheel walking **/
//    wheelWalking[0] = 45.00 * D2R;   wheelWalking[2] = 45.00 * D2R; wheelWalking[4] = 45.00 * D2R;
//    wheelWalking[1] = 45.00 * D2R;   wheelWalking[3] = 45.00 * D2R; wheelWalking[5] = 45.00 * D2R;

    /** Steer joints **/
//    steerJoint[0] = 45.00 * D2R;//FL
//    steerJoint[1] = 45.00 * D2R;//FR
//    steerJoint[2] = 45.00 * D2R;//RL
//    steerJoint[3] = 45.00 * D2R;//RR

    /** Contact Angle **/
//    contactAngle[0] = -90.00 * D2R;   contactAngle[2] = -45.00 * D2R; contactAngle[4] = -45.00 * D2R;
//    contactAngle[1] = -45.00 * D2R;   contactAngle[3] = -45.00 * D2R; contactAngle[5] = -45.00 * D2R;

//    slip[0] = 0.30; //slip[1] = 0.12; slip[2] = 10.0 * D2R;

    /** Store the values **/
    for (register int i=0; i<static_cast<int>(exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS); ++i)
    {
        joints[i] = passiveJoint[i];
    }
    for (register int i=0; i<static_cast<int>(exoter_kinematics::NUMBER_OF_WALKING_WHEELS); ++i)
    {
        joints[i+exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS] = wheelWalking[i];
    }
    for (register int i=0; i<static_cast<int>(exoter_kinematics::NUMBER_OF_STEERABLE_WHEELS); ++i)
    {
        joints[i+exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS+exoter_kinematics::NUMBER_OF_WALKING_WHEELS] = steerJoint[i];
    }
    for (register int i=0; i<static_cast<int>(exoter_kinematics::NUMBER_OF_WHEELS); ++i)
    {
        joints[i+exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS+exoter_kinematics::NUMBER_OF_WALKING_WHEELS+exoter_kinematics::NUMBER_OF_STEERABLE_WHEELS] = wheelDrive[i];
    }


    /** Only for the first wheel **/
    for (register int i=0; i<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++i)
    {
        joints[i+exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS+exoter_kinematics::NUMBER_OF_WALKING_WHEELS+exoter_kinematics::NUMBER_OF_STEERABLE_WHEELS+exoter_kinematics::NUMBER_OF_WHEELS] = slip[i];
    }
    for (register int i=0; i<static_cast<int>(exoter_kinematics::NUMBER_OF_WHEELS); ++i)
    {
        joints[i+exoter_kinematics::NUMBER_OF_PASSIVE_JOINTS+exoter_kinematics::NUMBER_OF_WALKING_WHEELS+exoter_kinematics::NUMBER_OF_STEERABLE_WHEELS+exoter_kinematics::NUMBER_OF_WHEELS+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)] = contactAngle[i];
    }


    std::cout << "EXOTER: joints contains:";
    for (std::vector<double>::iterator it = joints.begin() ; it != joints.end(); ++it)
        std::cout << ' ' << *it;
    std::cout << '\n';
//==============================================================================================
    std::cout<<"********* EXOTER ROBOT MODEL *********\n";
    exoter_kinematics::ExoterKinematicModel exoterModel(wheelRadius);
    exoterModel.fkSolver(joints, fkRobotModel, fkCovModel);

    std::cout<<"fkRobotModel of size: "<<fkRobotModel.size()<<" fkCovModel of size: "<<fkCovModel.size()<<"\n";
//============================================================================================

    std::cout<<"********* EXOTER KDL MODEL *********\n";
    exoter_kinematics::ExoterKinematicKDL exoterKDL(urdf_file, wheelRadius);

    std::cout<<"EXOTER max chain DoF: "<<exoterKDL.getMaxChainDoF()<<"\n";

    exoterKDL.fkSolver(joints, fkRobotKDL, fkCovKDL);

    std::cout<<"fkRobotKDL of size: "<<fkRobotKDL.size()<<" fkCovKDL of size: "<<fkCovKDL.size()<<"\n";

    for (register unsigned int i=0; i<fkRobotKDL.size(); ++i)
    {
        std::cout<<"fkRobotModel chain["<<i<<"]\n"<<fkRobotModel[i].matrix()<<"\n";
        std::cout<<"fkRobotKDL chain["<<i<<"]\n"<<fkRobotKDL[i].matrix()<<"\n";
        /** TO-DO **/
        for (register unsigned int j=0; j<4; ++j)
            for (register unsigned int l=0; l<4; ++l)
            BOOST_CHECK_SMALL(fkRobotModel[i].matrix()(j,l)-fkRobotKDL[i].matrix()(j,l), 0.0001);
    }

    std::cout<<"********* EXOTER ROBOT Model JACOBIAN *********\n\n";
    Eigen::Matrix <double, 6*NUMBER_OF_WHEELS, exoter_kinematics::ExoterKinematicModel::MODEL_DOF> JModel;

    JModel = exoterModel.jacobianSolver(joints);

    std::cout<<"J Model is of size "<<JModel.rows()<<" x "<<JModel.cols()<<"\n"<< JModel <<"\n";

    std::cout<<"**\n\n******* EXOTER KDL JACOBIAN *********\n";
    Eigen::Matrix <double, 6*NUMBER_OF_WHEELS, exoter_kinematics::ExoterKinematicKDL::MODEL_DOF> JKdl;

    JKdl = exoterKDL.jacobianSolver(joints);

    std::cout<<"J KDL is of size "<<JKdl.rows()<<" x "<<JKdl.cols()<<"\n"<< JKdl <<"\n\n";

    std::cout<<"Row key:\n";

    for (int i = 0; i < 36; i++)
    {
        switch (i%6)
        {
        case 0:
            std::cout<<"Row "<<i<<" is X_DOT."<<"\n";
            break;
        case 1:
            std::cout<<"Row "<<i<<" is Y_DOT."<<"\n";
            break;
        case 2:
            std::cout<<"Row "<<i<<" is Z_DOT."<<"\n";
            break;
        case 3:
            std::cout<<"Row "<<i<<" is PHI_X_DOT."<<"\n";
            break;
        case 4:
            std::cout<<"Row "<<i<<" is PHI_Y_DOT."<<"\n";
            break;
        case 5:
            std::cout<<"Row "<<i<<" is PHI_Z_DOT."<<"\n";
            break;
        }
    }

    std::cout<<"\nColumn key:\n";
    std::cout<<"Col "<<PASSIVE_LEFT<<" is PASSIVE_LEFT."<<"\n";
    std::cout<<"Col "<<PASSIVE_RIGHT<<" is PASSIVE_RIGHT."<<"\n";
    std::cout<<"Col "<<PASSIVE_REAR<<" is PASSIVE_REAR."<<"\n";
    std::cout<<"Col "<<WALKING_FL<<" is WALKING_FL."<<"\n";
    std::cout<<"Col "<<WALKING_FR<<" is WALKING_FR."<<"\n";
    std::cout<<"Col "<<WALKING_ML<<" is WALKING_ML."<<"\n";
    std::cout<<"Col "<<WALKING_MR<<" is WALKING_MR."<<"\n";
    std::cout<<"Col "<<WALKING_RL<<" is WALKING_RL."<<"\n";
    std::cout<<"Col "<<WALKING_RR<<" is WALKING_RR."<<"\n";
    std::cout<<"Col "<<STEERING_FL<<" is STEERING_FL."<<"\n";
    std::cout<<"Col "<<STEERING_FR<<" is STEERING_FR."<<"\n";
    std::cout<<"Col "<<STEERING_RL<<" is STEERING_RL."<<"\n";
    std::cout<<"Col "<<STEERING_RR<<" is STEERING_RR."<<"\n";
    std::cout<<"Col "<<ROLLING_FL<<" is ROLLING_FL."<<"\n";
    std::cout<<"Col "<<ROLLING_FR<<" is ROLLING_FR."<<"\n";
    std::cout<<"Col "<<ROLLING_ML<<" is ROLLING_ML."<<"\n";
    std::cout<<"Col "<<ROLLING_MR<<" is ROLLING_MR."<<"\n";
    std::cout<<"Col "<<ROLLING_RL<<" is ROLLING_RL."<<"\n";
    std::cout<<"Col "<<ROLLING_RR<<" is ROLLING_RR."<<"\n";
//    std::cout<<"Col "<<ROLL_SLIP_FL<<" is ROLL_SLIP_FL."<<"\n";
//    std::cout<<"Col "<<SIDE_SLIP_FL<<" is SIDE_SLIP_FL."<<"\n";
//    std::cout<<"Col "<<TURN_SLIP_FL<<" is TURN_SLIP_FL."<<"\n";
//    std::cout<<"Col "<<ROLL_SLIP_FR<<" is ROLL_SLIP_FR."<<"\n";
//    std::cout<<"Col "<<SIDE_SLIP_FR<<" is SIDE_SLIP_FR."<<"\n";
//    std::cout<<"Col "<<TURN_SLIP_FR<<" is TURN_SLIP_FR."<<"\n";
//    std::cout<<"Col "<<ROLL_SLIP_ML<<" is ROLL_SLIP_ML."<<"\n";
//    std::cout<<"Col "<<SIDE_SLIP_ML<<" is SIDE_SLIP_ML."<<"\n";
//    std::cout<<"Col "<<TURN_SLIP_ML<<" is TURN_SLIP_ML."<<"\n";
//    std::cout<<"Col "<<ROLL_SLIP_MR<<" is ROLL_SLIP_MR."<<"\n";
//    std::cout<<"Col "<<SIDE_SLIP_MR<<" is SIDE_SLIP_MR."<<"\n";
//    std::cout<<"Col "<<TURN_SLIP_MR<<" is TURN_SLIP_MR."<<"\n";
//    std::cout<<"Col "<<ROLL_SLIP_RL<<" is ROLL_SLIP_RL."<<"\n";
//    std::cout<<"Col "<<SIDE_SLIP_RL<<" is SIDE_SLIP_RL."<<"\n";
//    std::cout<<"Col "<<TURN_SLIP_RL<<" is TURN_SLIP_RL."<<"\n";
//    std::cout<<"Col "<<ROLL_SLIP_RR<<" is ROLL_SLIP_RR."<<"\n";
//    std::cout<<"Col "<<SIDE_SLIP_RR<<" is SIDE_SLIP_RR."<<"\n";
//    std::cout<<"Col "<<TURN_SLIP_RR<<" is TURN_SLIP_RR."<<"\n";
    std::cout<<"Col "<<CONTACT_FL<<" is CONTACT_FL."<<"\n";
    std::cout<<"Col "<<CONTACT_FR<<" is CONTACT_FR."<<"\n";
    std::cout<<"Col "<<CONTACT_ML<<" is CONTACT_ML."<<"\n";
    std::cout<<"Col "<<CONTACT_MR<<" is CONTACT_MR."<<"\n";
    std::cout<<"Col "<<CONTACT_RL<<" is CONTACT_RL."<<"\n";
    std::cout<<"Col "<<CONTACT_RR<<" is CONTACT_RR."<<"\n\n";

    for (int m = 0; m < JModel.rows(); m++)
        for (int n = 0; n < JModel.cols(); n++)
            if (n < 19 || n > 36)
                if (std::abs(JModel(m,n)-JKdl(m,n)) > 0.0001)
                    std::cout<<"Error in row "<<m<<", col "<<n<<". Value is: "<<JModel(m,n)<<", value should be: "<<JKdl(m,n)<<"\n";

    std::cout<<"**\n\n******* COMPUTED BODY AND JOINT RATES *********\n";

    std::map<int,double> known_body_rates;

    known_body_rates.insert(std::pair<int,double>(X_DOT, 0.03d));
    known_body_rates.insert(std::pair<int,double>(Y_DOT, 0.0d));
    known_body_rates.insert(std::pair<int,double>(PHI_X_DOT, 0.0d));
    known_body_rates.insert(std::pair<int,double>(PHI_Y_DOT, 0.0d));
    known_body_rates.insert(std::pair<int,double>(PHI_Z_DOT, 0.0d));

    std::map<int,double> known_joint_rates;

//    known_joint_rates.insert(std::pair<int,double>(PASSIVE_LEFT, 0.0d)); known_joint_rates.insert(std::pair<int,double>(PASSIVE_RIGHT, 0.0d)); known_joint_rates.insert(std::pair<int,double>(PASSIVE_REAR, 0.0d));

//    known_joint_rates.insert(std::pair<int,double>(WALKING_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(WALKING_FR, 0.0d));
//    known_joint_rates.insert(std::pair<int,double>(WALKING_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(WALKING_MR, 0.0d));
//    known_joint_rates.insert(std::pair<int,double>(WALKING_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(WALKING_RR, 0.0d));

    known_joint_rates.insert(std::pair<int,double>(STEERING_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(STEERING_FR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(STEERING_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(STEERING_RR, 0.0d));

    known_joint_rates.insert(std::pair<int,double>(ROLLING_FL, 1.29d)); known_joint_rates.insert(std::pair<int,double>(ROLLING_FR, 1.29d));
    known_joint_rates.insert(std::pair<int,double>(ROLLING_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(ROLLING_MR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLLING_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(ROLLING_RR, 0.0d));

    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FL, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_FR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_FR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_ML, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_MR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_MR, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RL, 0.0d));
    known_joint_rates.insert(std::pair<int,double>(ROLL_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(SIDE_SLIP_RR, 0.0d)); known_joint_rates.insert(std::pair<int,double>(TURN_SLIP_RR, 0.0d));

//    known_joint_rates.insert(std::pair<int,double>(CONTACT_FL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(CONTACT_FR, 0.0d));
//    known_joint_rates.insert(std::pair<int,double>(CONTACT_ML, 0.0d)); known_joint_rates.insert(std::pair<int,double>(CONTACT_MR, 0.0d));
//    known_joint_rates.insert(std::pair<int,double>(CONTACT_RL, 0.0d)); known_joint_rates.insert(std::pair<int,double>(CONTACT_RR, 0.0d));

    std::map<int,double> body_rates;
    std::map<int,double> joint_rates;

    exoterModel.computeRates(joints, known_body_rates, known_joint_rates, body_rates, joint_rates);

    std::cout << std::endl << "Computed body rates:" << std::endl;

    for (std::map<int,double>::const_iterator it = body_rates.begin(); it != body_rates.end(); it++)
            std::cout << it->second << std::endl;

    std::cout << std::endl << "Computed joint rates:" << std::endl;

    for (std::map<int,double>::const_iterator it = joint_rates.begin(); it != joint_rates.end(); it++)
            std::cout << it->second << std::endl;
}
