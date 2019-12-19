#include "ExoterKinematicModel.hpp"
#include <exoter_kinematics/Configuration.hpp>
#include <iostream>
#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

using namespace exoter_kinematics;
double r; 

//============================================================================================================================
ExoterKinematicModel::ExoterKinematicModel (const double wheel_radius)
{
    /** Take the wheel radius **/
    this->wheelRadius = wheel_radius;
    this->number_wheels = static_cast<unsigned int>(this->getNumberOfTrees());
}
//==========================================================================================================================
ExoterKinematicModel::~ExoterKinematicModel()
{
}

void ExoterKinematicModel::fkBody2ContactPoint(const int chainIdx, const std::vector<double> &positions,  Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov)
{
    std::vector<double> chainpositions(positions);
    chainpositions[ROLLING] = 0.0d;
    chainpositions[ROLL_SLIP] = 0.0d;
    chainpositions[SIDE_SLIP] = 0.0d;
    chainpositions[TURN_SLIP] = 0.0d;

    fkBody2ContactPointt(chainIdx, chainpositions, fkTrans, fkCov);
}

//================================================================================================================================
void ExoterKinematicModel::fkBody2ContactPointt(const int chainIdx, const std::vector<double> &positions,  Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov)
{	
    double passive_angle = 0.0d;
    double walking_angle = -positions[WALKING];
    double steering_angle = -positions[STEERING];
	double rolling_angle = positions[ROLLING];
	double roll_slip = positions[ROLL_SLIP];
	double side_slip = positions[SIDE_SLIP];
	double turn_slip = positions[TURN_SLIP];
    double contact_angle = -positions[CONTACT];
	
    Eigen::Affine3d T_BC;
    Eigen::Affine3d T_CCt;
	Eigen::Affine3d T_BCt;
	
	Eigen::Matrix3d rot_BC;
	
    rot_BC(0,0) = cos(walking_angle) * cos(steering_angle) * cos(contact_angle) - sin(walking_angle) * sin(contact_angle);
    rot_BC(0,1) = cos(walking_angle) * sin(steering_angle);
    rot_BC(0,2) = -(cos(walking_angle) * cos(steering_angle) * sin(contact_angle) + sin(walking_angle) * cos(contact_angle));
    rot_BC(1,0) = -sin(steering_angle) * cos(contact_angle);
    rot_BC(1,1) = cos(steering_angle);
    rot_BC(1,2) = sin(steering_angle) * sin(contact_angle);
    rot_BC(2,0) = sin(walking_angle) * cos(steering_angle) * cos(contact_angle) + cos(walking_angle) * sin(contact_angle);
    rot_BC(2,1) = sin(walking_angle) * sin(steering_angle);
    rot_BC(2,2) = -(sin(walking_angle) * cos(steering_angle) * sin(contact_angle) - cos(walking_angle) * cos(contact_angle));
	
	Eigen::Vector3d trans_BC;
    trans_BC.setZero();
	
	switch (chainIdx)
	{
		case FL:
		case FR:
		case ML:
		case MR:
            passive_angle = -positions[PASSIVE];
			trans_BC(0) = cos(passive_angle) * (chainIdx == FL || chainIdx == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL + cos(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius + sin(walking_angle) * cos(contact_angle) * this->wheelRadius + sin(walking_angle) * LEG_LENGTH + BODY_TO_FRONT;
			trans_BC(1) = -sin(steering_angle) * sin(contact_angle) * this->wheelRadius + (chainIdx == FL || chainIdx == ML ? 1 : -1) * CENTERLINE_TO_STEERING;
			trans_BC(2) = sin(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius - cos(walking_angle) * cos(contact_angle) * this->wheelRadius - cos(walking_angle) * LEG_LENGTH - VIRTUAL_TO_WALKING + sin(passive_angle) * (chainIdx == FL || chainIdx == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL;
			break;
		case RL:
		case RR:
            passive_angle = positions[PASSIVE];
            trans_BC(0) = cos(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius + sin(walking_angle) * cos(contact_angle) * this->wheelRadius + sin(walking_angle) * LEG_LENGTH - BODY_TO_REAR;
			trans_BC(1) = cos(passive_angle) * (chainIdx == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL - sin(steering_angle) * sin(contact_angle) * this->wheelRadius + (chainIdx == RL ? 1 : -1) * REAR_VIRTUAL_TO_STEERING;
			trans_BC(2) = sin(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius - cos(walking_angle) * cos(contact_angle) * this->wheelRadius - cos(walking_angle) * LEG_LENGTH - VIRTUAL_TO_WALKING + sin(passive_angle) * (chainIdx == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL;
			break;
	}
	
	T_BC.linear() = rot_BC;
	T_BC.translation() = trans_BC;
	
	Eigen::Matrix3d rot_CCt;
	
	rot_CCt(0,0) = cos(turn_slip);
	rot_CCt(0,1) = -sin(turn_slip);
	rot_CCt(0,2) = 0;
	rot_CCt(1,0) = sin(turn_slip);
	rot_CCt(1,1) = cos(turn_slip);
	rot_CCt(1,2) = 0;
	rot_CCt(2,0) = 0;
	rot_CCt(2,1) = 0;
	rot_CCt(2,2) = 1;
	
	Eigen::Vector3d trans_CCt;
	
	trans_CCt(0) = this->wheelRadius * rolling_angle + roll_slip;
	trans_CCt(1) = side_slip;
	trans_CCt(2) = 0;
	
	T_CCt.linear() = rot_CCt;
	T_CCt.translation() = trans_CCt;	
	
	T_BCt = T_BC * T_CCt;
	
	fkTrans = T_BCt;

	fkCov.setZero();

    return;
}
//=============================================================================================================================
void ExoterKinematicModel::fkSolver(const std::vector<double> &positions, std::vector<Eigen::Affine3d> &fkRobot, std::vector<base::Matrix6d> &fkCov)
{
    std::vector<double> chainpositions;
	chainpositions.resize(8);
	
    /** Check if the number of values is correct **/
    if (positions.size() == EXOTER_JOINT_DOF)
    {
        /** Resize the vectors **/
        fkRobot.resize(this->number_wheels);
        fkCov.resize(this->number_wheels);

        /** Calculate the Forward Kinematics for all the chains of the robot **/
        for (int i = 0; i < static_cast<int>(this->number_wheels); ++i)
        {
		    /** Fill position vector for current chain **/
			
			switch (i)
			{
				case FL:
				case FR:
				case ML:
				case MR:
					chainpositions[PASSIVE] = positions[i%2];															// passive				
					chainpositions[STEERING] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + i];		// steering
					break;
				case RL:
				case RR:
					chainpositions[PASSIVE] = positions[2];																// passive
					chainpositions[STEERING] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + i];  	// steering
					break;
			}
			
            chainpositions[WALKING] = positions[NUMBER_OF_PASSIVE_JOINTS + i];                                                                                                      // wheel walking
			chainpositions[ROLLING] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i];												// wheel rolling
			chainpositions[ROLL_SLIP] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * i];						// rolling slip
			chainpositions[SIDE_SLIP] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * i + 1];					// side slip
			chainpositions[TURN_SLIP] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * i + 2];					// turn slip
			chainpositions[CONTACT] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * NUMBER_OF_WHEELS + i];	// contact angle
						
            #ifdef DEBUG_PRINTS
            std::cout << "[ROBOT_FKSOLVER] chainpositions contains:";
            for (std::vector<double>::iterator it = chainpositions.begin(); it != chainpositions.end(); ++it)
                std::cout << ' ' << *it;
            std::cout << '\n';
            #endif

            /** Perform the forward kinematics **/
            fkBody2ContactPointt(i, chainpositions, fkRobot[i], fkCov[i]);
        }
    }
	
    return;
}
//=========================================================================================================================================

Eigen::Matrix<double, 6*NUMBER_OF_WHEELS, ExoterKinematicModel::MODEL_DOF> ExoterKinematicModel::jacobianSolver(const std::vector<double> &positions)
{
    Eigen::Matrix<double, 6*exoter_kinematics::NUMBER_OF_WHEELS, ExoterKinematicModel::MODEL_DOF> Jacobian;
	Jacobian.setZero();
	
    // constants
    // reference frame at rover body center on top of base plate
    double l1 = 0.380;    // rover reference frame to front bogie (horizontal)
    double l2 = 0.060;    // rover reference frame to front bogie (vertical)
    double l3 = 0.130;    // rover reference frame to rear bogie (vertical)
    double l4 = 0.720;    // rover center line to wheel center line
    double l5 = 0.710;    // rover reference frame to rear walking axis (horizontal)
    double l6 = 0.285;    // front bogie axis to front walking axis (horizontal)
    double l7 = 0.235;    // front bogie axis to center walking axis (horizontal)
    double l8 = 0.0425;   // front bogie axis to front/center walking axis (vertical)
    double l9 = 0.0275;   // rear bogie axis to rear walking axis (vertical)
    double l10 = 0.040;   // front/rear walking axis to front/rear steering axis (horizontal)
    double l11 = 0.260;   // walking axis to driving axis (vertical, "leg length")
    double l12 = 0.090;   // center walking axis to center steering axis (horizontal)
    double r = 0.14485;   // wheel radius
    
    // input joint positions
    double phi_FRB = positions[PASSIVE_RIGHT];
    double phi_RB = positions[PASSIVE_REAR];
    double phi_FLB = positions[PASSIVE_LEFT];

    double phi_FRW = positions[WALKING_FR];
    double phi_CRW = positions[WALKING_MR];
    double phi_RRW = positions[WALKING_RR];
    double phi_RLW = positions[WALKING_RL];
    double phi_CLW = positions[WALKING_ML];
    double phi_FLW = positions[WALKING_FL];

    double phi_FRS = positions[STEERING_FR];
    double phi_CRS = positions[STEERING_MR];
    double phi_RRS = positions[STEERING_RR];
    double phi_RLS = positions[STEERING_RL];
    double phi_CLS = positions[STEERING_ML];
    double phi_FLS = positions[STEERING_FL];
    
    double phi_FRA = positions[CONTACT_FR];
    double phi_CRA = positions[CONTACT_MR];
    double phi_RRA = positions[CONTACT_RR];
    double phi_RLA = positions[CONTACT_RL];
    double phi_CLA = positions[CONTACT_ML];
    double phi_FLA = positions[CONTACT_FL];

    //double phi_FRA = atan2((-1)*cos(phi_FRS)*cos(phi_FRW)*sin(phi_FRB)+(-1)*cos(phi_FRB)*cos(phi_FRS)*sin(phi_FRW), cos(phi_FRB)*cos(phi_FRW)+(-1)*sin(phi_FRB)*sin(phi_FRW));
    //double phi_CRA = atan2((-1)*cos(phi_CRS)*cos(phi_FRB)*sin(phi_CRW)+(-1)*cos(phi_CRS)*cos(phi_CRW)*sin(phi_FRB), cos(phi_CRW)*cos(phi_FRB)+(-1)*sin(phi_CRW)*sin(phi_FRB));
    //double phi_FLA = atan2((-1)*cos(phi_FLS)*cos(phi_FLW)*sin(phi_FLB)+(-1)*cos(phi_FLB)*cos(phi_FLS)*sin(phi_FLW), cos(phi_FLB)*cos(phi_FLW)+(-1)*sin(phi_FLB)*sin(phi_FLW));
    //double phi_CLA = atan2((-1)*cos(phi_CLS)*cos(phi_FLB)*sin(phi_CLW)+(-1)*cos(phi_CLS)*cos(phi_CLW)*sin(phi_FLB), cos(phi_CLW)*cos(phi_FLB)+(-1)*sin(phi_CLW)*sin(phi_FLB));
    //double phi_RLA = atan2(sin(phi_RB)*sin(phi_RLS)+(-1)*cos(phi_RB)*cos(phi_RLS)*sin(phi_RLW), cos(phi_RB)*cos(phi_RLW));
    //double phi_RRA = atan2(sin(phi_RB)*sin(phi_RRS)+(-1)*cos(phi_RB)*cos(phi_RRS)*sin(phi_RRW), cos(phi_RB)*cos(phi_RRW));
    
    //std::cout << "phi:" << std::endl;

    //std::cout << phi_FLB << std::endl;
    //std::cout << phi_FRB << std::endl;    
    //std::cout << phi_RB  << std::endl;

    std::cout << phi_FLW << std::endl;
    std::cout << phi_FRW << std::endl;
    std::cout << phi_CLW << std::endl;
    std::cout << phi_CRW << std::endl;
    std::cout << phi_RLW << std::endl;
    std::cout << phi_RRW << std::endl;

    //std::cout << phi_FLS << std::endl;
    //std::cout << phi_FRS << std::endl;
    //std::cout << phi_CLS << std::endl;
    //std::cout << phi_CRS << std::endl;
    //std::cout << phi_RLS << std::endl;
    //std::cout << phi_RRS << std::endl;

    std::cout << phi_FLA << std::endl;
    std::cout << phi_FRA << std::endl;
    std::cout << phi_CLA << std::endl;
    std::cout << phi_CRA << std::endl;
    std::cout << phi_RLA << std::endl;
    std::cout << phi_RRA << std::endl;

    // clang-format off
    Jacobian << cos(phi_FRA)*cos(phi_FRS)*cos(phi_FRB+phi_FRW)+(-1)*sin(phi_FRA)*sin(phi_FRB+phi_FRW),cos(phi_FRA)*sin(phi_FRS),(-1)*cos(phi_FRB)*(cos(phi_FRW)*sin(phi_FRA)+cos(phi_FRA)*cos(phi_FRS)*sin(phi_FRW))+sin(phi_FRB)*((-1)*cos(phi_FRA)*cos(phi_FRS)*cos(phi_FRW)+sin(phi_FRA)*sin(phi_FRW)),(-1)*sin(phi_FRB)*(l4*sin(phi_FRA)+r*sin(phi_FRS))*sin(phi_FRW)+cos(phi_FRB)*(cos(phi_FRW)*(l4*sin(phi_FRA)+(r+l11*cos(phi_FRA))*sin(phi_FRS))+cos(phi_FRA)*(l4*cos(phi_FRS)*sin(phi_FRW)+sin(phi_FRS)*((-1)*l8+l10*sin(phi_FRW))))+cos(phi_FRA)*(l4*cos(phi_FRS)*cos(phi_FRW)*sin(phi_FRB)+sin(phi_FRS)*((-1)*l2+sin(phi_FRB)*(l6+l10*cos(phi_FRW)+(-1)*l11*sin(phi_FRW)))),sin(phi_FRA)*(l10+l6*cos(phi_FRW)+l1*cos(phi_FRB+phi_FRW)+(-1)*l8*sin(phi_FRW)+(-1)*l2*sin(phi_FRB+phi_FRW))+cos(phi_FRS)*((-1)*r+cos(phi_FRA)*((-1)*l11+l8*cos(phi_FRW)+l2*cos(phi_FRB+phi_FRW)+l6*sin(phi_FRW)+l1*sin(phi_FRB+phi_FRW))),cos(phi_FRA)*((-1)*l4*cos(phi_FRS)*sin(phi_FRB)*sin(phi_FRW)+sin(phi_FRS)*(l1+sin(phi_FRB)*(l8+(-1)*l11*cos(phi_FRW)+(-1)*l10*sin(phi_FRW)))+cos(phi_FRB)*(l4*cos(phi_FRS)*cos(phi_FRW)+sin(phi_FRS)*(l6+l10*cos(phi_FRW)+(-1)*l11*sin(phi_FRW))))+(-1)*(l4*sin(phi_FRA)+r*sin(phi_FRS))*sin(phi_FRB+phi_FRW),sin(phi_FRA)*(l10+l6*cos(phi_FRW)+(-1)*l8*sin(phi_FRW))+cos(phi_FRS)*((-1)*r+cos(phi_FRA)*((-1)*l11+l8*cos(phi_FRW)+l6*sin(phi_FRW))),0,0,(-1)*(r+l11*cos(phi_FRA))*cos(phi_FRS)+l10*sin(phi_FRA),0,0,0,0,0,0,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        (-1)*cos(phi_FRB+phi_FRW)*sin(phi_FRS),cos(phi_FRS),sin(phi_FRS)*sin(phi_FRB+phi_FRW),(-1)*(r*sin(phi_FRA)+l4*sin(phi_FRS))*sin(phi_FRB+phi_FRW)+cos(phi_FRS)*((-1)*l2+(-1)*l8*cos(phi_FRB)+(1/2)*r*cos(phi_FRA+(-1)*phi_FRB+(-1)*phi_FRW)+l11*cos(phi_FRB+phi_FRW)+(1/2)*r*cos(phi_FRA+phi_FRB+phi_FRW)+l6*sin(phi_FRB)+l10*sin(phi_FRB+phi_FRW)),sin(phi_FRS)*(l11+r*cos(phi_FRA)+(-1)*l8*cos(phi_FRW)+(-1)*l2*cos(phi_FRB+phi_FRW)+(-1)*l6*sin(phi_FRW)+(-1)*l1*sin(phi_FRB+phi_FRW)),(-1)*cos(phi_FRB+phi_FRW)*(r*sin(phi_FRA)+l4*sin(phi_FRS))+cos(phi_FRS)*(l1+l6*cos(phi_FRB)+l10*cos(phi_FRB+phi_FRW)+l8*sin(phi_FRB)+(1/2)*r*sin(phi_FRA+(-1)*phi_FRB+(-1)*phi_FRW)+(-1)*l11*sin(phi_FRB+phi_FRW)+(-1/2)*r*sin(phi_FRA+phi_FRB+phi_FRW)),sin(phi_FRS)*(l11+r*cos(phi_FRA)+(-1)*l8*cos(phi_FRW)+(-1)*l6*sin(phi_FRW)),0,0,(l11+r*cos(phi_FRA))*sin(phi_FRS),0,0,0,0,0,(-1)*r*sin(phi_FRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        cos(phi_FRB)*(cos(phi_FRS)*cos(phi_FRW)*sin(phi_FRA)+cos(phi_FRA)*sin(phi_FRW))+sin(phi_FRB)*(cos(phi_FRA)*cos(phi_FRW)+(-1)*cos(phi_FRS)*sin(phi_FRA)*sin(phi_FRW)),sin(phi_FRA)*sin(phi_FRS),cos(phi_FRA)*cos(phi_FRB+phi_FRW)+(-1)*cos(phi_FRS)*sin(phi_FRA)*sin(phi_FRB+phi_FRW),(-1)*l4*cos(phi_FRA)*cos(phi_FRB+phi_FRW)+sin(phi_FRA)*(l4*cos(phi_FRS)*sin(phi_FRB+phi_FRW)+sin(phi_FRS)*((-1)*l2+(-1)*l8*cos(phi_FRB)+l11*cos(phi_FRB+phi_FRW)+l6*sin(phi_FRB)+l10*sin(phi_FRB+phi_FRW))),cos(phi_FRS)*sin(phi_FRA)*((-1)*l11+l8*cos(phi_FRW)+l2*cos(phi_FRB+phi_FRW)+l6*sin(phi_FRW)+l1*sin(phi_FRB+phi_FRW))+(-1)*cos(phi_FRA)*(l10+l6*cos(phi_FRW)+l1*cos(phi_FRB+phi_FRW)+(-1)*l8*sin(phi_FRW)+(-1)*l2*sin(phi_FRB+phi_FRW)),l4*cos(phi_FRA)*cos(phi_FRW)*sin(phi_FRB)+cos(phi_FRB)*(l4*cos(phi_FRS)*cos(phi_FRW)*sin(phi_FRA)+l4*cos(phi_FRA)*sin(phi_FRW)+sin(phi_FRA)*sin(phi_FRS)*(l6+l10*cos(phi_FRW)+(-1)*l11*sin(phi_FRW)))+sin(phi_FRA)*((-1)*l4*cos(phi_FRS)*sin(phi_FRB)*sin(phi_FRW)+sin(phi_FRS)*(l1+sin(phi_FRB)*(l8+(-1)*l11*cos(phi_FRW)+(-1)*l10*sin(phi_FRW)))),cos(phi_FRS)*sin(phi_FRA)*((-1)*l11+l8*cos(phi_FRW)+l6*sin(phi_FRW))+(-1)*cos(phi_FRA)*(l10+l6*cos(phi_FRW)+(-1)*l8*sin(phi_FRW)),0,0,(-1)*l10*cos(phi_FRA)+(-1)*l11*cos(phi_FRS)*sin(phi_FRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_FRA)*cos(phi_FRS)*cos(phi_FRB+phi_FRW)+(-1)*sin(phi_FRA)*sin(phi_FRB+phi_FRW),cos(phi_FRA)*sin(phi_FRS),(-1)*cos(phi_FRB)*(cos(phi_FRW)*sin(phi_FRA)+cos(phi_FRA)*cos(phi_FRS)*sin(phi_FRW))+sin(phi_FRB)*((-1)*cos(phi_FRA)*cos(phi_FRS)*cos(phi_FRW)+sin(phi_FRA)*sin(phi_FRW)),cos(phi_FRA)*sin(phi_FRS),0,0,cos(phi_FRA)*sin(phi_FRS),0,0,0,0,0,(-1)*sin(phi_FRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,(-1)*cos(phi_FRB+phi_FRW)*sin(phi_FRS),cos(phi_FRS),sin(phi_FRS)*sin(phi_FRB+phi_FRW),cos(phi_FRS),0,0,cos(phi_FRS),0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_FRB)*(cos(phi_FRS)*cos(phi_FRW)*sin(phi_FRA)+cos(phi_FRA)*sin(phi_FRW))+sin(phi_FRB)*(cos(phi_FRA)*cos(phi_FRW)+(-1)*cos(phi_FRS)*sin(phi_FRA)*sin(phi_FRW)),sin(phi_FRA)*sin(phi_FRS),cos(phi_FRA)*cos(phi_FRB+phi_FRW)+(-1)*cos(phi_FRS)*sin(phi_FRA)*sin(phi_FRB+phi_FRW),sin(phi_FRA)*sin(phi_FRS),0,0,sin(phi_FRA)*sin(phi_FRS),0,0,0,0,0,cos(phi_FRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,
    
        cos(phi_CRA)*cos(phi_CRS)*cos(phi_CRW+phi_FRB)+(-1)*sin(phi_CRA)*sin(phi_CRW+phi_FRB),cos(phi_CRA)*sin(phi_CRS),(-1)*cos(phi_CRW)*(cos(phi_FRB)*sin(phi_CRA)+cos(phi_CRA)*cos(phi_CRS)*sin(phi_FRB))+sin(phi_CRW)*((-1)*cos(phi_CRA)*cos(phi_CRS)*cos(phi_FRB)+sin(phi_CRA)*sin(phi_FRB)),(-1)*(l4*sin(phi_CRA)+r*sin(phi_CRS))*sin(phi_CRW)*sin(phi_FRB)+cos(phi_CRW)*(cos(phi_FRB)*(l4*sin(phi_CRA)+(r+l11*cos(phi_CRA))*sin(phi_CRS))+cos(phi_CRA)*(l4*cos(phi_CRS)+(-1)*l12*sin(phi_CRS))*sin(phi_FRB))+(-1)*cos(phi_CRA)*((-1)*l4*cos(phi_CRS)*cos(phi_FRB)*sin(phi_CRW)+sin(phi_CRS)*(l2+cos(phi_FRB)*(l8+l12*sin(phi_CRW))+(l7+l11*sin(phi_CRW))*sin(phi_FRB))),(-1)*sin(phi_CRA)*(l12+l7*cos(phi_CRW)+(-1)*l1*cos(phi_CRW+phi_FRB)+l8*sin(phi_CRW)+l2*sin(phi_CRW+phi_FRB))+cos(phi_CRS)*((-1)*r+cos(phi_CRA)*((-1)*l11+l8*cos(phi_CRW)+l2*cos(phi_CRW+phi_FRB)+(-1)*l7*sin(phi_CRW)+l1*sin(phi_CRW+phi_FRB))),(-1)*(l4*sin(phi_CRA)+r*sin(phi_CRS))*sin(phi_CRW+phi_FRB)+cos(phi_CRA)*(l4*cos(phi_CRS)*cos(phi_CRW+phi_FRB)+sin(phi_CRS)*(l1+(-1)*l7*cos(phi_FRB)+(-1)*l12*cos(phi_CRW+phi_FRB)+l8*sin(phi_FRB)+(-1)*l11*sin(phi_CRW+phi_FRB))),(-1)*sin(phi_CRA)*(l12+l7*cos(phi_CRW)+l8*sin(phi_CRW))+(-1)*cos(phi_CRS)*(r+cos(phi_CRA)*(l11+(-1)*l8*cos(phi_CRW)+l7*sin(phi_CRW))),0,0,0,(-1)*(r+l11*cos(phi_CRA))*cos(phi_CRS)+(-1)*l12*sin(phi_CRA),0,0,0,0,0,0,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        (-1)*cos(phi_CRW+phi_FRB)*sin(phi_CRS),cos(phi_CRS),sin(phi_CRS)*sin(phi_CRW+phi_FRB),(-1)*cos(phi_CRS)*(l2+cos(phi_FRB)*(l8+(-1)*(l11+r*cos(phi_CRA))*cos(phi_CRW)+l12*sin(phi_CRW))+(l7+l12*cos(phi_CRW)+(l11+r*cos(phi_CRA))*sin(phi_CRW))*sin(phi_FRB))+(-1)*(r*sin(phi_CRA)+l4*sin(phi_CRS))*sin(phi_CRW+phi_FRB),sin(phi_CRS)*(l11+r*cos(phi_CRA)+(-1)*l8*cos(phi_CRW)+(-1)*l2*cos(phi_CRW+phi_FRB)+l7*sin(phi_CRW)+(-1)*l1*sin(phi_CRW+phi_FRB)),(-1)*cos(phi_CRW+phi_FRB)*(r*sin(phi_CRA)+l4*sin(phi_CRS))+cos(phi_CRS)*(l1+(-1)*l7*cos(phi_FRB)+(-1)*l12*cos(phi_CRW+phi_FRB)+(1/2)*r*sin(phi_CRA+(-1)*phi_CRW+(-1)*phi_FRB)+l8*sin(phi_FRB)+(-1)*l11*sin(phi_CRW+phi_FRB)+(-1/2)*r*sin(phi_CRA+phi_CRW+phi_FRB)),sin(phi_CRS)*(l11+r*cos(phi_CRA)+(-1)*l8*cos(phi_CRW)+l7*sin(phi_CRW)),0,0,0,(l11+r*cos(phi_CRA))*sin(phi_CRS),0,0,0,0,0,(-1)*r*sin(phi_CRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        cos(phi_CRS)*cos(phi_CRW+phi_FRB)*sin(phi_CRA)+cos(phi_CRA)*sin(phi_CRW+phi_FRB),sin(phi_CRA)*sin(phi_CRS),cos(phi_CRA)*cos(phi_CRW+phi_FRB)+(-1)*cos(phi_CRS)*sin(phi_CRA)*sin(phi_CRW+phi_FRB),(-1)*l4*cos(phi_CRA)*cos(phi_CRW+phi_FRB)+(-1)*sin(phi_CRA)*((-1)*l4*cos(phi_CRS)*sin(phi_CRW+phi_FRB)+sin(phi_CRS)*(l2+l8*cos(phi_FRB)+(-1)*l11*cos(phi_CRW+phi_FRB)+l7*sin(phi_FRB)+l12*sin(phi_CRW+phi_FRB))),cos(phi_CRS)*sin(phi_CRA)*((-1)*l11+l8*cos(phi_CRW)+l2*cos(phi_CRW+phi_FRB)+(-1)*l7*sin(phi_CRW)+l1*sin(phi_CRW+phi_FRB))+cos(phi_CRA)*(l12+l7*cos(phi_CRW)+(-1)*l1*cos(phi_CRW+phi_FRB)+l8*sin(phi_CRW)+l2*sin(phi_CRW+phi_FRB)),l4*cos(phi_CRS)*cos(phi_CRW+phi_FRB)*sin(phi_CRA)+l4*cos(phi_CRA)*sin(phi_CRW+phi_FRB)+sin(phi_CRA)*sin(phi_CRS)*(l1+(-1)*l7*cos(phi_FRB)+(-1)*l12*cos(phi_CRW+phi_FRB)+l8*sin(phi_FRB)+(-1)*l11*sin(phi_CRW+phi_FRB)),(-1)*cos(phi_CRS)*sin(phi_CRA)*(l11+(-1)*l8*cos(phi_CRW)+l7*sin(phi_CRW))+cos(phi_CRA)*(l12+l7*cos(phi_CRW)+l8*sin(phi_CRW)),0,0,0,l12*cos(phi_CRA)+(-1)*l11*cos(phi_CRS)*sin(phi_CRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_CRA)*cos(phi_CRS)*cos(phi_CRW+phi_FRB)+(-1)*sin(phi_CRA)*sin(phi_CRW+phi_FRB),cos(phi_CRA)*sin(phi_CRS),(-1)*cos(phi_CRW)*(cos(phi_FRB)*sin(phi_CRA)+cos(phi_CRA)*cos(phi_CRS)*sin(phi_FRB))+sin(phi_CRW)*((-1)*cos(phi_CRA)*cos(phi_CRS)*cos(phi_FRB)+sin(phi_CRA)*sin(phi_FRB)),cos(phi_CRA)*sin(phi_CRS),0,0,0,cos(phi_CRA)*sin(phi_CRS),0,0,0,0,0,(-1)*sin(phi_CRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,(-1)*cos(phi_CRW+phi_FRB)*sin(phi_CRS),cos(phi_CRS),sin(phi_CRS)*sin(phi_CRW+phi_FRB),cos(phi_CRS),0,0,0,cos(phi_CRS),0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_CRS)*cos(phi_CRW+phi_FRB)*sin(phi_CRA)+cos(phi_CRA)*sin(phi_CRW+phi_FRB),sin(phi_CRA)*sin(phi_CRS),cos(phi_CRA)*cos(phi_CRW+phi_FRB)+(-1)*cos(phi_CRS)*sin(phi_CRA)*sin(phi_CRW+phi_FRB),sin(phi_CRA)*sin(phi_CRS),0,0,0,sin(phi_CRA)*sin(phi_CRS),0,0,0,0,0,cos(phi_CRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,
    
        cos(phi_RRA)*cos(phi_RRS)*cos(phi_RRW)+(-1)*sin(phi_RRA)*sin(phi_RRW),cos(phi_RRW)*sin(phi_RB)*sin(phi_RRA)+cos(phi_RRA)*(cos(phi_RB)*sin(phi_RRS)+cos(phi_RRS)*sin(phi_RB)*sin(phi_RRW)),cos(phi_RRA)*sin(phi_RB)*sin(phi_RRS)+(-1)*cos(phi_RB)*(cos(phi_RRW)*sin(phi_RRA)+cos(phi_RRA)*cos(phi_RRS)*sin(phi_RRW)),cos(phi_RRW)*((l4+(-1)*l3*sin(phi_RB))*sin(phi_RRA)+(r+l11*cos(phi_RRA))*sin(phi_RRS))+cos(phi_RRA)*(cos(phi_RRS)*(l4+(-1)*l3*sin(phi_RB))*sin(phi_RRW)+sin(phi_RRS)*(l9+(-1)*l3*cos(phi_RB)+(-1)*l10*sin(phi_RRW))),(((-1)*l3+l4*sin(phi_RB))*sin(phi_RRA)+r*sin(phi_RB)*sin(phi_RRS))*sin(phi_RRW)+cos(phi_RRA)*(cos(phi_RRS)*cos(phi_RRW)*(l3+(-1)*l4*sin(phi_RB))+sin(phi_RB)*sin(phi_RRS)*(l5+l10*cos(phi_RRW)+l11*sin(phi_RRW)))+(-1)*cos(phi_RB)*(sin(phi_RRA)*(l10+l5*cos(phi_RRW)+(-1)*l9*sin(phi_RRW))+cos(phi_RRS)*(r+cos(phi_RRA)*(l11+l9*cos(phi_RRW)+l5*sin(phi_RRW)))),cos(phi_RB)*((-1)*(l4*sin(phi_RRA)+r*sin(phi_RRS))*sin(phi_RRW)+cos(phi_RRA)*(l4*cos(phi_RRS)*cos(phi_RRW)+(-1)*sin(phi_RRS)*(l5+l10*cos(phi_RRW)+l11*sin(phi_RRW))))+(-1)*sin(phi_RB)*(sin(phi_RRA)*(l10+l5*cos(phi_RRW)+(-1)*l9*sin(phi_RRW))+cos(phi_RRS)*(r+cos(phi_RRA)*(l11+l9*cos(phi_RRW)+l5*sin(phi_RRW)))),0,cos(phi_RRW)*(l4*sin(phi_RRA)+(r+l11*cos(phi_RRA))*sin(phi_RRS))+cos(phi_RRA)*(l4*cos(phi_RRS)*sin(phi_RRW)+sin(phi_RRS)*(l9+(-1)*l10*sin(phi_RRW))),0,0,0,(-1)*(r+l11*cos(phi_RRA))*cos(phi_RRS)+(-1)*l10*sin(phi_RRA),0,0,0,0,0,0,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        (-1)*cos(phi_RRW)*sin(phi_RRS),cos(phi_RB)*cos(phi_RRS)+(-1)*sin(phi_RB)*sin(phi_RRS)*sin(phi_RRW),cos(phi_RRS)*sin(phi_RB)+cos(phi_RB)*sin(phi_RRS)*sin(phi_RRW),(-1)*(r*sin(phi_RRA)+(l4+(-1)*l3*sin(phi_RB))*sin(phi_RRS))*sin(phi_RRW)+cos(phi_RRS)*(l9+(-1)*l3*cos(phi_RB)+(l11+r*cos(phi_RRA))*cos(phi_RRW)+(-1)*l10*sin(phi_RRW)),cos(phi_RRW)*(((-1)*l3+l9*cos(phi_RB))*sin(phi_RRS)+sin(phi_RB)*(r*sin(phi_RRA)+l4*sin(phi_RRS)))+cos(phi_RB)*sin(phi_RRS)*(l11+r*cos(phi_RRA)+l5*sin(phi_RRW))+cos(phi_RRS)*sin(phi_RB)*(l5+l10*cos(phi_RRW)+(l11+r*cos(phi_RRA))*sin(phi_RRW)),sin(phi_RB)*sin(phi_RRS)*(l11+r*cos(phi_RRA)+l9*cos(phi_RRW)+l5*sin(phi_RRW))+(-1)*cos(phi_RB)*(cos(phi_RRW)*(r*sin(phi_RRA)+l4*sin(phi_RRS))+cos(phi_RRS)*(l5+l10*cos(phi_RRW)+(l11+r*cos(phi_RRA))*sin(phi_RRW))),0,(-1)*(r*sin(phi_RRA)+l4*sin(phi_RRS))*sin(phi_RRW)+cos(phi_RRS)*(l9+(l11+r*cos(phi_RRA))*cos(phi_RRW)+(-1)*l10*sin(phi_RRW)),0,0,0,(l11+r*cos(phi_RRA))*sin(phi_RRS),0,0,0,0,0,(-1)*r*sin(phi_RRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        cos(phi_RRS)*cos(phi_RRW)*sin(phi_RRA)+cos(phi_RRA)*sin(phi_RRW),(-1)*cos(phi_RRA)*cos(phi_RRW)*sin(phi_RB)+sin(phi_RRA)*(cos(phi_RB)*sin(phi_RRS)+cos(phi_RRS)*sin(phi_RB)*sin(phi_RRW)),sin(phi_RB)*sin(phi_RRA)*sin(phi_RRS)+cos(phi_RB)*(cos(phi_RRA)*cos(phi_RRW)+(-1)*cos(phi_RRS)*sin(phi_RRA)*sin(phi_RRW)),(-1)*cos(phi_RRA)*cos(phi_RRW)*(l4+(-1)*l3*sin(phi_RB))+sin(phi_RRA)*(cos(phi_RRS)*(l4+(-1)*l3*sin(phi_RB))*sin(phi_RRW)+sin(phi_RRS)*(l9+(-1)*l3*cos(phi_RB)+l11*cos(phi_RRW)+(-1)*l10*sin(phi_RRW))),cos(phi_RRS)*cos(phi_RRW)*(l3+(-1)*l4*sin(phi_RB))*sin(phi_RRA)+l5*sin(phi_RB)*sin(phi_RRA)*sin(phi_RRS)+l10*cos(phi_RRW)*sin(phi_RB)*sin(phi_RRA)*sin(phi_RRS)+l3*cos(phi_RRA)*sin(phi_RRW)+(-1)*l4*cos(phi_RRA)*sin(phi_RB)*sin(phi_RRW)+l11*sin(phi_RB)*sin(phi_RRA)*sin(phi_RRS)*sin(phi_RRW)+cos(phi_RB)*((-1)*cos(phi_RRS)*sin(phi_RRA)*(l11+l9*cos(phi_RRW)+l5*sin(phi_RRW))+cos(phi_RRA)*(l10+l5*cos(phi_RRW)+(-1)*l9*sin(phi_RRW))),cos(phi_RRA)*(l4*cos(phi_RB)*sin(phi_RRW)+sin(phi_RB)*(l10+l5*cos(phi_RRW)+(-1)*l9*sin(phi_RRW)))+sin(phi_RRA)*((-1)*cos(phi_RRS)*sin(phi_RB)*(l11+l9*cos(phi_RRW)+l5*sin(phi_RRW))+cos(phi_RB)*(l4*cos(phi_RRS)*cos(phi_RRW)+(-1)*sin(phi_RRS)*(l5+l10*cos(phi_RRW)+l11*sin(phi_RRW)))),0,(-1)*l4*cos(phi_RRA)*cos(phi_RRW)+sin(phi_RRA)*(l4*cos(phi_RRS)*sin(phi_RRW)+sin(phi_RRS)*(l9+l11*cos(phi_RRW)+(-1)*l10*sin(phi_RRW))),0,0,0,l10*cos(phi_RRA)+(-1)*l11*cos(phi_RRS)*sin(phi_RRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_RRA)*cos(phi_RRS)*cos(phi_RRW)+(-1)*sin(phi_RRA)*sin(phi_RRW),cos(phi_RRW)*sin(phi_RB)*sin(phi_RRA)+cos(phi_RRA)*(cos(phi_RB)*sin(phi_RRS)+cos(phi_RRS)*sin(phi_RB)*sin(phi_RRW)),cos(phi_RRA)*sin(phi_RB)*sin(phi_RRS)+(-1)*cos(phi_RB)*(cos(phi_RRW)*sin(phi_RRA)+cos(phi_RRA)*cos(phi_RRS)*sin(phi_RRW)),0,cos(phi_RRA)*cos(phi_RRS)*cos(phi_RRW)+(-1)*sin(phi_RRA)*sin(phi_RRW),0,0,0,cos(phi_RRA)*sin(phi_RRS),0,0,0,0,0,(-1)*sin(phi_RRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,(-1)*cos(phi_RRW)*sin(phi_RRS),cos(phi_RB)*cos(phi_RRS)+(-1)*sin(phi_RB)*sin(phi_RRS)*sin(phi_RRW),cos(phi_RRS)*sin(phi_RB)+cos(phi_RB)*sin(phi_RRS)*sin(phi_RRW),0,(-1)*cos(phi_RRW)*sin(phi_RRS),0,0,0,cos(phi_RRS),0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_RRS)*cos(phi_RRW)*sin(phi_RRA)+cos(phi_RRA)*sin(phi_RRW),(-1)*cos(phi_RRA)*cos(phi_RRW)*sin(phi_RB)+sin(phi_RRA)*(cos(phi_RB)*sin(phi_RRS)+cos(phi_RRS)*sin(phi_RB)*sin(phi_RRW)),sin(phi_RB)*sin(phi_RRA)*sin(phi_RRS)+cos(phi_RB)*(cos(phi_RRA)*cos(phi_RRW)+(-1)*cos(phi_RRS)*sin(phi_RRA)*sin(phi_RRW)),0,cos(phi_RRS)*cos(phi_RRW)*sin(phi_RRA)+cos(phi_RRA)*sin(phi_RRW),0,0,0,sin(phi_RRA)*sin(phi_RRS),0,0,0,0,0,cos(phi_RRA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,
    
        cos(phi_RLA)*cos(phi_RLS)*cos(phi_RLW)+(-1)*sin(phi_RLA)*sin(phi_RLW),cos(phi_RLW)*sin(phi_RB)*sin(phi_RLA)+cos(phi_RLA)*(cos(phi_RB)*sin(phi_RLS)+cos(phi_RLS)*sin(phi_RB)*sin(phi_RLW)),cos(phi_RLA)*sin(phi_RB)*sin(phi_RLS)+(-1)*cos(phi_RB)*(cos(phi_RLW)*sin(phi_RLA)+cos(phi_RLA)*cos(phi_RLS)*sin(phi_RLW)),cos(phi_RLW)*((-1)*(l4+l3*sin(phi_RB))*sin(phi_RLA)+(r+l11*cos(phi_RLA))*sin(phi_RLS))+cos(phi_RLA)*((-1)*cos(phi_RLS)*(l4+l3*sin(phi_RB))*sin(phi_RLW)+sin(phi_RLS)*(l9+(-1)*l3*cos(phi_RB)+(-1)*l10*sin(phi_RLW))),(-1)*((l3+l4*sin(phi_RB))*sin(phi_RLA)+(-1)*r*sin(phi_RB)*sin(phi_RLS))*sin(phi_RLW)+cos(phi_RLA)*(cos(phi_RLS)*cos(phi_RLW)*(l3+l4*sin(phi_RB))+sin(phi_RB)*sin(phi_RLS)*(l5+l10*cos(phi_RLW)+l11*sin(phi_RLW)))+(-1)*cos(phi_RB)*(sin(phi_RLA)*(l10+l5*cos(phi_RLW)+(-1)*l9*sin(phi_RLW))+cos(phi_RLS)*(r+cos(phi_RLA)*(l11+l9*cos(phi_RLW)+l5*sin(phi_RLW)))),(-1)*cos(phi_RB)*(((-1)*l4*sin(phi_RLA)+r*sin(phi_RLS))*sin(phi_RLW)+cos(phi_RLA)*(l4*cos(phi_RLS)*cos(phi_RLW)+sin(phi_RLS)*(l5+l10*cos(phi_RLW)+l11*sin(phi_RLW))))+(-1)*sin(phi_RB)*(sin(phi_RLA)*(l10+l5*cos(phi_RLW)+(-1)*l9*sin(phi_RLW))+cos(phi_RLS)*(r+cos(phi_RLA)*(l11+l9*cos(phi_RLW)+l5*sin(phi_RLW)))),0,cos(phi_RLW)*((-1)*l4*sin(phi_RLA)+(r+l11*cos(phi_RLA))*sin(phi_RLS))+cos(phi_RLA)*((-1)*l4*cos(phi_RLS)*sin(phi_RLW)+sin(phi_RLS)*(l9+(-1)*l10*sin(phi_RLW))),0,0,0,0,(-1)*(r+l11*cos(phi_RLA))*cos(phi_RLS)+(-1)*l10*sin(phi_RLA),0,0,0,0,0,0,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        (-1)*cos(phi_RLW)*sin(phi_RLS),cos(phi_RB)*cos(phi_RLS)+(-1)*sin(phi_RB)*sin(phi_RLS)*sin(phi_RLW),cos(phi_RLS)*sin(phi_RB)+cos(phi_RB)*sin(phi_RLS)*sin(phi_RLW),((-1)*r*sin(phi_RLA)+(l4+l3*sin(phi_RB))*sin(phi_RLS))*sin(phi_RLW)+cos(phi_RLS)*(l9+(-1)*l3*cos(phi_RB)+(l11+r*cos(phi_RLA))*cos(phi_RLW)+(-1)*l10*sin(phi_RLW)),cos(phi_RLW)*(((-1)*l3+l9*cos(phi_RB))*sin(phi_RLS)+sin(phi_RB)*(r*sin(phi_RLA)+(-1)*l4*sin(phi_RLS)))+cos(phi_RB)*sin(phi_RLS)*(l11+r*cos(phi_RLA)+l5*sin(phi_RLW))+cos(phi_RLS)*sin(phi_RB)*(l5+l10*cos(phi_RLW)+(l11+r*cos(phi_RLA))*sin(phi_RLW)),sin(phi_RB)*sin(phi_RLS)*(l11+r*cos(phi_RLA)+l9*cos(phi_RLW)+l5*sin(phi_RLW))+(-1)*cos(phi_RB)*(cos(phi_RLW)*(r*sin(phi_RLA)+(-1)*l4*sin(phi_RLS))+cos(phi_RLS)*(l5+l10*cos(phi_RLW)+(l11+r*cos(phi_RLA))*sin(phi_RLW))),0,((-1)*r*sin(phi_RLA)+l4*sin(phi_RLS))*sin(phi_RLW)+cos(phi_RLS)*(l9+(l11+r*cos(phi_RLA))*cos(phi_RLW)+(-1)*l10*sin(phi_RLW)),0,0,0,0,(l11+r*cos(phi_RLA))*sin(phi_RLS),0,0,0,0,0,(-1)*r*sin(phi_RLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        cos(phi_RLS)*cos(phi_RLW)*sin(phi_RLA)+cos(phi_RLA)*sin(phi_RLW),(-1)*cos(phi_RLA)*cos(phi_RLW)*sin(phi_RB)+sin(phi_RLA)*(cos(phi_RB)*sin(phi_RLS)+cos(phi_RLS)*sin(phi_RB)*sin(phi_RLW)),sin(phi_RB)*sin(phi_RLA)*sin(phi_RLS)+cos(phi_RB)*(cos(phi_RLA)*cos(phi_RLW)+(-1)*cos(phi_RLS)*sin(phi_RLA)*sin(phi_RLW)),cos(phi_RLA)*cos(phi_RLW)*(l4+l3*sin(phi_RB))+sin(phi_RLA)*((-1)*cos(phi_RLS)*(l4+l3*sin(phi_RB))*sin(phi_RLW)+sin(phi_RLS)*(l9+(-1)*l3*cos(phi_RB)+l11*cos(phi_RLW)+(-1)*l10*sin(phi_RLW))),cos(phi_RLS)*cos(phi_RLW)*(l3+l4*sin(phi_RB))*sin(phi_RLA)+l5*sin(phi_RB)*sin(phi_RLA)*sin(phi_RLS)+l10*cos(phi_RLW)*sin(phi_RB)*sin(phi_RLA)*sin(phi_RLS)+l3*cos(phi_RLA)*sin(phi_RLW)+l4*cos(phi_RLA)*sin(phi_RB)*sin(phi_RLW)+l11*sin(phi_RB)*sin(phi_RLA)*sin(phi_RLS)*sin(phi_RLW)+cos(phi_RB)*((-1)*cos(phi_RLS)*sin(phi_RLA)*(l11+l9*cos(phi_RLW)+l5*sin(phi_RLW))+cos(phi_RLA)*(l10+l5*cos(phi_RLW)+(-1)*l9*sin(phi_RLW))),cos(phi_RLA)*((-1)*l4*cos(phi_RB)*sin(phi_RLW)+sin(phi_RB)*(l10+l5*cos(phi_RLW)+(-1)*l9*sin(phi_RLW)))+(-1)*sin(phi_RLA)*(cos(phi_RLS)*sin(phi_RB)*(l11+l9*cos(phi_RLW)+l5*sin(phi_RLW))+cos(phi_RB)*(l4*cos(phi_RLS)*cos(phi_RLW)+sin(phi_RLS)*(l5+l10*cos(phi_RLW)+l11*sin(phi_RLW)))),0,l4*cos(phi_RLA)*cos(phi_RLW)+sin(phi_RLA)*((-1)*l4*cos(phi_RLS)*sin(phi_RLW)+sin(phi_RLS)*(l9+l11*cos(phi_RLW)+(-1)*l10*sin(phi_RLW))),0,0,0,0,l10*cos(phi_RLA)+(-1)*l11*cos(phi_RLS)*sin(phi_RLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_RLA)*cos(phi_RLS)*cos(phi_RLW)+(-1)*sin(phi_RLA)*sin(phi_RLW),cos(phi_RLW)*sin(phi_RB)*sin(phi_RLA)+cos(phi_RLA)*(cos(phi_RB)*sin(phi_RLS)+cos(phi_RLS)*sin(phi_RB)*sin(phi_RLW)),cos(phi_RLA)*sin(phi_RB)*sin(phi_RLS)+(-1)*cos(phi_RB)*(cos(phi_RLW)*sin(phi_RLA)+cos(phi_RLA)*cos(phi_RLS)*sin(phi_RLW)),0,cos(phi_RLA)*cos(phi_RLS)*cos(phi_RLW)+(-1)*sin(phi_RLA)*sin(phi_RLW),0,0,0,0,cos(phi_RLA)*sin(phi_RLS),0,0,0,0,0,(-1)*sin(phi_RLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,(-1)*cos(phi_RLW)*sin(phi_RLS),cos(phi_RB)*cos(phi_RLS)+(-1)*sin(phi_RB)*sin(phi_RLS)*sin(phi_RLW),cos(phi_RLS)*sin(phi_RB)+cos(phi_RB)*sin(phi_RLS)*sin(phi_RLW),0,(-1)*cos(phi_RLW)*sin(phi_RLS),0,0,0,0,cos(phi_RLS),0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_RLS)*cos(phi_RLW)*sin(phi_RLA)+cos(phi_RLA)*sin(phi_RLW),(-1)*cos(phi_RLA)*cos(phi_RLW)*sin(phi_RB)+sin(phi_RLA)*(cos(phi_RB)*sin(phi_RLS)+cos(phi_RLS)*sin(phi_RB)*sin(phi_RLW)),sin(phi_RB)*sin(phi_RLA)*sin(phi_RLS)+cos(phi_RB)*(cos(phi_RLA)*cos(phi_RLW)+(-1)*cos(phi_RLS)*sin(phi_RLA)*sin(phi_RLW)),0,cos(phi_RLS)*cos(phi_RLW)*sin(phi_RLA)+cos(phi_RLA)*sin(phi_RLW),0,0,0,0,sin(phi_RLA)*sin(phi_RLS),0,0,0,0,0,cos(phi_RLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,
    
        cos(phi_CLA)*cos(phi_CLS)*cos(phi_CLW+phi_FLB)+(-1)*sin(phi_CLA)*sin(phi_CLW+phi_FLB),cos(phi_CLA)*sin(phi_CLS),(-1)*cos(phi_CLW)*(cos(phi_FLB)*sin(phi_CLA)+cos(phi_CLA)*cos(phi_CLS)*sin(phi_FLB))+sin(phi_CLW)*((-1)*cos(phi_CLA)*cos(phi_CLS)*cos(phi_FLB)+sin(phi_CLA)*sin(phi_FLB)),(l4*sin(phi_CLA)+(-1)*r*sin(phi_CLS))*sin(phi_CLW)*sin(phi_FLB)+(-1)*cos(phi_CLW)*(cos(phi_FLB)*(l4*sin(phi_CLA)+(-1)*(r+l11*cos(phi_CLA))*sin(phi_CLS))+cos(phi_CLA)*(l4*cos(phi_CLS)+l12*sin(phi_CLS))*sin(phi_FLB))+(-1)*cos(phi_CLA)*(l4*cos(phi_CLS)*cos(phi_FLB)*sin(phi_CLW)+sin(phi_CLS)*(l2+cos(phi_FLB)*(l8+l12*sin(phi_CLW))+(l7+l11*sin(phi_CLW))*sin(phi_FLB))),(-1)*sin(phi_CLA)*(l12+l7*cos(phi_CLW)+(-1)*l1*cos(phi_CLW+phi_FLB)+l8*sin(phi_CLW)+l2*sin(phi_CLW+phi_FLB))+cos(phi_CLS)*((-1)*r+cos(phi_CLA)*((-1)*l11+l8*cos(phi_CLW)+l2*cos(phi_CLW+phi_FLB)+(-1)*l7*sin(phi_CLW)+l1*sin(phi_CLW+phi_FLB))),(l4*sin(phi_CLA)+(-1)*r*sin(phi_CLS))*sin(phi_CLW+phi_FLB)+(-1)*cos(phi_CLA)*(l4*cos(phi_CLS)*cos(phi_CLW+phi_FLB)+sin(phi_CLS)*((-1)*l1+l7*cos(phi_FLB)+l12*cos(phi_CLW+phi_FLB)+(-1)*l8*sin(phi_FLB)+l11*sin(phi_CLW+phi_FLB))),0,0,(-1)*sin(phi_CLA)*(l12+l7*cos(phi_CLW)+l8*sin(phi_CLW))+(-1)*cos(phi_CLS)*(r+cos(phi_CLA)*(l11+(-1)*l8*cos(phi_CLW)+l7*sin(phi_CLW))),0,0,0,0,(-1)*(r+l11*cos(phi_CLA))*cos(phi_CLS)+(-1)*l12*sin(phi_CLA),0,0,0,0,0,0,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        (-1)*cos(phi_CLW+phi_FLB)*sin(phi_CLS),cos(phi_CLS),sin(phi_CLS)*sin(phi_CLW+phi_FLB),(-1)*cos(phi_CLS)*(l2+cos(phi_FLB)*(l8+(-1)*(l11+r*cos(phi_CLA))*cos(phi_CLW)+l12*sin(phi_CLW))+(l7+l12*cos(phi_CLW)+(l11+r*cos(phi_CLA))*sin(phi_CLW))*sin(phi_FLB))+(-1)*(r*sin(phi_CLA)+(-1)*l4*sin(phi_CLS))*sin(phi_CLW+phi_FLB),sin(phi_CLS)*(l11+r*cos(phi_CLA)+(-1)*l8*cos(phi_CLW)+(-1)*l2*cos(phi_CLW+phi_FLB)+l7*sin(phi_CLW)+(-1)*l1*sin(phi_CLW+phi_FLB)),cos(phi_CLW+phi_FLB)*((-1)*r*sin(phi_CLA)+l4*sin(phi_CLS))+cos(phi_CLS)*(l1+(-1)*l7*cos(phi_FLB)+(-1)*l12*cos(phi_CLW+phi_FLB)+(1/2)*r*sin(phi_CLA+(-1)*phi_CLW+(-1)*phi_FLB)+l8*sin(phi_FLB)+(-1)*l11*sin(phi_CLW+phi_FLB)+(-1/2)*r*sin(phi_CLA+phi_CLW+phi_FLB)),0,0,sin(phi_CLS)*(l11+r*cos(phi_CLA)+(-1)*l8*cos(phi_CLW)+l7*sin(phi_CLW)),0,0,0,0,(l11+r*cos(phi_CLA))*sin(phi_CLS),0,0,0,0,0,(-1)*r*sin(phi_CLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        cos(phi_CLS)*cos(phi_CLW+phi_FLB)*sin(phi_CLA)+cos(phi_CLA)*sin(phi_CLW+phi_FLB),sin(phi_CLA)*sin(phi_CLS),cos(phi_CLA)*cos(phi_CLW+phi_FLB)+(-1)*cos(phi_CLS)*sin(phi_CLA)*sin(phi_CLW+phi_FLB),l4*cos(phi_CLA)*cos(phi_CLW+phi_FLB)+(-1)*sin(phi_CLA)*(l4*cos(phi_CLS)*sin(phi_CLW+phi_FLB)+sin(phi_CLS)*(l2+l8*cos(phi_FLB)+(-1)*l11*cos(phi_CLW+phi_FLB)+l7*sin(phi_FLB)+l12*sin(phi_CLW+phi_FLB))),cos(phi_CLS)*sin(phi_CLA)*((-1)*l11+l8*cos(phi_CLW)+l2*cos(phi_CLW+phi_FLB)+(-1)*l7*sin(phi_CLW)+l1*sin(phi_CLW+phi_FLB))+cos(phi_CLA)*(l12+l7*cos(phi_CLW)+(-1)*l1*cos(phi_CLW+phi_FLB)+l8*sin(phi_CLW)+l2*sin(phi_CLW+phi_FLB)),(-1)*l4*cos(phi_CLS)*cos(phi_CLW+phi_FLB)*sin(phi_CLA)+(-1)*l4*cos(phi_CLA)*sin(phi_CLW+phi_FLB)+sin(phi_CLA)*sin(phi_CLS)*(l1+(-1)*l7*cos(phi_FLB)+(-1)*l12*cos(phi_CLW+phi_FLB)+l8*sin(phi_FLB)+(-1)*l11*sin(phi_CLW+phi_FLB)),0,0,(-1)*cos(phi_CLS)*sin(phi_CLA)*(l11+(-1)*l8*cos(phi_CLW)+l7*sin(phi_CLW))+cos(phi_CLA)*(l12+l7*cos(phi_CLW)+l8*sin(phi_CLW)),0,0,0,0,l12*cos(phi_CLA)+(-1)*l11*cos(phi_CLS)*sin(phi_CLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_CLA)*cos(phi_CLS)*cos(phi_CLW+phi_FLB)+(-1)*sin(phi_CLA)*sin(phi_CLW+phi_FLB),cos(phi_CLA)*sin(phi_CLS),(-1)*cos(phi_CLW)*(cos(phi_FLB)*sin(phi_CLA)+cos(phi_CLA)*cos(phi_CLS)*sin(phi_FLB))+sin(phi_CLW)*((-1)*cos(phi_CLA)*cos(phi_CLS)*cos(phi_FLB)+sin(phi_CLA)*sin(phi_FLB)),0,0,cos(phi_CLA)*sin(phi_CLS),0,0,0,0,cos(phi_CLA)*sin(phi_CLS),0,0,0,0,0,(-1)*sin(phi_CLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,(-1)*cos(phi_CLW+phi_FLB)*sin(phi_CLS),cos(phi_CLS),sin(phi_CLS)*sin(phi_CLW+phi_FLB),0,0,cos(phi_CLS),0,0,0,0,cos(phi_CLS),0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_CLS)*cos(phi_CLW+phi_FLB)*sin(phi_CLA)+cos(phi_CLA)*sin(phi_CLW+phi_FLB),sin(phi_CLA)*sin(phi_CLS),cos(phi_CLA)*cos(phi_CLW+phi_FLB)+(-1)*cos(phi_CLS)*sin(phi_CLA)*sin(phi_CLW+phi_FLB),0,0,sin(phi_CLA)*sin(phi_CLS),0,0,0,0,sin(phi_CLA)*sin(phi_CLS),0,0,0,0,0,cos(phi_CLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,
    
        cos(phi_FLA)*cos(phi_FLS)*cos(phi_FLB+phi_FLW)+(-1)*sin(phi_FLA)*sin(phi_FLB+phi_FLW),cos(phi_FLA)*sin(phi_FLS),(-1)*cos(phi_FLB)*(cos(phi_FLW)*sin(phi_FLA)+cos(phi_FLA)*cos(phi_FLS)*sin(phi_FLW))+sin(phi_FLB)*((-1)*cos(phi_FLA)*cos(phi_FLS)*cos(phi_FLW)+sin(phi_FLA)*sin(phi_FLW)),sin(phi_FLB)*(l4*sin(phi_FLA)+(-1)*r*sin(phi_FLS))*sin(phi_FLW)+cos(phi_FLB)*(cos(phi_FLW)*((-1)*l4*sin(phi_FLA)+(r+l11*cos(phi_FLA))*sin(phi_FLS))+(-1)*cos(phi_FLA)*(l4*cos(phi_FLS)*sin(phi_FLW)+sin(phi_FLS)*(l8+(-1)*l10*sin(phi_FLW))))+(-1)*cos(phi_FLA)*(l4*cos(phi_FLS)*cos(phi_FLW)*sin(phi_FLB)+sin(phi_FLS)*(l2+(-1)*sin(phi_FLB)*(l6+l10*cos(phi_FLW)+(-1)*l11*sin(phi_FLW)))),sin(phi_FLA)*(l10+l6*cos(phi_FLW)+l1*cos(phi_FLB+phi_FLW)+(-1)*l8*sin(phi_FLW)+(-1)*l2*sin(phi_FLB+phi_FLW))+cos(phi_FLS)*((-1)*r+cos(phi_FLA)*((-1)*l11+l8*cos(phi_FLW)+l2*cos(phi_FLB+phi_FLW)+l6*sin(phi_FLW)+l1*sin(phi_FLB+phi_FLW))),cos(phi_FLA)*(l4*cos(phi_FLS)*sin(phi_FLB)*sin(phi_FLW)+sin(phi_FLS)*(l1+sin(phi_FLB)*(l8+(-1)*l11*cos(phi_FLW)+(-1)*l10*sin(phi_FLW)))+cos(phi_FLB)*((-1)*l4*cos(phi_FLS)*cos(phi_FLW)+sin(phi_FLS)*(l6+l10*cos(phi_FLW)+(-1)*l11*sin(phi_FLW))))+(l4*sin(phi_FLA)+(-1)*r*sin(phi_FLS))*sin(phi_FLB+phi_FLW),0,0,sin(phi_FLA)*(l10+l6*cos(phi_FLW)+(-1)*l8*sin(phi_FLW))+cos(phi_FLS)*((-1)*r+cos(phi_FLA)*((-1)*l11+l8*cos(phi_FLW)+l6*sin(phi_FLW))),0,0,0,0,0,(-1)*(r+l11*cos(phi_FLA))*cos(phi_FLS)+l10*sin(phi_FLA),0,0,0,0,0,0,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1)*r,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        (-1)*cos(phi_FLB+phi_FLW)*sin(phi_FLS),cos(phi_FLS),sin(phi_FLS)*sin(phi_FLB+phi_FLW),((-1)*r*sin(phi_FLA)+l4*sin(phi_FLS))*sin(phi_FLB+phi_FLW)+cos(phi_FLS)*((-1)*l2+(-1)*l8*cos(phi_FLB)+(1/2)*r*cos(phi_FLA+(-1)*phi_FLB+(-1)*phi_FLW)+l11*cos(phi_FLB+phi_FLW)+(1/2)*r*cos(phi_FLA+phi_FLB+phi_FLW)+l6*sin(phi_FLB)+l10*sin(phi_FLB+phi_FLW)),sin(phi_FLS)*(l11+r*cos(phi_FLA)+(-1)*l8*cos(phi_FLW)+(-1)*l2*cos(phi_FLB+phi_FLW)+(-1)*l6*sin(phi_FLW)+(-1)*l1*sin(phi_FLB+phi_FLW)),cos(phi_FLB+phi_FLW)*((-1)*r*sin(phi_FLA)+l4*sin(phi_FLS))+cos(phi_FLS)*(l1+l6*cos(phi_FLB)+l10*cos(phi_FLB+phi_FLW)+l8*sin(phi_FLB)+(1/2)*r*sin(phi_FLA+(-1)*phi_FLB+(-1)*phi_FLW)+(-1)*l11*sin(phi_FLB+phi_FLW)+(-1/2)*r*sin(phi_FLA+phi_FLB+phi_FLW)),0,0,sin(phi_FLS)*(l11+r*cos(phi_FLA)+(-1)*l8*cos(phi_FLW)+(-1)*l6*sin(phi_FLW)),0,0,0,0,0,(l11+r*cos(phi_FLA))*sin(phi_FLS),0,0,0,0,0,(-1)*r*sin(phi_FLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        cos(phi_FLB)*(cos(phi_FLS)*cos(phi_FLW)*sin(phi_FLA)+cos(phi_FLA)*sin(phi_FLW))+sin(phi_FLB)*(cos(phi_FLA)*cos(phi_FLW)+(-1)*cos(phi_FLS)*sin(phi_FLA)*sin(phi_FLW)),sin(phi_FLA)*sin(phi_FLS),cos(phi_FLA)*cos(phi_FLB+phi_FLW)+(-1)*cos(phi_FLS)*sin(phi_FLA)*sin(phi_FLB+phi_FLW),l4*cos(phi_FLA)*cos(phi_FLB+phi_FLW)+sin(phi_FLA)*((-1)*l4*cos(phi_FLS)*sin(phi_FLB+phi_FLW)+sin(phi_FLS)*((-1)*l2+(-1)*l8*cos(phi_FLB)+l11*cos(phi_FLB+phi_FLW)+l6*sin(phi_FLB)+l10*sin(phi_FLB+phi_FLW))),cos(phi_FLS)*sin(phi_FLA)*((-1)*l11+l8*cos(phi_FLW)+l2*cos(phi_FLB+phi_FLW)+l6*sin(phi_FLW)+l1*sin(phi_FLB+phi_FLW))+(-1)*cos(phi_FLA)*(l10+l6*cos(phi_FLW)+l1*cos(phi_FLB+phi_FLW)+(-1)*l8*sin(phi_FLW)+(-1)*l2*sin(phi_FLB+phi_FLW)),(-1)*l4*cos(phi_FLA)*cos(phi_FLW)*sin(phi_FLB)+(-1)*cos(phi_FLB)*(l4*cos(phi_FLS)*cos(phi_FLW)*sin(phi_FLA)+l4*cos(phi_FLA)*sin(phi_FLW)+(-1)*sin(phi_FLA)*sin(phi_FLS)*(l6+l10*cos(phi_FLW)+(-1)*l11*sin(phi_FLW)))+sin(phi_FLA)*(l4*cos(phi_FLS)*sin(phi_FLB)*sin(phi_FLW)+sin(phi_FLS)*(l1+sin(phi_FLB)*(l8+(-1)*l11*cos(phi_FLW)+(-1)*l10*sin(phi_FLW)))),0,0,cos(phi_FLS)*sin(phi_FLA)*((-1)*l11+l8*cos(phi_FLW)+l6*sin(phi_FLW))+(-1)*cos(phi_FLA)*(l10+l6*cos(phi_FLW)+(-1)*l8*sin(phi_FLW)),0,0,0,0,0,(-1)*l10*cos(phi_FLA)+(-1)*l11*cos(phi_FLS)*sin(phi_FLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,cos(phi_FLA)*cos(phi_FLS)*cos(phi_FLB+phi_FLW)+(-1)*sin(phi_FLA)*sin(phi_FLB+phi_FLW),cos(phi_FLA)*sin(phi_FLS),(-1)*cos(phi_FLB)*(cos(phi_FLW)*sin(phi_FLA)+cos(phi_FLA)*cos(phi_FLS)*sin(phi_FLW))+sin(phi_FLB)*((-1)*cos(phi_FLA)*cos(phi_FLS)*cos(phi_FLW)+sin(phi_FLA)*sin(phi_FLW)),0,0,cos(phi_FLA)*sin(phi_FLS),0,0,0,0,0,cos(phi_FLA)*sin(phi_FLS),0,0,0,0,0,(-1)*sin(phi_FLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,0,0,0,0,0,0,
    
        0,0,0,(-1)*cos(phi_FLB+phi_FLW)*sin(phi_FLS),cos(phi_FLS),sin(phi_FLS)*sin(phi_FLB+phi_FLW),0,0,cos(phi_FLS),0,0,0,0,0,cos(phi_FLS),0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1),0,0,0,0,0,0,
    
        0,0,0,cos(phi_FLB)*(cos(phi_FLS)*cos(phi_FLW)*sin(phi_FLA)+cos(phi_FLA)*sin(phi_FLW))+sin(phi_FLB)*(cos(phi_FLA)*cos(phi_FLW)+(-1)*cos(phi_FLS)*sin(phi_FLA)*sin(phi_FLW)),sin(phi_FLA)*sin(phi_FLS),cos(phi_FLA)*cos(phi_FLB+phi_FLW)+(-1)*cos(phi_FLS)*sin(phi_FLA)*sin(phi_FLB+phi_FLW),0,0,sin(phi_FLA)*sin(phi_FLS),0,0,0,0,0,sin(phi_FLA)*sin(phi_FLS),0,0,0,0,0,cos(phi_FLA),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,(-1);
    // clang-format on

    return Jacobian;
}

//======================================================
std::string ExoterKinematicModel::name ()
{
        return this->model_name;
}

//==============================================================
void ExoterKinematicModel::contactPointsPerTree (std::vector<unsigned int> &contactPoints)
{
    /** Exoter has the same number of contact points per tree (one per Wheel) **/
    contactPoints.resize(this->number_wheels);
    for (std::vector<unsigned int>::iterator it = contactPoints.begin() ; it != contactPoints.end(); ++it)
    {
        *it = 1;
    }

    return;
}
//==================================
void ExoterKinematicModel::setPointsInContact (const std::vector<int> &pointsInContact)
{
        this->contact_idx = pointsInContact;
}

void ExoterKinematicModel::computeRates(const std::vector<double> &joint_positions,
                                        const std::map<int,double> &known_body_rates, const std::map<int,double> &known_joint_rates,
                                        std::map<int,double> &body_rates, std::map<int,double> &joint_rates)
{
    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, MODEL_DOF> J = jacobianSolver(joint_positions);
    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> A;
    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> B;

    rearrangeKinematicMatrices(known_body_rates, known_joint_rates, J, A, B);

    // Assemble known rates vector.
    Eigen::VectorXd xi;
    Eigen::VectorXd omega;

    int current_index = 0;
    int number_of_known_body_rates = known_body_rates.size();
    int number_of_known_joint_rates = known_joint_rates.size();

    omega.resize(number_of_known_body_rates + number_of_known_joint_rates);

    for (std::map<int,double>::const_iterator it = known_body_rates.begin(); it != known_body_rates.end(); it++, current_index++)
        omega(current_index) = it->second;

    for (std::map<int,double>::const_iterator it = known_joint_rates.begin(); it != known_joint_rates.end(); it++, current_index++)
        omega(current_index) = it->second;

    // Solve using normal equations.
    xi = (A.transpose() * A).ldlt().solve(A.transpose() * B * omega);

    // Assemble output maps.
    body_rates.clear();
    joint_rates.clear();

    int known_index = 0;
    int unknown_index = 0;

    for (unsigned int i = 0; i < 6; i++)
    {
        if (known_body_rates.find(i) != known_body_rates.end())
            body_rates.insert(std::pair<int,double>(i,omega(known_index++)));
        else
            body_rates.insert(std::pair<int,double>(i,xi(unknown_index++)));
    }

    for (unsigned int i = 0; i < MODEL_DOF - 6; i++)
    {
        if (known_joint_rates.find(i) != known_joint_rates.end())
            joint_rates.insert(std::pair<int,double>(i,omega(known_index++)));
        else
            joint_rates.insert(std::pair<int,double>(i,xi(unknown_index++)));
    }

    std::cout << "known rates: " << omega << std::endl;
    std::cout << "unknown rates: " << xi << std::endl;
    std::cout << std::endl;
}

void ExoterKinematicModel::rearrangeKinematicMatrices(const std::map<int,double> &known_body_rates, const std::map<int,double> &known_joint_rates,
                                                      const Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, MODEL_DOF> &J,
                                                      Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &A, Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &B)
{
    int number_of_known_rates = known_body_rates.size() + known_joint_rates.size();
    int number_of_unknown_rates = MODEL_DOF - number_of_known_rates;

    A.resize(6 * NUMBER_OF_WHEELS, number_of_unknown_rates);
    B.resize(6 * NUMBER_OF_WHEELS, number_of_known_rates);

    int known_index = 0;
    int unknown_index = 0;

    for (unsigned int i = 0; i < 6; i++)
    {
        if (known_body_rates.find(i) != known_body_rates.end())
            B.col(known_index++) = J.col(i);
        else
            A.col(unknown_index++) = -1 * J.col(i);
    }

    for (unsigned int i = 0; i < MODEL_DOF - 6; i++)
    {
        if (known_joint_rates.find(i) != known_joint_rates.end())
            B.col(known_index++) = J.col(i + 6);
        else
            A.col(unknown_index++) = -1 * J.col(i + 6);
    }
}
