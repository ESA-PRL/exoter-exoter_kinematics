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
			trans_BC(0) = cos(passive_angle) * (chainIdx == FL || chainIdx == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL + cos(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius + sin(walking_angle) * cos(contact_angle) * this->wheelRadius + sin(walking_angle) * LEG_LENGTH + FRONT_PASSIVE_TO_VIRTUAL;
			trans_BC(1) = -sin(steering_angle) * sin(contact_angle) * this->wheelRadius + (chainIdx == FL || chainIdx == ML ? 1 : -1) * CENTERLINE_TO_STEERING;
			trans_BC(2) = sin(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius - cos(walking_angle) * cos(contact_angle) * this->wheelRadius - cos(walking_angle) * LEG_LENGTH - VIRTUAL_TO_WALKING + sin(passive_angle) * (chainIdx == FL || chainIdx == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL;
			break;
		case RL:
		case RR:
            passive_angle = positions[PASSIVE];
            trans_BC(0) = cos(walking_angle) * cos(steering_angle) * sin(contact_angle) * this->wheelRadius + sin(walking_angle) * cos(contact_angle) * this->wheelRadius + sin(walking_angle) * LEG_LENGTH - WALKING_AXES_DISTANCE;
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
    if (positions.size() == ExoterKinematicModel::MODEL_DOF)
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
					chainpositions[PASSIVE] = positions[i%2];															// passive				
					chainpositions[STEERING] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + i];		// steering
					break;
				case ML:
				case MR:
					chainpositions[PASSIVE] = positions[i%2];															// passive
					chainpositions[STEERING] = 0.0d;																	// steering
					break;
				case RL:
				case RR:
					chainpositions[PASSIVE] = positions[2];																// passive
					chainpositions[STEERING] = positions[NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS - 2 + i];	// steering
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
	
    double passive_angle;
    double walking_angle;
    double steering_angle;
    //double rolling_angle;     // Jacobian is independent of rolling angle.
    //double roll_slip;         // Currently not taken into account.
    //double side_slip;         // Currently not taken into account.
    //double turn_slip;         // Currently not taken into account.
    double contact_angle;
	
    int passive_index;
    int walking_index;
    int steering_index;
    int rolling_index;
    int roll_slip_index;
    int side_slip_index;
    int turn_slip_index;
    int contact_index;
	
	for (int i = 0; i < static_cast<int>(this->number_wheels); ++i)
	{	
        passive_angle = 0.0d;
        walking_angle = 0.0d;
        steering_angle = 0.0d;
//      roll_slip = 0.0d;         // Currently not taken into account.
//      side_slip = 0.0d;         // Currently not taken into account.
//      turn_slip = 0.0d;         // Currently not taken into account.
        contact_angle = 0.0d;

        passive_index = -1;
        walking_index = -1;
        steering_index = -1;
        rolling_index = -1;
        roll_slip_index = -1;
        side_slip_index = -1;
        turn_slip_index = -1;
        contact_index = -1;

		switch (i)
		{
			case FL:
			case FR:
				steering_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + i;
                steering_angle = -positions[steering_index];
			case ML:
			case MR:
				passive_index = i%2;
				passive_angle = -positions[passive_index];
				break;
			case RL:
			case RR:
				passive_index = 2;
				passive_angle = positions[passive_index];
				steering_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS - 2 + i;
                steering_angle = -positions[steering_index];
				break;
		}
		
		walking_index = NUMBER_OF_PASSIVE_JOINTS + i;
        rolling_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i;
        roll_slip_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * i;
        side_slip_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * i + 1;
        turn_slip_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * i + 2;
		contact_index = NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + NUMBER_OF_WHEELS + 3 * NUMBER_OF_WHEELS + i;		
				
        walking_angle = -positions[walking_index];
        //roll_slip = positions[roll_slip_index];
        //side_slip = positions[side_slip_index];
        //turn_slip = positions[turn_slip_index];
        contact_angle = -positions[contact_index];
		
		switch (i)
		{
			case FL:
			case FR:     
                Jacobian(i*6+X_DOT,steering_index) = -(cos(walking_angle) * (i == FL ? 1 : -1) * CENTERLINE_TO_STEERING);
                Jacobian(i*6+Y_DOT,steering_index) = -(sin(walking_angle) * (VIRTUAL_TO_WALKING - sin(passive_angle) * FRONT_PASSIVE_TO_VIRTUAL) - cos(walking_angle) * (cos(passive_angle) * FRONT_PASSIVE_TO_VIRTUAL + BODY_TO_FRONT));
                Jacobian(i*6+Z_DOT,steering_index) = -(sin(walking_angle) * (i == FL ? 1 : -1) * CENTERLINE_TO_STEERING);
			case ML:
			case MR:
                Jacobian(i*6+X_DOT,passive_index) = -(sin(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL);
                Jacobian(i*6+X_DOT,walking_index) = -(VIRTUAL_TO_WALKING - sin(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL);
                Jacobian(i*6+X_DOT,rolling_index) = this->wheelRadius * (cos(walking_angle) * cos(steering_angle) * cos(contact_angle) - sin(walking_angle) * sin(contact_angle));
                Jacobian(i*6+X_DOT,roll_slip_index) = 0;
                Jacobian(i*6+X_DOT,side_slip_index) = 0;
                Jacobian(i*6+X_DOT,turn_slip_index) = 0;
                Jacobian(i*6+X_DOT,contact_index) = -(cos(steering_angle) * (cos(walking_angle) * LEG_LENGTH - sin(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL + VIRTUAL_TO_WALKING) + sin(walking_angle) * sin(steering_angle) * (i == FL || i == ML ? 1 : -1) * CENTERLINE_TO_STEERING);
                Jacobian(i*6+Y_DOT,passive_index) = 0;
                Jacobian(i*6+Y_DOT,walking_index) = 0;
                Jacobian(i*6+Y_DOT,rolling_index) = -sin(steering_angle) * cos(contact_angle) * this->wheelRadius;
                Jacobian(i*6+Y_DOT,roll_slip_index) = 0;
                Jacobian(i*6+Y_DOT,side_slip_index) = 0;
                Jacobian(i*6+Y_DOT,turn_slip_index) = 0;
                Jacobian(i*6+Y_DOT,contact_index) = -(-sin(steering_angle) * (LEG_LENGTH + sin(walking_angle) * (BODY_TO_FRONT + cos(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL) + cos(walking_angle) * (VIRTUAL_TO_WALKING - sin(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL)));
                Jacobian(i*6+Z_DOT,passive_index) = -(-cos(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL);
                Jacobian(i*6+Z_DOT,walking_index) = -(cos(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL + BODY_TO_FRONT);
                Jacobian(i*6+Z_DOT,rolling_index) = this->wheelRadius * (cos(walking_angle) * sin(contact_angle) + sin(walking_angle) * cos(steering_angle) * cos(contact_angle));
                Jacobian(i*6+Z_DOT,roll_slip_index) = 0;
                Jacobian(i*6+Z_DOT,side_slip_index) = 0;
                Jacobian(i*6+Z_DOT,turn_slip_index) = 0;
                Jacobian(i*6+Z_DOT,contact_index) = -(cos(steering_angle) * (BODY_TO_FRONT + cos(passive_angle) * (i == FL || i == FR ? 1 : -1) * FRONT_PASSIVE_TO_VIRTUAL + sin(walking_angle) * LEG_LENGTH) - sin(steering_angle) * cos(walking_angle) * (i == FL || i == ML ? 1 : -1) * CENTERLINE_TO_STEERING);
				break;
			case RL:
			case RR:
                Jacobian(i*6+X_DOT,passive_index) = 0;
                Jacobian(i*6+X_DOT,walking_index) = -(VIRTUAL_TO_WALKING - sin(passive_angle) * (i == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL);
                Jacobian(i*6+X_DOT,steering_index) = -(cos(walking_angle) * (i == RL ? 1 : -1) * (cos(passive_angle) * REAR_PASSIVE_TO_VIRTUAL + REAR_VIRTUAL_TO_STEERING));
                Jacobian(i*6+X_DOT,rolling_index) = this->wheelRadius * (cos(walking_angle) * cos(steering_angle) * cos(contact_angle) - sin(walking_angle) * sin(contact_angle));
                Jacobian(i*6+X_DOT,roll_slip_index) = 0;
                Jacobian(i*6+X_DOT,side_slip_index) = 0;
                Jacobian(i*6+X_DOT,turn_slip_index) = 0;
                Jacobian(i*6+X_DOT,contact_index) = -(sin(walking_angle) * sin(steering_angle) * (i == RL ? 1 : -1) * (cos(passive_angle) * REAR_PASSIVE_TO_VIRTUAL + REAR_VIRTUAL_TO_STEERING) + cos(steering_angle) * (cos(walking_angle) * LEG_LENGTH - sin(passive_angle) * (i == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL + VIRTUAL_TO_WALKING));
                Jacobian(i*6+Y_DOT,passive_index) = sin(passive_angle) * (i == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL;
                Jacobian(i*6+Y_DOT,walking_index) = 0;
                Jacobian(i*6+Y_DOT,steering_index) = -(sin(walking_angle) * (VIRTUAL_TO_WALKING - sin(passive_angle) * (i == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL) + cos(walking_angle) * BODY_TO_REAR);
                Jacobian(i*6+Y_DOT,rolling_index) = -sin(steering_angle) * cos(contact_angle) * this->wheelRadius;
                Jacobian(i*6+Y_DOT,roll_slip_index) = 0;
                Jacobian(i*6+Y_DOT,side_slip_index) = 0;
                Jacobian(i*6+Y_DOT,turn_slip_index) = 0;
                Jacobian(i*6+Y_DOT,contact_index) = -(sin(steering_angle) * (sin(walking_angle) * BODY_TO_REAR + cos(walking_angle) * (sin(passive_angle) * (i == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL - VIRTUAL_TO_WALKING) - LEG_LENGTH));
                Jacobian(i*6+Z_DOT,passive_index) = -cos(passive_angle) * (i == RL ? 1 : -1) * REAR_PASSIVE_TO_VIRTUAL;
                Jacobian(i*6+Z_DOT,walking_index) = BODY_TO_REAR;
                Jacobian(i*6+Z_DOT,steering_index) = -(sin(walking_angle) * (i == RL ? 1 : -1) * (cos(passive_angle) * REAR_PASSIVE_TO_VIRTUAL + REAR_VIRTUAL_TO_STEERING));
                Jacobian(i*6+Z_DOT,rolling_index) = this->wheelRadius * (cos(walking_angle) * sin(contact_angle) + sin(walking_angle) * cos(steering_angle) * cos(contact_angle));
                Jacobian(i*6+Z_DOT,roll_slip_index) = 0;
                Jacobian(i*6+Z_DOT,side_slip_index) = 0;
                Jacobian(i*6+Z_DOT,turn_slip_index) = 0;
                Jacobian(i*6+Z_DOT,contact_index) = -(cos(steering_angle) * (sin(walking_angle) * LEG_LENGTH - BODY_TO_REAR) - cos(walking_angle) * sin(steering_angle) * (i == RL ? 1 : -1) * (cos(passive_angle) * REAR_PASSIVE_TO_VIRTUAL + REAR_VIRTUAL_TO_STEERING));
				break;
		}
		
		switch (i)
		{
			case FL:
			case FR:
			case RL:
			case RR:
                Jacobian(i*6+PHI_X_DOT,steering_index) = sin(walking_angle);
                Jacobian(i*6+PHI_Z_DOT,steering_index) = -cos(walking_angle);
			case ML:
			case MR:
                Jacobian(i*6+PHI_X_DOT,contact_index) = -(sin(steering_angle) * cos(walking_angle));
                Jacobian(i*6+PHI_Y_DOT,walking_index) = -1;
                Jacobian(i*6+PHI_Y_DOT,contact_index) = -(cos(steering_angle));
                Jacobian(i*6+PHI_Z_DOT,contact_index) = -(sin(steering_angle) * sin(walking_angle));
				break;
		}
	}

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
    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, 6> E;

    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
        E.block<6,6>(6*i,0) = Eigen::Matrix<double,6,6>::Identity();

    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> A;
    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> B;

    rearrangeKinematicMatrices(known_body_rates, known_joint_rates, E, J, A, B);

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

    for (int i = 0; i < 6; i++)
    {
        if (known_body_rates.find(i) != known_body_rates.end())
            body_rates.insert(std::pair<int,double>(i,omega(known_index++)));
        else
            body_rates.insert(std::pair<int,double>(i,xi(unknown_index++)));
    }

    for (unsigned int i = 0; i < MODEL_DOF; i++)
    {
        if (known_joint_rates.find(i) != known_joint_rates.end())
            joint_rates.insert(std::pair<int,double>(i,omega(known_index++)));
        else
            joint_rates.insert(std::pair<int,double>(i,xi(unknown_index++)));
    }
}

void ExoterKinematicModel::rearrangeKinematicMatrices(const std::map<int,double> &known_body_rates, const std::map<int,double> &known_joint_rates,
                                                      const Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, 6> &E, const Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, MODEL_DOF> &J,
                                                      Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &A, Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &B)
{
    int number_of_known_rates = known_body_rates.size() + known_joint_rates.size();
    int number_of_unknown_rates = MODEL_DOF + 6 - number_of_known_rates;

    A.resize(6 * NUMBER_OF_WHEELS, number_of_unknown_rates);
    B.resize(6 * NUMBER_OF_WHEELS, number_of_known_rates);

    int current_col = 0;

    for (std::map<int,double>::const_iterator it = known_body_rates.begin(); it != known_body_rates.end(); it++, current_col++)
        B.col(current_col) = -1 * E.col(it->first);

    for (std::map<int,double>::const_iterator it = known_joint_rates.begin(); it != known_joint_rates.end(); it++, current_col++)
        B.col(current_col) = J.col(it->first);

    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> E_temp = E;
    Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> J_temp = J;

    removeMatrixColumns(E_temp, known_body_rates);
    removeMatrixColumns(J_temp, known_joint_rates);

    A.block(0,0,E_temp.rows(),E_temp.cols()) = E_temp;
    A.block(0,E_temp.cols(),J_temp.rows(),J_temp.cols()) = -1 * J_temp;
}

void ExoterKinematicModel::removeMatrixColumns(Eigen::Matrix<double, 6 * NUMBER_OF_WHEELS, Eigen::Dynamic> &matrix, const std::map<int,double> columns_to_remove)
{
    for (std::map<int,double>::const_reverse_iterator rit = columns_to_remove.rbegin(); rit != columns_to_remove.rend(); rit++)
    {
        matrix.block(0, rit->first, matrix.rows(), matrix.cols() - rit->first - 1) = matrix.block(0, rit->first + 1, matrix.rows(), matrix.cols() - rit->first - 1);
        matrix.conservativeResize(matrix.rows(), matrix.cols() - 1);
    }
}
