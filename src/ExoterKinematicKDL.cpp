#include "ExoterKinematicKDL.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace exoter_kinematics;

ExoterKinematicKDL::ExoterKinematicKDL (std::string &urdf_file, const double wheel_radius)
{
    std::string xml_string;
    const char * urdf_char = urdf_file.c_str();
    std::fstream xml_file(urdf_char, std::fstream::in);
    while ( xml_file.good() )
    {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
    }
    xml_file.close();

    /** Take the wheel radius **/
    this->wheelRadius = wheel_radius;

    /** Check the number of wheels with the template value **/
    this->number_wheels = static_cast<unsigned int>(this->getNumberOfTrees());

    /** Size propertly the jacobian matrices. One per wheel/One foot per wheel **/
    wheelJacobian.resize(number_wheels);

    boost::shared_ptr<urdf::ModelInterface> robot = urdf::parseURDF(xml_string);
    if (!robot)
    {
        throw std::runtime_error("[EXOTER_KDL_MODEL] Constructor could not parse URDF model\n");
    }

    /** Assign an information string name **/
    this->model_name = robot->getName();

    LOG_INFO("[EXOTER_KDL_MODEL] Robot name is: %s\n", this->model_name.c_str());
    LOG_INFO("[EXOTER_KDL_MODEL] Robot has %d number of Trees\n",this->number_wheels);

    /* get root link*/
    boost::shared_ptr<const urdf::Link> root_link = robot->getRoot();
    if (!root_link)
        throw std::runtime_error("[EXOTER_KDL_MODEL] Constructor could not find Root link\n");

    LOG_INFO("[EXOTER_KDL_MODEL] Robot Link: %s has %d child(ren)\n", root_link->name.c_str(), root_link->child_links.size());

    if (!kdl_parser::treeFromUrdfModel(*robot, this->tree))
        throw std::runtime_error("[EXOTER_KDL_MODEL] Constructor could not extract KDL tree\n");

    KDL::SegmentMap::const_iterator root = this->tree.getRootSegment();
    #ifdef DEBUG_PRINTS
    std::cout << " ======================================" << std::endl;
    std::cout << " Original Tree has " << this->tree.getNrOfSegments() << " link(s) and "<< this->tree.getNrOfJoints() <<" Joint(s)\n";
    std::cout << " ======================================" << std::endl;
    #endif

    for (register unsigned int j=0; j<this->number_wheels; ++j)
    {
        std::stringstream chainidx;
        chainidx << j;
        #ifdef DEBUG_PRINTS
        std::cout<<"[EXOTER_KDL_MODEL] chainname: "<< chainidx.str()<<"\n";
        #endif

        KDL::Chain chain;//!Create the chain

        /** Create the contact angle and the slip vector/displacement subchain **/
        chain.addSegment(KDL::Segment(chainidx.str()+"_CONTACT", KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-wheelRadius))));//Contact angle
        chain.addSegment(KDL::Segment(chainidx.str()+"_MOVE",KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));//Translation along X-axis
        chain.addSegment(KDL::Segment(chainidx.str()+"_SLIPX",KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));//Slip vector along X-axis
        chain.addSegment(KDL::Segment(chainidx.str()+"_SLIPY",KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));//Slip vector along Y-axis
        chain.addSegment(KDL::Segment(chainidx.str()+"_SLIPZ",KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));//Slip vector along Z-axis

        /** Add to the end of the selected chain **/
        std::string endChain;
        switch (j)
        {
            case 1:
                 endChain = getContactPoint(root->second.children[1], 0);
                 break;
            case 2:
                 endChain = getContactPoint(root->second.children[0], 1);
                 break;
            case 3:
                 endChain = getContactPoint(root->second.children[1], 1);
                 break;
            case 4:
                 endChain = getContactPoint(root->second.children[2], 0);
                 break;
            case 5:
                 endChain = getContactPoint(root->second.children[2], 1);
                 break;
            default:
                 endChain = getContactPoint(root->second.children[0], 0);
                 break;

        }
        bool exit_value = tree.addChain(chain, endChain);
        if(!exit_value)
        {
            LOG_INFO("[EXOTER_KDL_MODEL] ERROR in Constructor\n");
        }
        LOG_INFO("[EXOTER_KDL_MODEL] Odometry Kinematics Chain has: %d new more segments\n", chain.getNrOfSegments());


    }

    #ifdef DEBUG_PRINTS
    /** Walk through tree **/
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << this->tree.getNrOfSegments() << " link(s) and "<< this->tree.getNrOfJoints() <<" Joint(s)\n";
    std::cout << " Root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    printLink(root, "");
    #endif
}

ExoterKinematicKDL::~ExoterKinematicKDL()
{
}

std::string ExoterKinematicKDL::name ()
{
        return this->model_name;
}

void ExoterKinematicKDL::contactPointsPerTree (std::vector<unsigned int> &contactPoints)
{
    /** Exoter has the same number of contact points per tree (one per Wheel) **/
    contactPoints.resize(this->number_wheels);
    for (std::vector<unsigned int>::iterator it = contactPoints.begin() ; it != contactPoints.end(); ++it)
    {
        *it = 1;
    }

    return;
}

void ExoterKinematicKDL::setPointsInContact (const std::vector<int> &pointsInContact)
{
        this->contact_idx = pointsInContact;
}

void ExoterKinematicKDL::fkBody2ContactPointt(const int chainIdx, const std::vector<double> &positions,  Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov)
{
    /** Check if the number of values is correct (Since a priori we don't know
     * which kinematics chain is, the maxchainDOF is needed) **/
    if (positions.size() == this->getMaxChainDoF())
    {
        KDL::SegmentMap::const_iterator root = this->tree.getRootSegment();

        if (chainIdx > exoter_kinematics::NUMBER_OF_WHEELS)
        {
            fkTrans.matrix() = std::numeric_limits<double>::quiet_NaN() * Eigen::Matrix<double, 4, 4>::Identity();

            /** No uncertainty provided **/
            fkCov = std::numeric_limits<double>::quiet_NaN() * base::Matrix6d::Identity();

            return;
        }
        else
        {
            std::string contactPoint;
            switch (chainIdx)
            {
                case 1:
                     contactPoint = getContactPoint(root->second.children[1], 0);
                     break;
                case 2:
                     contactPoint = getContactPoint(root->second.children[0], 1);
                     break;
                case 3:
                     contactPoint = getContactPoint(root->second.children[1], 1);
                     break;
                case 4:
                     contactPoint = getContactPoint(root->second.children[2], 0);
                     break;
                case 5:
                     contactPoint = getContactPoint(root->second.children[2], 1);
                     break;
                default:
                     contactPoint = getContactPoint(root->second.children[0], 0);
                     break;


            }

            #ifdef DEBUG_PRINTS
            std::cout<<"[FORWARD_KINEMATICS] ContactPoint is :"<<contactPoint<<"\n";
            #endif

            /** Get the kinematics chain from the body to the contact point **/
            KDL::Chain chain;
            bool exit_value = tree.getChain(root->second.segment.getName(), contactPoint, chain);

            if (!exit_value)
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"[FORWARD_KINEMATICS] Chain no found\n";
                #endif
            }

            /** Create solver based on kinematic chain **/
            KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

            /** Create joint array **/
            unsigned int nj = chain.getNrOfJoints();
            KDL::JntArray jointpositions = KDL::JntArray(nj);
            #ifdef DEBUG_PRINTS
            std::cout<<"[FORWARD_KINEMATICS] Number of Joints is: "<<nj<<"\n";
            #endif

            /** Fill the joints positions (TO-DO: improve the coding stile with a fork) **/
            switch (chainIdx)
            {
                case 1:
                    jointpositions(0) = positions[1];//passive
                    jointpositions(1) = -positions[1];//mimic
                    jointpositions(2) = positions[4];//wheel walking
                    jointpositions(3) = positions[10];//steer
                    jointpositions(4) = positions[positions.size()-1];//contact angle
                    jointpositions(5) = this->wheelRadius*positions[14];//displacement due to wheel rotation

                    /** Slip vector **/
                    jointpositions(6) = positions[exoter_kinematics::EXOTER_JOINT_DOF];
                    jointpositions(7) = positions[exoter_kinematics::EXOTER_JOINT_DOF+1];
                    jointpositions(8) = positions[exoter_kinematics::EXOTER_JOINT_DOF+2];

                    break;
                case 2:
                    jointpositions(0) = positions[0];//passive
                    jointpositions(1) = -positions[0];//mimic
                    jointpositions(2) = positions[5];//wheel walking
                    jointpositions(3) = positions[positions.size()-1];//contact angle
                    jointpositions(4) = this->wheelRadius*positions[15];//displacement due to wheel rotation

                    /** Slip vector **/
                    jointpositions(5) = positions[exoter_kinematics::EXOTER_JOINT_DOF];
                    jointpositions(6) = positions[exoter_kinematics::EXOTER_JOINT_DOF+1];
                    jointpositions(7) = positions[exoter_kinematics::EXOTER_JOINT_DOF+2];

                    break;
                case 3:
                    jointpositions(0) = positions[1];//passive
                    jointpositions(1) = -positions[1];//mimic
                    jointpositions(2) = positions[6];//wheel walking
                    jointpositions(3) = positions[positions.size()-1];//contact angle
                    jointpositions(4) = this->wheelRadius*positions[16];//displacement due to wheel rotation

                    /** Slip vector **/
                    jointpositions(5) = positions[exoter_kinematics::EXOTER_JOINT_DOF];
                    jointpositions(6) = positions[exoter_kinematics::EXOTER_JOINT_DOF+1];
                    jointpositions(7) = positions[exoter_kinematics::EXOTER_JOINT_DOF+2];


                    break;
                case 4:
                    jointpositions(0) = positions[2];//passive
                    jointpositions(1) = -positions[2];//mimic
                    jointpositions(2) = positions[7];//wheel walking
                    jointpositions(3) = positions[11];//steer
                    jointpositions(4) = positions[positions.size()-1];//contact angle
                    jointpositions(5) = this->wheelRadius*positions[17];//displacement due to wheel rotation

                    /** Slip vector **/
                    jointpositions(6) = positions[exoter_kinematics::EXOTER_JOINT_DOF];
                    jointpositions(7) = positions[exoter_kinematics::EXOTER_JOINT_DOF+1];
                    jointpositions(8) = positions[exoter_kinematics::EXOTER_JOINT_DOF+2];

                    break;
                case 5:
                    jointpositions(0) = positions[2];//passive
                    jointpositions(1) = -positions[2];//mimic
                    jointpositions(2) = positions[8];//wheel walking
                    jointpositions(3) = positions[12];//steer
                    jointpositions(4) = positions[positions.size()-1];//contact angle
                    jointpositions(5) = this->wheelRadius*positions[18];//displacement due to wheel rotation

                    /** Slip vector **/
                    jointpositions(6) = positions[exoter_kinematics::EXOTER_JOINT_DOF];
                    jointpositions(7) = positions[exoter_kinematics::EXOTER_JOINT_DOF+1];
                    jointpositions(8) = positions[exoter_kinematics::EXOTER_JOINT_DOF+2];

                    break;
                default:
                    jointpositions(0) = positions[0];//passive
                    jointpositions(1) = -positions[0];//mimic
                    jointpositions(2) = positions[3];//wheel walking
                    jointpositions(3) = positions[9];//steer
                    jointpositions(4) = positions[positions.size()-1];//contact angle
                    jointpositions(5) = this->wheelRadius*positions[13];//displacement due to wheel rotation

                    /** Slip vector **/
                    jointpositions(6) = positions[exoter_kinematics::EXOTER_JOINT_DOF];
                    jointpositions(7) = positions[exoter_kinematics::EXOTER_JOINT_DOF+1];
                    jointpositions(8) = positions[exoter_kinematics::EXOTER_JOINT_DOF+2];

                    break;
            }


            /** Create the frame that will contain the results **/
            KDL::Frame cartpos;

            /** Calculate forward position kinematics **/
            bool kinematics_status;
            kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
            if(kinematics_status>=0)
            {
                /** Store the solution in the arguments **/
                double x,y,z,w;
                cartpos.M.GetQuaternion(x,y,z,w);
                Eigen::Quaternion<double> orientation(w,x,y,z);
                fkTrans = Eigen::Affine3d(orientation);
                fkTrans.translation() = Eigen::Vector3d (cartpos.p.x(), cartpos.p.y(), cartpos.p.z());

            }

            /** No uncertainty provided **/
            fkCov.setZero();
        }
    }

    return;
}

void ExoterKinematicKDL::fkSolver(const std::vector<double> &positions, std::vector<Eigen::Affine3d> &fkRobot, std::vector<base::Matrix6d> &fkCov)
{
    register unsigned int chainit = 0;
    std::vector<double> chainpositions;

    /** Check if the number of values is correct **/
    if (positions.size() == ExoterKinematicKDL::MODEL_DOF)
    {
        /** Resize the vectors **/
        fkRobot.resize(this->number_wheels);
        fkCov.resize(this->number_wheels);

        /** Resize the chain positions **/
        chainpositions.resize(this->getMaxChainDoF());
        std::fill(chainpositions.begin(), chainpositions.end(), 0.00);

        /** Fill the beginning (common part) **/
        for (register int i=0; i<exoter_kinematics::EXOTER_JOINT_DOF; ++i)
        {
            chainpositions[i] = positions[i];
        }

        /** Calculate the Forward Kinematics for all the chains of the robot **/
        for (register int i=0; i<static_cast<int>(this->number_wheels); ++i)
        {
            /** Create the joints for this chain (the rest) **/
            for (register int l=0; l<exoter_kinematics::SLIP_VECTOR_SIZE; ++l)
            {
                chainpositions[exoter_kinematics::EXOTER_JOINT_DOF+l] = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::SLIP_VECTOR_SIZE*i)+l];
            }

            /** Contact angle **/
            chainpositions[chainpositions.size()-1] = positions[ExoterKinematicKDL::MODEL_DOF-(number_wheels-i)];

            #ifdef DEBUG_PRINTS
            std::cout << "[ROBOT_FKSOLVER] chainpositions contains:";
            for (std::vector<double>::iterator it = chainpositions.begin() ; it != chainpositions.end(); ++it)
                std::cout << ' ' << *it;
            std::cout << '\n';
            #endif

            /** Perform the forward kinematics **/
            fkBody2ContactPointt(chainit, chainpositions, fkRobot[chainit], fkCov[chainit]);

            chainit++;
        }
    }
    return;

}

Eigen::Matrix<double, 6*NUMBER_OF_WHEELS, ExoterKinematicKDL::MODEL_DOF> ExoterKinematicKDL::jacobianSolver(const std::vector<double> &positions)
{
    Eigen::Matrix<double, 6*exoter_kinematics::NUMBER_OF_WHEELS, ExoterKinematicKDL::MODEL_DOF> J;

    /** Check if the number of values is correct **/
    if (positions.size() == ExoterKinematicKDL::MODEL_DOF)
    {
        /** Calculate the Jacobian Solver for all the chains of the robot **/
        for (register int i=0; i<static_cast<int>(this->number_wheels); ++i)
        {
            /** Get the chain **/
            std::stringstream chainidx;
            chainidx << i;

            KDL::SegmentMap::const_iterator root = this->tree.getRootSegment();
            std::string rootName = root->second.segment.getName();
            KDL::Chain chain, chainCitoCit, chainToContactPoint, chainFromContactPoint;
            bool exit_value1, exit_value2, exit_value3;
            exit_value1 = tree.getChain(rootName, chainidx.str()+"_CONTACT", chainToContactPoint);
            exit_value2 = tree.getChain(chainidx.str()+"_CONTACT", chainidx.str()+"_SLIPZ", chainCitoCit); //!CONTACT link is not taken (KDL works like that!!!)
            exit_value3 = tree.getChain(chainidx.str()+"_CONTACT", rootName, chainFromContactPoint);
            if (!exit_value1 || !exit_value2 || !exit_value3)
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"[JACOBIAN_SOLVER_KDL] ERROR Chain no found\n";
                #endif
            }
            else
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"[JACOBIAN_SOLVER_KDL] ChainIdx from "<<chainidx.str()+"_SLIPZ to "<< rootName <<"\n";
                #endif
            }

            /** Connect the kinematics chains and the root Segment **/
            chain.addSegment(root->second.segment); //! base_link
            chain.addChain(chainToContactPoint); //! From Body To contact plane
            chain.addChain(chainCitoCit); //! Movement and slip vector
            chain.addChain(chainFromContactPoint); //! From Contact To Body base

            #ifdef DEBUG_PRINTS
            std::cout << " ======================================" << std::endl;
            std::cout << " Chain has " << chain.getNrOfSegments() << " number of segments\n";
            for (register int l = 0; l<static_cast<int>(chain.getNrOfSegments()); ++l)
                std::cout <<"Segment["<<l<<"]: "<<chain.getSegment(l).getName() <<"\n";
            std::cout << " ======================================" << std::endl;
            #endif

            /** Create joint array **/
            unsigned int njTotal = chain.getNrOfJoints();
            KDL::JntArray jointpositions = KDL::JntArray(njTotal);

            /** Create KDL jacobian **/
            KDL::Jacobian kdljacob;
            kdljacob.resize(njTotal);

            /** Fill the joints positions (TO-DO: improve the coding stile with a fork) **/
            switch (i)
            {
                case 1:
                    jointpositions(0) = positions[1];//passive
                    jointpositions(1) = -positions[1];//mimic
                    jointpositions(2) = positions[4];//wheel walking
                    jointpositions(3) = positions[10];//steer
                    jointpositions(4) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)+1];
                    jointpositions(5) = 0.00;//displacement due to wheel rotation

                    /** Slip vector **/
                    for(register int j=0; j<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++j)
                        jointpositions(6+j) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(i*exoter_kinematics::SLIP_VECTOR_SIZE)+j];

                    break;
                case 2:
                    jointpositions(0) = positions[0];//passive
                    jointpositions(1) = -positions[0];//mimic
                    jointpositions(2) = positions[5];//wheel walking
                    jointpositions(3) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)+2];
                    jointpositions(4) = 0.00;//displacement due to wheel rotation

                    /** Slip vector **/
                    for(register int j=0; j<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++j)
                        jointpositions(5+j) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(i*exoter_kinematics::SLIP_VECTOR_SIZE)+j];

                    break;
                case 3:
                    jointpositions(0) = positions[1];//passive
                    jointpositions(1) = -positions[1];//mimic
                    jointpositions(2) = positions[6];//wheel walking
                    jointpositions(3) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)+3];
                    jointpositions(4) = 0.00;//displacement due to wheel rotation

                    /** Slip vector **/
                    for(register int j=0; j<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++j)
                        jointpositions(5+j) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(i*exoter_kinematics::SLIP_VECTOR_SIZE)+j];

                    break;
                case 4:
                    jointpositions(0) = positions[2];//passive
                    jointpositions(1) = -positions[2];//mimic
                    jointpositions(2) = positions[7];//wheel walking
                    jointpositions(3) = positions[11];//steer
                    jointpositions(4) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)+4];
                    jointpositions(5) = 0.00;//displacement due to wheel rotation

                    /** Slip vector **/
                    for(register int j=0; j<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++j)
                        jointpositions(6+j) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(i*exoter_kinematics::SLIP_VECTOR_SIZE)+j];

                    break;
                case 5:
                    jointpositions(0) = positions[2];//passive
                    jointpositions(1) = -positions[2];//mimic
                    jointpositions(2) = positions[8];//wheel walking
                    jointpositions(3) = positions[12];//steer
                    jointpositions(4) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)+5];
                    jointpositions(5) = 0.00;//displacement due to wheel rotation

                    /** Slip vector **/
                    for(register int j=0; j<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++j)
                        jointpositions(6+j) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(i*exoter_kinematics::SLIP_VECTOR_SIZE)+j];

                    break;

                default:
                    jointpositions(0) = positions[0];//passive
                    jointpositions(1) = -positions[0];//mimic
                    jointpositions(2) = positions[3];//wheel walking
                    jointpositions(3) = positions[9];//steer
                    jointpositions(4) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::NUMBER_OF_WHEELS*exoter_kinematics::SLIP_VECTOR_SIZE)+0];
                    jointpositions(5) = 0.00;//displacement due to wheel rotation

                    /** Slip vector **/
                    for(register int j=0; j<static_cast<int>(exoter_kinematics::SLIP_VECTOR_SIZE); ++j)
                        jointpositions(6+j) = positions[exoter_kinematics::EXOTER_JOINT_DOF+(i*exoter_kinematics::SLIP_VECTOR_SIZE)+j];

                    break;
            }

            /** The sub-chain from Ci to Body **/
            double auxSubChainNrJoint = chainToContactPoint.getNrOfJoints() + chainCitoCit.getNrOfJoints();
            for (register int j=0; j<static_cast<int>(chainFromContactPoint.getNrOfJoints()); ++j)
                jointpositions(auxSubChainNrJoint+j) = jointpositions(chainToContactPoint.getNrOfJoints()-1-j);

            #ifdef DEBUG_PRINTS
            std::cout<<"[JACOBIAN_SOLVER_KDL] njTotal is: "<<njTotal<<"\n";
            std::cout<<"[JACOBIAN_SOLVER_KDL] Joints to Ci: "<<chainToContactPoint.getNrOfJoints()<<"\n";
            std::cout<<"[JACOBIAN_SOLVER_KDL] Joints displacement: "<<chainCitoCit.getNrOfJoints()<<"\n";
            std::cout<<"[JACOBIAN_SOLVER_KDL] Joints from Ci: "<<chainFromContactPoint.getNrOfJoints()<<"\n";
            std::cout << "[JACOBIAN_SOLVER_KDL] jointpositions contains:";
            for (register unsigned int it = 0; it<njTotal; ++it)
                std::cout << ' ' << jointpositions(it);
            std::cout << '\n';
            #endif

            /** Solver for the Jacobian **/
            KDL::ChainJntToJacSolver jacsolver = KDL::ChainJntToJacSolver(chain);
            jacsolver.JntToJac(jointpositions, kdljacob);

            /** Get the Jacobian in Eigen Matrix class **/
            Eigen::Matrix<double, 6, Eigen::Dynamic> eigenjacob;
            eigenjacob.resize(6,njTotal);
            eigenjacob = kdljacob.data;
            #ifdef DEBUG_PRINTS
            std::cout<<"[JACOBIAN_SOLVER_KDL] Jacobian is "<<kdljacob.rows()<<" x " <<kdljacob.columns() <<std::endl;
            std::cout<<"[JACOBIAN_SOLVER_KDL] Jacobian:\n"<<eigenjacob<<std::endl;
            #endif

            /** Get the Jacobian in the desired form **/
            wheelJacobian[i].setZero();
            switch(i)
            {
                /** FR Wheel**/
             case 1:
                wheelJacobian[i].col(1) = eigenjacob.col(njTotal-2) - eigenjacob.col(njTotal-1);//passive parallel joints
                wheelJacobian[i].col(4) = eigenjacob.col(njTotal-3);//wheel walking
                wheelJacobian[i].col(10) = eigenjacob.col(njTotal-4);//steer joint
                wheelJacobian[i].col(14) = wheelRadius * eigenjacob.col(5);//wheel rotation
                break;

                /** ML Wheel**/
             case 2:
                wheelJacobian[i].col(0) = eigenjacob.col(njTotal-2) - eigenjacob.col(njTotal-1);//passive parallel joints
                wheelJacobian[i].col(5) = eigenjacob.col(njTotal-3);//wheel walking
                wheelJacobian[i].col(15) = wheelRadius * eigenjacob.col(4);//wheel rotation
                break;

                /** MR Wheel**/
             case 3:
                wheelJacobian[i].col(1) = eigenjacob.col(njTotal-2) - eigenjacob.col(njTotal-1);//passive parallel joints
                wheelJacobian[i].col(6) = eigenjacob.col(njTotal-3);//wheel walking
                wheelJacobian[i].col(16) = wheelRadius * eigenjacob.col(4);//wheel rotation
                break;

                /** RL Wheel**/
             case 4:
                wheelJacobian[i].col(2) = eigenjacob.col(njTotal-2) - eigenjacob.col(njTotal-1);//passive parallel joints
                wheelJacobian[i].col(7) = eigenjacob.col(njTotal-3);//wheel walking
                wheelJacobian[i].col(11) = eigenjacob.col(njTotal-4);//steer joint
                wheelJacobian[i].col(17) = wheelRadius * eigenjacob.col(5);//wheel rotation
                break;

                /** RR Wheel**/
             case 5:
                wheelJacobian[i].col(2) = eigenjacob.col(njTotal-2) - eigenjacob.col(njTotal-1);//passive parallel joints
                wheelJacobian[i].col(8) = eigenjacob.col(njTotal-3);//wheel walking
                wheelJacobian[i].col(12) = eigenjacob.col(njTotal-4);//steer joint
                wheelJacobian[i].col(18) = wheelRadius * eigenjacob.col(5);//wheel rotation
                break;

                /** FL Wheel**/
             default:
                wheelJacobian[i].col(0) = eigenjacob.col(njTotal-2) - eigenjacob.col(njTotal-1);//passive parallel joints
                wheelJacobian[i].col(3) = eigenjacob.col(njTotal-3);//wheel walking
                wheelJacobian[i].col(9) = eigenjacob.col(njTotal-4);//steer joint
                wheelJacobian[i].col(13) = wheelRadius * eigenjacob.col(5);//wheel rotation
                break;
            }

            switch(i)
            {
             case 0: case 1: case 4: case 5:
                /** Slip Vector and Contact angles **/
                wheelJacobian[i].block<6, exoter_kinematics::SLIP_VECTOR_SIZE+exoter_kinematics::CONTACT_POINT_DOF>(0, exoter_kinematics::EXOTER_JOINT_DOF) =
                    eigenjacob.block<6, exoter_kinematics::SLIP_VECTOR_SIZE+exoter_kinematics::CONTACT_POINT_DOF>(0, 6);
                break;
             case 2: case 3:
                /** Slip Vector and Contact angles **/
                wheelJacobian[i].block<6, exoter_kinematics::SLIP_VECTOR_SIZE+exoter_kinematics::CONTACT_POINT_DOF>(0, exoter_kinematics::EXOTER_JOINT_DOF) =
                    eigenjacob.block<6, exoter_kinematics::SLIP_VECTOR_SIZE+exoter_kinematics::CONTACT_POINT_DOF>(0, 5);
                break;
             default:
                break;
            }


            #ifdef DEBUG_PRINTS
            std::cout<<"[JACOBIAN_SOLVER_KDL] wheelJacobian is "<<wheelJacobian[i].rows()<<" x "<<wheelJacobian[i].cols()<<std::endl;
            std::cout<<"[JACOBIAN_SOLVER_KDL] wheelJacobian:\n"<<wheelJacobian[i]<<std::endl;
            #endif

        }

        J.setZero();

        /** Form the robot Jacobian matrix (sparse Jacobian matrix) **/
        for (register int i=0; i<static_cast<int>(this->number_wheels); ++i)
        {
            /** Joints including wheel rotation **/
            J.block<6, exoter_kinematics::EXOTER_JOINT_DOF>(6*i, 0) = wheelJacobian[i].block<6, exoter_kinematics::EXOTER_JOINT_DOF>(0,0);
            /** Slip vectors **/
            J.block<6, exoter_kinematics::SLIP_VECTOR_SIZE>(6*i, exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::SLIP_VECTOR_SIZE*i)) = wheelJacobian[i].block<6, exoter_kinematics::SLIP_VECTOR_SIZE>(0,exoter_kinematics::EXOTER_JOINT_DOF);
            /** Contact Angles **/
            J.block<6, exoter_kinematics::CONTACT_POINT_DOF>(6*i, exoter_kinematics::EXOTER_JOINT_DOF+(exoter_kinematics::SLIP_VECTOR_SIZE*exoter_kinematics::NUMBER_OF_WHEELS)+(exoter_kinematics::CONTACT_POINT_DOF*i)) = wheelJacobian[i].block<6, exoter_kinematics::CONTACT_POINT_DOF>(0,exoter_kinematics::EXOTER_JOINT_DOF+exoter_kinematics::SLIP_VECTOR_SIZE);
        }

    }
    return J;
}
