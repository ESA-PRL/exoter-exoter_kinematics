set(EXOTER_HDRS
    Configuration.hpp
    ExoterKinematicKDL.hpp
    ExoterKinematicModel.hpp
    ExoterLocomotionControl.hpp
    ExoterWheelwalkingControl.hpp
    ExoterEgressControl.hpp
    ExoterLocomotionKinematics.hpp
    ExoterWheelwalkingKinematics.hpp
    ExoterEgressKinematics.hpp
    ExoterWheelwalkingTypes.hpp
    ExoterEgressTypes.hpp
)

set(EXOTER_SRCS
    ExoterKinematicKDL.cpp
    ExoterKinematicModel.cpp
    ExoterLocomotionControl.cpp
    ExoterWheelwalkingControl.cpp
    ExoterEgressControl.cpp
    ExoterLocomotionKinematics.cpp
    ExoterWheelwalkingKinematics.cpp
    ExoterEgressKinematics.cpp
)

rock_library(exoter_kinematics
    ${EXOTER_SRCS}
    DEPS_PKGCONFIG base-types base-lib odometry kdl_parser
    DEPS_CMAKE Boost
    HEADERS ${EXOTER_HDRS})

