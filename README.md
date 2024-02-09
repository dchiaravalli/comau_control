C++ Ros repository for ComauSmartSix libC4GOpen based controller connection

**Installation**
Package dependecies required:
- sudo apt update && sudo apt install ros-noetic-kdl-conversions ros-noetic-kdl-parser 

Additional repository for matlab communication:
- git clone https://github.com/dchiaravalli/comau_traj.git


**Run**
The package is running several files exploiting two main classes:
- *ComauIK*: the class implementing the kinematic of the robot:
    ComauIK::ik -> solves the inverse kinematic for the wrist up configuration of the robot (input in millimiters)
    ComauIK::ik_neg5 -> solves the inverse kinematic for the wrist down configuration of the robot (input in millimiters)

    ComauIK::total_fk -> returns a direct kinematic solution as KDL::Frame in millimiters
    ComauIk::fk_eigen -> returns a direcf kinematic solution as Eigen::Vector3d in meters

    ComauIK::setToolOffsets-> set the new reference frame (position only) for the kinematic (millimiters with respect to the standard frame (center of joint 5))

- *ComauTrajectoryServer*: the class for execution of a given trajectory (usable also from matlab):

The control of the robot is mainly handled by three processes: 
- *comau_workspace_reconfigure*: Allows fast motion of the robot to cartesian target using rqt_dynamic_reconfigure (built-in inverse kinematics)

- *comau_trajectory_matlab*: Provides action server for trajectory execution (built-in inverse kinematics)

- *comau_ik_service*: activate direct/inverse kinematics service routines