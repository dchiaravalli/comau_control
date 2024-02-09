/* 
 * File:   ComauTrajectoryServer.h
 * Author: Davide Chiaravalli,
 *
 * Created on Feb 28, 2023, 4:05 PM
 */

#ifndef COMAUTRAJ
#define COMAUTRAJ

//ros routines
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <comau_control/ComauIK.h>

//ros messages
#include <sensor_msgs/JointState.h>
#include <comau_traj/ComauTrajectoryAction.h>
#include <comau_control/IKPoseReference.h>

//c++ routines
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <kdl/frames_io.hpp>

namespace comau_control {

    class ComauTrajectoryServer
    {
        public:
            //Class constructor
            ComauTrajectoryServer(ros::NodeHandle n);
            virtual ~ComauTrajectoryServer();

            //Class methods
            void run();

            //Class getter/setter
            //None active (do your own business :))

        private:
            //Class variables
            ros::NodeHandle n;                                                                      //ros nodehandle
            actionlib::SimpleActionServer<comau_traj::ComauTrajectoryAction> as_;                //action server for trajectory execution
            ros::Subscriber robot_sub;
            ros::Publisher robot_pub;
            ros::ServiceServer object_pose_service;

            bool go;
            bool busy;
            std::vector<double> robot_joint_state;
            std::vector<double> real_pose;
            Eigen::Vector3d robot_ef_state;
            comau_control::ComauIK *robot_ik;
            std::vector<double> trajectory_check;
            std::string action_name_;
            std::vector<double> pose_offset;

            //Class private methods
            void initialiseRos();
            void executeCb(const comau_traj::ComauTrajectoryGoalConstPtr &goal);
            void robotCb(const sensor_msgs::JointState &state);
            bool objectPoseCb(comau_control::IKPoseReference::Request &req,comau_control::IKPoseReference::Response &res);
            bool ikCheck(std::vector<double> trajectory_reference);

    };

}
#endif 