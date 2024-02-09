/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ComauTrajectoryServer.cpp
 * Author: Davide Chiaravalli
 * 
 * Created on Feb 28, 2023, 4:05 PM
 */



#include "comau_control/ComauTrajectoryServer.h"

namespace comau_control  {

    ComauTrajectoryServer::ComauTrajectoryServer(ros::NodeHandle n):as_(n, "/comau_smart_six/execute_trajectory", boost::bind(&ComauTrajectoryServer::executeCb, this, _1), false),
    action_name_("/comau_smart_six/execute_trajectory"),n(n)
    {
        this->robot_ik = new ComauIK();
        this->go = false;
        this->busy = false;
        this->initialiseRos();
        this->robot_joint_state.resize(6);
        this->pose_offset.resize(3);
        std::fill(pose_offset.begin(),pose_offset.end(),0.0);
        as_.start();
    }

    ComauTrajectoryServer::~ComauTrajectoryServer()
    {
        //nothing to destroy
    }

    void ComauTrajectoryServer::initialiseRos()
    {
        this->robot_sub = this->n.subscribe("/comau_smart_six/joint_states",1,&ComauTrajectoryServer::robotCb,this);
        this->robot_pub = this->n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command", 1, true);
        this->object_pose_service = this->n.advertiseService("/comau_smart_six/set_object_pose",&ComauTrajectoryServer::objectPoseCb,this);
    }

    void ComauTrajectoryServer::robotCb(const sensor_msgs::JointState &state)
    {
        for (int i=0;i<6;i++){
            this->robot_joint_state[i]=state.position[i];
        }
        this->robot_ef_state = this->robot_ik->fk_eigen(this->robot_joint_state);
        this->go = true;
    }

    bool ComauTrajectoryServer::objectPoseCb(comau_control::IKPoseReference::Request &req,comau_control::IKPoseReference::Response &res)
    {
        std::cout<<"set tool offset"<<std::endl;
        this->robot_ik->setToolOffsets(Eigen::Vector3d(req.object_pose.position.x,req.object_pose.position.y,req.object_pose.position.z));
        this->pose_offset[0] = req.object_pose.position.x;
        this->pose_offset[1] = req.object_pose.position.y;
        this->pose_offset[2] = req.object_pose.position.z;
        res.success = true;
        return true;
    }

    bool ComauTrajectoryServer::ikCheck(std::vector<double> trajectory_reference){
        if (trajectory_check.empty())
        {
            for (int i=0;i<6;i++)
            {
                trajectory_check.push_back(trajectory_reference[i]);
            }
            return true;
        }
        else
        {
            for(int i=0;i<6;i++)
            {
                if(fabs(trajectory_check[i]-trajectory_reference[i])>0.2){
                    std::cout<<"i "<<i<<std::endl;
                    std::cout<<"check "<<trajectory_check[i]<<std::endl;
                    std::cout<<"ref "<<trajectory_reference[i]<<std::endl;
                    return false;
                }
                trajectory_check[i]=trajectory_reference[i];
            }
            return true;
        }
    }

    void ComauTrajectoryServer::executeCb(const comau_traj::ComauTrajectoryGoalConstPtr &goal)
    {
        if ((!this->busy)&&(this->go))
        {
            std::cout<<"check goal"<<std::endl;
            if ((goal->reference_traj.points[0].positions[0]!=0)||(goal->reference_traj.points[0].positions[1]!=0)||(goal->reference_traj.points[0].positions[2]!=0))
            {
                comau_traj::ComauTrajectoryResult result_;
                ROS_INFO("%s: Aborted (non zero starting point)", action_name_.c_str());
                std::cout<<"pose "<<goal->reference_traj.points[0].positions[0]<<" "<<goal->reference_traj.points[0].positions[1]<<" "<<goal->reference_traj.points[0].positions[2]<<" "<<std::endl;
                this->as_.setAborted(result_);
                return;
            }
            //activate task lock
            std::cout<<"activate task lock"<<std::endl;
            this->busy = true;

            //initialise class variables
            int points = goal->reference_traj.points.size();
            KDL::Frame ik_frame, acc_frame;
            std::vector<double> joint_sol;
            joint_sol.resize(6);
            comau_traj::ComauTrajectoryFeedback feedback_;

            //define control frequency
            ros::Rate r(500);

            Eigen::Vector3d robot_start_state = this->robot_ef_state;
            bool compensation = false;
            if (goal->compensation)
                compensation = true;

            ROS_INFO("Performing trajectory");
            for (int i=0;i<points;i++)
            {
                //std::cout<<i<<std::endl;
                //check for preemption
                if (as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    // set the action state to preempted
                    as_.setPreempted();
                    break;
                }

                //call to inverse kinematic 
                ik_frame.p = KDL::Vector(1000*(robot_start_state[0]+goal->reference_traj.points[i].positions[0]),1000*(robot_start_state[1]+goal->reference_traj.points[i].positions[1]),1000*(robot_start_state[2]+goal->reference_traj.points[i].positions[2]));
                ik_frame.M = KDL::Rotation::RPY(0,-0.02,0);
                Eigen::Vector3d support(goal->reference_traj.points[i].accelerations[0],goal->reference_traj.points[i].accelerations[1],goal->reference_traj.points[i].accelerations[2]);
                this->robot_ik->ik_neg5(ik_frame,support,compensation,joint_sol);

                //check for solution consistency w.r.t. actual position
                if(!this->ikCheck(joint_sol)){
                    std::cout<<"big angles difference, stopping"<<std::endl;
                    return;
                }

                //reference publish
                sensor_msgs::JointState reference_joints;
                reference_joints.position.resize(6);
                for(int j=0;j<6;j++)
                {
                    reference_joints.position[j] = joint_sol[j];
                }
                // reference_joints.position[0] = 0.000353339665948738;
                // reference_joints.position[1] = 0.10736046341190163;
                // reference_joints.position[2] = -1.641126558099759;
                // reference_joints.position[3] = joint_sol[3];
                // reference_joints.position[4] = 1.3931053364028114;
                // reference_joints.position[5] = 0.0002212587523151457;

                ros::Time tmsg=ros::Time::now();
                reference_joints.header.stamp.sec=tmsg.sec;
                reference_joints.header.stamp.nsec=tmsg.nsec;
                this->robot_pub.publish(reference_joints);

                //set feedback
                feedback_.task_percentage = i/(float)points+1/(float(points));
                as_.publishFeedback(feedback_);

                //impose control frequency 
                r.sleep();
            }
            this->as_.setSucceeded();
            this->busy = false;
        }
        else
        {
            if (this->busy)
                ROS_INFO("Robot busy, abort previous trajectory first");
            else if(!this->go)
                ROS_INFO("Joint state from robot not received, can't activate motion");
            this->as_.setAborted();
        }

    }

    void ComauTrajectoryServer::run()
    {
        ros::spin();
    }

}