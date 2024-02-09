#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ss_exponential_filter/sloshing_trajectory.h>
#include "ss_exponential_filter/SS_filter.h"
#include "ss_exponential_filter/ComauSSGenerator.h"
#include "comau_control/IKServiceTotal.h"
#include "comau_control/ComauIK.h"

std::vector<double> current_pose,current_EF;
comau_control::ComauIK *robot;
trajectory_msgs::JointTrajectory traj;

void targetDefiner(std::vector<bool> &flags,Eigen::Vector3d &target_pose);								//function to define next position
void writeJointToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj);
void writeSSToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj);
void writeTrajToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj);

//save robot_current pose
void jointStatesCallback(const sensor_msgs::JointState &state){
	for (int i=0;i<6;i++){
		current_pose[i]=state.position[i];
	}
	current_EF=robot->fk(current_pose);
}

int main(int argc, char **argv)
{

    //ros structure initialisation
	ros::init(argc, argv, "move_comau_to_target");
    ros::NodeHandle n;



    /**
     *  Subscribers 
     */
    ros::Subscriber joint_sub = n.subscribe("/comau_smart_six/joint_states", 1, jointStatesCallback);

    /**
     *  Publishers
     */
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command", 1);
	ros::ServiceClient service = n.serviceClient<comau_control::IKServiceTotal>("comau_ik_service/ik_service_traj_neg");
    ros::ServiceClient service_slosh  = n.serviceClient<ss_exponential_filter::sloshing_trajectory>("sloshing_suppression_service");
    //variable definition
    std::vector<bool> flags;
	bool publish=false;
	int count=0;
	comau_control::IKServiceTotal srv;
	sensor_msgs::JointState comau_ref;
	std::ofstream joint_file,ss_file,traj_file;
	Eigen::Vector3d pose;
	std::vector<Eigen::Vector3d> reference;
	Eigen::Vector3d target_pose;
	ss_exponential_filter::ComauSSGenerator generator(2,3,0.002);
	ss_exponential_filter::sloshing_trajectory srv_slosh;
	ss_exponential_filter::SS_filter ssfilter(1,0.002);

	//variable initialisation
	flags.resize(5);
	//robot = new comau_control::ComauIK(127.5,0,209);
	robot = new comau_control::ComauIK(0,0,0);
	for (int i=0;i<6;i++){
		flags[i]=false;
	}
	current_pose.resize(5);
	current_EF.resize(3);
	current_EF[0]=797;
	current_EF[1]=0;
	current_EF[2]=1170;
	std::fill(current_pose.begin(), current_pose.end(), 0.0);
	comau_ref.position.resize(6);

	//start thread for recording data
	std::thread target(targetDefiner,std::ref(flags),std::ref(target_pose));
	std::cout<<"ciao"<<std::endl;
	ros::Rate r(500);
	while(ros::ok())
	{
		if(flags[3]){
			reference.clear();
			for (int i=0;i<3;i++){
				pose[i]=current_EF[i]/1000;
				std::cout<<current_EF[i]<<std::endl;
			}
			reference.push_back(pose);
			reference.push_back(target_pose);
			generator.startNewSplineTrajectory();
			generator.setNewViaPoints(reference);
			srv_slosh.request.traj=ssfilter.EigenToJointTrajectoryPos(generator.generateSimplifiedBSpline());
			traj_file.open("/home/davide/ros/sloshing_ws/files/traj_check.csv");
			writeTrajToFile(traj_file,srv_slosh.request.traj);
			traj_file.close();
			srv_slosh.request.set_initial_pos.data=false;
			if (service_slosh.call(srv_slosh)){
				traj=srv_slosh.response.filtered_traj;
			}
			ss_file.open("/home/davide/ros/sloshing_ws/files/ss_check.csv");
			writeSSToFile(ss_file,traj);
			ss_file.close();
			srv.request.target_traj=traj;
			srv.request.wrist_compensation.data=false;
			if (service.call(srv)){
				traj=srv.response.q_out;
				joint_file.open("/home/davide/ros/sloshing_ws/files/check.csv",std::ofstream::out);
        		writeJointToFile(joint_file,traj);
				joint_file.close();
			}
			else std::cout<<"epic fail"<<std::endl;
			count=0;
			flags[3]=false;
		}
		if(flags[4]){
			for (int i=0;i<6;i++){
				comau_ref.position[i]=traj.points[count].positions[i];
			}
			//comau_ref.position[5]=M_PI;
			joint_pub.publish(comau_ref);
			count++;
			if (count==traj.points.size()){
				flags[4]=false;
				flags[2]=true;
				count =0;
			}
		}

		ros::spinOnce();
		r.sleep();		
	}

	return 0;
}
	
	
void targetDefiner(std::vector<bool> &flags ,Eigen::Vector3d &target_pose)
{
	char again;
	flags[0]=true;
	flags[1]=true;
	while(flags[0]){
		while(flags[1])
		{
			std::cout<<"define the new target x coordinate for comau"<<std::endl;
			std::cin>>target_pose[0];
			std::cout<<"define the new target y coordinate for comau"<<std::endl;
			std::cin>>target_pose[1];
			std::cout<<"define the new target z coordinate for comau"<<std::endl;
			std::cin>>target_pose[2];	
			std::cout<<"the target pose is set at: "<<target_pose[0]<<", "<<target_pose[1]<<", "<<target_pose[2]<<std::endl;
			std::cout<<"do you confirm? (type 'y' to confirm)"<<std::endl;
			std::cin>>again;
			if (again=='y') flags[1]=false;
		}
		flags[1]=true;
		flags[2]=false;
		std::cout<<"preparing and providing the new trajectory to the robot"<<std::endl;
		flags[3]=true;
		while (flags[3]);
		std::cout<<"trajectory evaluate check on matlab in check.csv if you want, ('y' to send) "<<std::endl;
		std::cin>>again;
		if (again=='y') flags[4]=true; else flags[2]=true;
				while(!flags[2]);

	}
}

void writeJointToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj){
    for (int j=0;j<traj.points.size();j++){
        for (int i = 0; i < 6; i++)
        {
            traj_file << traj.points[j].positions[i] << ";" ;
        }
        traj_file << std::endl;
    }

}

void writeSSToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj){
    for (int j=0;j<traj.points.size();j++){
        for (int i = 0; i < 3; i++)
        {
            traj_file << traj.points[j].positions[i] << ";" ;
        }
        for (int i = 0; i < 3; i++)
        {
            traj_file << traj.points[j].accelerations[i] << ";" ;
        }
        traj_file << std::endl;
    }

}

void writeTrajToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj){
    for (int j=0;j<traj.points.size();j++){
        for (int i = 0; i < 3; i++)
        {
            traj_file << traj.points[j].positions[i] << ";" ;
        }
        traj_file << std::endl;
    }

}