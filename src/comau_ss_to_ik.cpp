#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <comau_control/IKServiceTotal.h>

comau_control::IKServiceTotal srv;
ros::ServiceClient service;


void writeAllToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj){
    for (int j=0;j<traj.points.size();j++){
        for (int i = 0; i < 6; i++)
        {
            traj_file << traj.points[j].positions[i] << ";" ;
        }
        traj_file << std::endl;
    }

}

void trajStatesCallback(const trajectory_msgs::JointTrajectory &traj){
    srv.request.target_traj=traj;
    srv.request.wrist_compensation.data=true;
    std::ofstream traj_file;
    if (service.call(srv)){
        traj_file.open("/home/davide/ros/sloshing_ws/files/comau_ref/test1.csv",std::ofstream::out);
        writeAllToFile(traj_file,srv.response.q_out);
    }
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_ss_to_ik");
    ros::NodeHandle n;

     //  Subscribers 
    ros::Subscriber joint_sub = n.subscribe("/ss_filtered_traj", 1, trajStatesCallback);
    service = n.serviceClient<comau_control::IKServiceTotal>("comau_ik_service/ik_service_traj");


    ros::spin();
	return 0;
}
	
	
