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


sensor_msgs::JointState command,state;


void writeAllToFile(std::ofstream &state_file,sensor_msgs::JointState &joint_command,sensor_msgs::JointState &joint_states){
    for (int i = 0; i < 6; i++)
    {
        state_file << joint_command.position[i] << ";" ;
    }
    for (int i = 0; i < 6; i++)
    {
        state_file << joint_states.position[i] << ";" ;
    }
    state_file << std::endl;


}

void jointStatesCallback(const sensor_msgs::JointState &joint_states){
    state=joint_states;
}

void jointCommandCallback(const sensor_msgs::JointState &joint_command){
    command=joint_command;
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_trajectory_recorder");
    ros::NodeHandle n;
    std::stringstream file_name;
    

    //Variable initialization
    file_name.str("");
    file_name<<"/home/davide/ros/sloshing_ws/files/comau_readings/";
    command.position.resize(6);
    std::fill(command.position.begin(),command.position.end(),0.0);
    state.position.resize(6);
    std::fill(state.position.begin(),state.position.end(),0.0);

    if (argc>1) {
        file_name<<argv[1];
    }

    std::ofstream comau_file;
    comau_file.open(file_name.str().c_str(),std::ofstream::out);

     //  Subscribers 
    ros::Subscriber joint_sub = n.subscribe("/comau_smart_six/joint_states", 1, jointStatesCallback);
    ros::Subscriber joint_pub = n.subscribe("/comau_smart_six/joint_command", 1, jointCommandCallback);

    ros::Rate r(500);
    while (ros::ok())
    {
        writeAllToFile(comau_file,command,state);
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}
	
	
