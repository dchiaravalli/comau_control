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


void jointStatesCallback(const sensor_msgs::JointState &state){

}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_trajectory_follower");
    ros::NodeHandle n;
    std::string name;
    std::stringstream file;
    ros::Time tmsg;
    sensor_msgs::JointState reference;
    std::vector<std::vector<double>> traj;
    std::vector<double> temp;
    reference.position.resize(6);
    temp.resize(6);
    int count=1;
    char test;
    file.str(""); 
    file<<"/home/davide/ros/sloshing_ws/files/comau_ref/";
    if (argc>1){
        name=argv[1];
    }
    file<<name;

    std::ifstream save_file(file.str().c_str());
    std::cout<<save_file.is_open()<<std::endl;
    save_file>>temp[0];
    while(save_file.get()!=EOF){
        save_file>>temp[count];
        count = (count+1) %6;
        if (count==0){
            traj.push_back(temp);
        }
    }

std::cout<<"file read"<<std::endl;
     //  Subscribers 
    ros::Subscriber joint_sub = n.subscribe("/comau_smart_six/joint_states", 1, jointStatesCallback);

count=0;
 // Publishers
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command", 1);
    count=0;
	ros::Rate r(500);
	while(ros::ok())
	{   
        tmsg=ros::Time::now();
        reference.header.stamp.sec=tmsg.sec;
        reference.header.stamp.nsec=tmsg.nsec;
        for (int i=0;i<6;i++){
            reference.position[i]=traj[count][i];
        }
        joint_pub.publish(reference);
        count++;
        if (count==traj.size()){
            ros::shutdown();
        }
		ros::spinOnce();
		r.sleep();		
	}

save_file.close();
	return 0;
}
	
	
