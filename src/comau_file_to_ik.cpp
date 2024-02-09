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
#include <trajectory_msgs/JointTrajectory.h>

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


int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "move_comau_to_target");
    ros::NodeHandle n;
    trajectory_msgs::JointTrajectoryPoint point;
    trajectory_msgs::JointTrajectory traj;
    point.positions.resize(3);
    point.accelerations.resize(3);
    std::string name;
    bool pos=true;
    std::stringstream file;
    int count=1;
    file.str(""); 
    file<<"/home/davide/ros/sloshing_ws/files/";
    if (argc>1){
        name=argv[1];
    }
    file<<name;

    std::ifstream save_file(file.str().c_str());
    save_file>>point.positions[0];
    while(save_file.get()!=EOF){
        if (pos){
            save_file>>point.positions[count];
            count = count %3;
            if (count==0){
                pos=false;
                continue;
            }
        }
        else{
            save_file>>point.positions[count];
            count = count %3;
            if (count==0){
                traj.points.push_back(point);
                pos=true;;
            }
        }


    }
    save_file.close();
    srv.request.target_traj=traj;
    srv.request.wrist_compensation.data=true;
    if (service.call(srv)){

    }


     //  Subscribers 

    service = n.serviceClient<comau_control::IKServiceTotal>("comau_ik_service/ik_service_traj");


    ros::spin();
	return 0;
}
	
	
