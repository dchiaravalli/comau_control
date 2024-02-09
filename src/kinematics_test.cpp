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
#include "comau_control/IKService.h"



void jointStatesCallback(const sensor_msgs::JointState &state){

}

void setPose(std::vector<double> &reference,bool &go,bool &send)
{
    char again;
    bool yes=true;
    while(true){
        while(yes){
            std::cout<<"do you want to move comau? (type 'y' to proceed)"<<std::endl;
            std::cin>>again;
            if(again=='y'){
                yes=false;
            }
        }
        yes=true;
        while(yes){
            std::cout<<"set x coordinate"<<std::endl;
            std::cin >>reference[0];
            std::cout<<"set y coordinate"<<std::endl;
            std::cin >>reference[1];
            std::cout<<"set z coordinate"<<std::endl;
            std::cin >>reference[2];
            std::cout<<"you have set: "<<reference[0]<<" "<<reference[1]<<" "<<reference[2]<<std::endl;
            std::cout<<"do you confirm? (type 'y' to proceed)"<<std::endl;
            std::cin>>again;
            if(again=='y'){
                yes=false;
                go=true;
            }
        }
        std::cout<<"do you like the filtered angles? 'y'"<<std::endl;
        std::cin>>again;
        if(again=='y'){
            send=true;
        }
        yes=true;
    }

}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_trajectory_follower");
    ros::NodeHandle n;
    std::string name;
    std::stringstream file;
    comau_control::IKService srv;
    sensor_msgs::JointState reference;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(3);
    point.accelerations.resize(3);
    std::vector<double> comau_ef;
    comau_ef.resize(3);
    reference.position.resize(6);
    bool go=false;
    bool send=false;
     //  Subscribers 
    ros::Subscriber joint_sub = n.subscribe("/comau_smart_six/joint_states", 1, jointStatesCallback);
    ros::ServiceClient service = n.serviceClient<comau_control::IKService>("comau_ik_service/ik_service");
    //std::thread set(setPose, std::ref(comau_ef),std::ref(go),std::ref(send));
    srv.request.wrist_compensation.data=true;
    srv.request.restart.data=true;
 // Publishers
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command", 1);
    reference.position[0]=0;
    reference.position[1]=0;
    reference.position[2]=-1.82;
    reference.position[3]=0;
    reference.position[4]=1.32;
    reference.position[5]=0;

    // reference.position[0]=0;
    // reference.position[1]=1;
    // reference.position[2]=-1;
    // reference.position[3]=0;
    // reference.position[4]=1.2;
    // reference.position[5]=0;
    reference.header.frame_id="filtered";


	ros::Rate r(500);
   while(ros::ok()){
       //reference.position[3]=reference.position[3]+0.00002;
       //if (reference.position[3]>=0.5)reference.position[3]=0.5;
    joint_pub.publish(reference);
    		ros::spinOnce();
		r.sleep();		
	}
	/*while(ros::ok())
	{   

            
        //reference.header.stamp.sec=ros::Time::now().toSec();
        //reference.header.stamp.nsec=ros::Time::now().toNSec();
        if (go){
            for (int i=0;i<3;i++)
            {
                point.positions[i]=comau_ef[i];
            }
            point.accelerations[0]=0;
            point.accelerations[1]=0;
            point.accelerations[2]=0;
            srv.request.target_pose=point;
            std::cout<<srv.request.target_pose.accelerations[2]<<std::endl;
            if (service.call(srv)){
                for (int i=0;i<6;i++){
                    reference.position[i]=srv.response.q_out.data[i];
                    std::cout<<reference.position[i]<<std::endl;
                }
            }
            go=false;
        }
        if (send){
            std::cout<<"sono qui"<<std::endl;
            joint_pub.publish(reference);
            send=false;
        }
		ros::spinOnce();
		r.sleep();		
	}*/
	return 0;
}
	
	
