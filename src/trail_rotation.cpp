#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <signal.h>
#include <Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <comau_control/comau_poseConfig.h>
#include <dynamic_reconfigure/server.h>
#include "comau_control/ComauIK.h"
#include <ss_exponential_filter/LocateObjectAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>

bool configure,out_of_bounds,go;
std::vector<double> joint_pose,reference_pose;
KDL::Frame ef_frame;

void jointStatesCallback(const sensor_msgs::JointState &msg){
    
    for(int i =0;i<6;i++){
        joint_pose[i] = msg.position[i];
    }
    configure = true;
}

void emptyCb(const std_msgs::Float64MultiArray &msg)
{
    go =true;
}


enum State{
    START,
    INIZIALIZATION,
    WAITING,
    WORKING
};


int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_trajectory_follower");
    ros::NodeHandle n;

    sensor_msgs::JointState reference;
    ss_exponential_filter::LocateObjectGoal goal;
    std_msgs::Float64MultiArray result;

    //variable initialization
    configure = false;
    joint_pose.resize(6);
    std::fill(joint_pose.begin(),joint_pose.end(),0.0);
    reference_pose.resize(6);
    std::fill(reference_pose.begin(),reference_pose.end(),0.0);
    reference.position.resize(6);
    out_of_bounds = false;
    float object_heigth = 0.01;
    go=false;
    

    //  Subscribers 
    ros::Subscriber joint_sub = n.subscribe("/comau_smart_six/joint_states", 1, jointStatesCallback);
    ros::Subscriber go_sub = n.subscribe("/comau_smart_six/go",1,emptyCb);

    // Publishers
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command", 1);

    //action servers
    actionlib::SimpleActionClient<ss_exponential_filter::LocateObjectAction> ac("locate_rigid_objects", true);
    std::cout<<"look for server"<<std::endl;
    //ac.waitForServer();
    std::cout<<"done waiting for server"<<std::endl;

    //state machine definition and initialization
    State state = START;
    Eigen::Vector3d temp(0,0,0);
    bool success = true;

    //FK/IK handler
    

    //unit rotation definition
    float unit = 1.0/180.0*M_PI/500.0;              //delta of rotation per cycle
    float rotation = 0;                             //current value of rotation around x
    float delta = 0;


    comau_control::ComauIK ik_handler(0,0,0);

	ros::Rate r(500);

	while(ros::ok())
	{   
        out_of_bounds = false;

        switch(state){

            case START:             
            {
                if (configure) 
                {
                    state = INIZIALIZATION;
                    std::cout<<"go to initialization"<<std::endl;
                }
                break;
            }

            case INIZIALIZATION:    
            {
                ef_frame = ik_handler.total_fk(joint_pose);
                double roll,pitch,yaw;
                ros::param::set("end_effector_x", ef_frame.p.x()/1000);
                ros::param::set("end_effector_y", ef_frame.p.y()/1000);
                ros::param::set("end_effector_z", ef_frame.p.z()/1000);
                ef_frame.M.GetRPY(roll,pitch,yaw);
                ros::param::set("end_effector_roll", roll);
                ros::param::set("end_effector_pitch", pitch);
                ros::param::set("end_effector_yaw", yaw);
                std::cout<<ef_frame.p.x()<<std::endl;
                std::cout<<ef_frame.p.y()<<std::endl;
                std::cout<<ef_frame.p.z()<<std::endl;
                std::cout<<roll<<std::endl;
                std::cout<<pitch<<std::endl;
                std::cout<<yaw<<std::endl;

                state = WAITING;
                std::cout<<"go to waiting"<<std::endl;
                break;
            }

            case WAITING:
            {
                if (go)
                {
                    std::cout<<"go data"<<std::endl;
                    //std::cout<<result.data[0]<<std::endl;
                    std::cout<<ef_frame.p[0]<<std::endl;
                    //ac.sendGoal(goal);
                    //ac.waitForResult();
                    //result = ac.getResult()->position;
                    //ik_handler.setToolOffsets(Eigen::Vector3d(result.data[0]/1000.0,result.data[1]/1000.0,result.data[2]/1000.0+object_heigth));
                    //ef_frame.p[0] = ef_frame.p[0];//-result.data[0]/1000.0;
                    //ef_frame.p[1] = ef_frame.p[1];//-result.data[1]/1000.0;
                    //ef_frame.p[2] = ef_frame.p[2];//-result.data[2]/1000.0-object_heigth;
                    state = WORKING;
                    std::cout<<"go to working"<<std::endl;
                }
                break;

            }

            case WORKING:           
            {
                ef_frame.M = KDL::Rotation::RotX(rotation);
                if (rotation < 1)
                {
                    delta = delta + unit;
                    rotation = rotation + unit;
                }
                success = ik_handler.ik_neg5(ef_frame,temp,false,reference_pose);
                if (success)
                {
                    for (int i =0;i<6;i++){
                        reference.position[i] = reference_pose[i];
                        if (std::isnan(reference.position[i])){
                            std::cout<<"out of bounds"<<std::endl;
                            out_of_bounds = true;
                        }
                    }
                    if(out_of_bounds) break;
                    joint_pub.publish(reference);
                }
                if (delta>(1/180.0*M_PI))
                {
                    delta = 0;
                    go = false;
                    std::cout<<"rotation: "<<rotation<<std::endl;
                    state = WAITING;
                }
            }
            default:           ;

        }
        

		ros::spinOnce();
		r.sleep();		
	}
	return 0;
}