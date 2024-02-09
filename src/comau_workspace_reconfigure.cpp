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

bool configure,out_of_bounds;
std::vector<double> joint_pose,reference_pose;
KDL::Frame ef_frame;

void jointStatesCallback(const sensor_msgs::JointState &msg){
    
    for(int i =0;i<6;i++){
        joint_pose[i] = msg.position[i];
    }
    configure = true;
}

void configCallback(comau_control::comau_poseConfig &config,uint32_t level){
    /*std::cout<<config.end_effector_x<<std::endl;
    std::cout<<config.end_effector_y<<std::endl;
    std::cout<<config.end_effector_z<<std::endl;
    std::cout<<config.end_effector_roll<<std::endl;
    std::cout<<config.end_effector_pitch<<std::endl;
    std::cout<<config.end_effector_yaw<<std::endl;*/
    if (config.activate_motion){
        ef_frame.p = KDL::Vector(config.end_effector_x*1000,config.end_effector_y*1000,config.end_effector_z*1000);
        ef_frame.M = KDL::Rotation::RPY(config.end_effector_roll,config.end_effector_pitch,config.end_effector_yaw); 
    }
}


enum State{
    START,
    INIZIALIZATION,
    WORKING
};


int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_trajectory_follower");
    ros::NodeHandle n;

    sensor_msgs::JointState reference;

    //variable initialization
    configure = false;
    joint_pose.resize(6);
    std::fill(joint_pose.begin(),joint_pose.end(),0.0);
    reference_pose.resize(6);
    std::fill(reference_pose.begin(),reference_pose.end(),0.0);
    reference.position.resize(6);
    out_of_bounds = false;

    //  Subscribers 
    ros::Subscriber joint_sub = n.subscribe("/comau_smart_six/joint_states", 1, jointStatesCallback);

    // Publishers
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command", 1);

    //state machine definition and initialization
    State state = START;
    Eigen::Vector3d temp(0,0,0);
    bool success = true;

    /*dynamic_reconfigure::Server<comau_control::comau_poseConfig> dr_srv;  
    dynamic_reconfigure::Server<comau_control::comau_poseConfig>::CallbackType cb;
    cb = boost::bind(&configCallback, _1, _2);
    dr_srv.setCallback(cb);*/
    //FK/IK handler
    comau_control::ComauIK ik_handler(127,0,-50);

    reference.position[0]=0;
    reference.position[1]=1;
    reference.position[2]=-1;
    reference.position[3]=0;
    reference.position[4]=1.2;
    reference.position[5]=0;
    reference.header.frame_id="filtered";


	ros::Rate r(500);

    dynamic_reconfigure::Server<comau_control::comau_poseConfig> dr_srv;  
    dynamic_reconfigure::Server<comau_control::comau_poseConfig>::CallbackType cb;
    cb = boost::bind(&configCallback, _1, _2);
    dr_srv.setCallback(cb);
	while(ros::ok())
	{   
        out_of_bounds = false;

        switch(state){

            case START:             {
                                        if (configure) state = INIZIALIZATION;
                                        break;
            }

            case INIZIALIZATION:    {
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

                                        state = WORKING;
                                        break;
            }

            case WORKING:           {
                                        success = ik_handler.ik_neg5(ef_frame,temp,false,reference_pose);
                                        if (success){
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
                                        /*std::cout<<"joints"<<std::endl;
                                        for (int i=0;i<6;i++){
                                            std::cout<<reference.position[i]<<std::endl;
                                        }*/
                                        //joint_pub.publish(reference);
            }


            default:           ;

        }
        

		ros::spinOnce();
		r.sleep();		
	}
	return 0;
}
	
	
