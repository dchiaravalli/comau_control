#include "ros/ros.h"
#include <iostream>
#include "kdl_conversions/kdl_msg.h"
#include "comau_control/IKService.h"
#include "comau_control/ComauSmartSix.h"
#include "comau_control/ComauIK.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <fstream>


comau_control::ComauIK *robot;
ros::ServiceClient service;

comau_control::IKService srv;




int main(int argc, char **argv) {
    ros::init(argc, argv, "testk");
    ros::NodeHandle nh;

    std::vector<double> temp,niet;


    robot = new comau_control::ComauIK();

    //Publishers and subscribers

    service = nh.serviceClient<comau_control::IKService>("comau_ik_service/ik_service_neg");

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(3);
    point.accelerations.resize(3);
    point.positions[0]=1;
    point.positions[1]=0;
    point.positions[2]=1.006;
    point.accelerations[0]=0;
    point.accelerations[1]=0;
    point.accelerations[2]=0;
    srv.request.target_pose=point;
    srv.request.wrist_compensation.data=false;
    srv.request.restart.data=true;


        if (service.call(srv)){

                for(int i=0;i<srv.response.q_out.data.size();i++){
                    std::cout<<srv.response.q_out.data[i]<<std::endl;

            }

        }
        else std::cout<<"failed"<<std::endl;
        /*
        temp.resize(6);
        temp[0]=0;
        temp[1]=0;
        temp[2]=-M_PI/2;
        temp[3]=0;
        temp[4]=-M_PI/2;
        temp[5]=M_PI;
        niet=robot->fk(temp);
        std::cout<<niet[0]<<std::endl;
        std::cout<<niet[1]<<std::endl;
        std::cout<<niet[2]<<std::endl;*/
    std::cout<<"done"<<std::endl;
    //ros::spin();

    return 0;
}