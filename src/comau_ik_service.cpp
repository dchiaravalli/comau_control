#include "ros/ros.h"
#include <Eigen/Dense>
#include "kdl_conversions/kdl_msg.h"
#include "comau_control/IKService.h"
#include "comau_control/IKServiceTotal.h"
#include "comau_control/ComauIK.h"

ros::NodeHandle *nh;

comau_control::ComauIK *robot;

bool ik(comau_control::IKService::Request &req, comau_control::IKService::Response &res) {
    bool comp;
    if(req.restart.data)
        robot->reset();
    KDL::Vector v;
    KDL::Frame target_frame;

    if (req.wrist_compensation.data){

        v=KDL::Vector(req.target_pose.accelerations[0],req.target_pose.accelerations[1],req.target_pose.accelerations[2]);
                            std::cout<<"qui"<<std::endl;
        comp=true;
    }
    else
        comp=false;

        res.q_out.data.resize(6);
    target_frame.p=KDL::Vector(1000*req.target_pose.positions[0],1000*req.target_pose.positions[1],1000*req.target_pose.positions[2]);
    target_frame.M=KDL::Rotation::RPY(0,0,0);
    //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*req.target_pose.positions[1],1000*req.target_pose.positions[0]));
    bool state = robot->ik(target_frame,v,comp,res.q_out.data);
    if (state){
        return true;
    }
    return false;
}

bool ik_neg(comau_control::IKService::Request &req, comau_control::IKService::Response &res) {
    bool comp;
    if(req.restart.data)
        robot->reset();
    Eigen::Vector3d v;
    KDL::Frame target_frame;

    if (req.wrist_compensation.data){

        v=Eigen::Vector3d(req.target_pose.accelerations[0],req.target_pose.accelerations[1],req.target_pose.accelerations[2]);
                            std::cout<<"qui"<<std::endl;
        comp=true;
    }
    else
        comp=false;

        res.q_out.data.resize(6);
    target_frame.p=KDL::Vector(1000*req.target_pose.positions[0],1000*req.target_pose.positions[1],1000*req.target_pose.positions[2]);
    target_frame.M=KDL::Rotation::RPY(0,0,0);
    //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*req.target_pose.positions[1],1000*req.target_pose.positions[0]));
    bool state = robot->ik_neg5(target_frame,v,comp,res.q_out.data);
    if (state){
        return true;
    }
    return false;
}

bool iktraj(comau_control::IKServiceTotal::Request &req, comau_control::IKServiceTotal::Response &res) {
    bool comp;
    robot->reset();
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);
    std::vector<double> q_out;
    q_out.resize(6);
    KDL::Frame target_frame;
    if (req.wrist_compensation.data){
        comp=true;
    }
    else
        comp=false;
    for (int i=0;i<req.target_traj.points.size();i++){
        target_frame.p=KDL::Vector(1000*req.target_traj.points[i].positions[0],1000*req.target_traj.points[i].positions[1],1000*req.target_traj.points[i].positions[2]);
        //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*req.target_traj.points[i].positions[1],1000*req.target_traj.points[i].positions[0]));
        target_frame.M=KDL::Rotation::RPY(0,0,0);
        KDL::Vector v(req.target_traj.points[i].accelerations[0],req.target_traj.points[i].accelerations[1],req.target_traj.points[i].accelerations[2]);
        bool state = robot->ik(target_frame,v,comp,q_out);
        if (state){
            for (int j=0;j<6;j++){
                point.positions[j]=q_out[j];
            }
            res.q_out.points.push_back(point);
        }
    }
    return true;
}

bool iktraj_neg(comau_control::IKServiceTotal::Request &req, comau_control::IKServiceTotal::Response &res) {
    bool comp;
    robot->reset();
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);
    std::vector<double> q_out;
    q_out.resize(6);
    KDL::Frame target_frame;
    if (req.wrist_compensation.data){
        comp=true;
    }
    else
        comp=false;
    for (int i=0;i<req.target_traj.points.size();i++){
        target_frame.p=KDL::Vector(1000*req.target_traj.points[i].positions[0],1000*req.target_traj.points[i].positions[1],1000*req.target_traj.points[i].positions[2]);
        //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*req.target_traj.points[i].positions[1],1000*req.target_traj.points[i].positions[0]));
        target_frame.M=KDL::Rotation::RPY(0,0,0);
        Eigen::Vector3d v(req.target_traj.points[i].accelerations[0],req.target_traj.points[i].accelerations[1],req.target_traj.points[i].accelerations[2]);
        bool state = robot->ik_neg5(target_frame,v,comp,q_out);
        if (state){
            for (int j=0;j<6;j++){
                point.positions[j]=q_out[j];
            }
            res.q_out.points.push_back(point);
        }
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "comau_ik_service");
    nh = new ros::NodeHandle("~");
    //robot = new comau_control::ComauIK(127.5,0,209);
    robot = new comau_control::ComauIK(0,0,0);

    ros::ServiceServer service = nh->advertiseService("ik_service", ik);
    ros::ServiceServer serviceTraj = nh->advertiseService("ik_service_traj", iktraj);
    ros::ServiceServer service_neg = nh->advertiseService("ik_service_neg", ik_neg);
    ros::ServiceServer serviceTraj_neg = nh->advertiseService("ik_service_traj_neg", iktraj_neg);
    ros::spin();

    return 0;
}