#include <ros/ros.h>
#include <comau_control/ComauTrajectoryServer.h>

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "matlab_trajectory_follower");
    ros::NodeHandle n;
    comau_control::ComauTrajectoryServer server(n);
    server.run();

    return 0;
}