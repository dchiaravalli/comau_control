#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
ros::Publisher pub;
ros::Publisher pub1;



void force_cb(const geometry_msgs::WrenchStamped &msg)
{
    std_msgs::Float64 result;
    geometry_msgs::WrenchStamped new_wrench;
    new_wrench = msg;
    new_wrench.wrench.force.x += 2230; 
    new_wrench.wrench.force.y -= 1135; 
    new_wrench.wrench.force.z -= 41131; 
    result.data = sqrt(pow(new_wrench.wrench.force.x,2)+ pow(new_wrench.wrench.force.y,2)+pow(new_wrench.wrench.force.z,2));

    pub.publish(new_wrench);
    pub1.publish(result);
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_trajectory_follower");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::WrenchStamped>("mean_force",1);
    pub1 = n.advertise<std_msgs::Float64>("mean_force_module",1);
    ros::Subscriber sub = n.subscribe("ur10e_right/wrench",1,force_cb);
    ros::spin();
    return 0;
}
