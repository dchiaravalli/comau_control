#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <ss_exponential_filter/Liquid_handler.h>


Eigen::Vector3d force;
Eigen::Vector3d torque;

void sensor_cb(const geometry_msgs::Twist &msg){
    force[0] = msg.linear.x -2.105;
    force[1] = msg.linear.y +0.625;
    force[2] = msg.linear.z -192.175;
    torque[0] = msg.angular.x +0.182;
    torque[1] = msg.angular.y -0.8285;
    torque[2] = msg.angular.z +10.125;
}


int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "comau_sensor_test");
    ros::NodeHandle n;

    ros::Subscriber sensor_sub = n.subscribe("/atift",1,sensor_cb);

    ss_exponential_filter::Liquid_handler handler(n,"/atift");


	ros::Rate r(500);
   while(ros::ok()){
    /*std::cout<<"- - - - - - - - - - - - - - - - - - - -"<<std::endl;
    std::cout<<"linear:"<<std::endl;
    std::cout<<"  x:  "<<force[0]<<std::endl;
    std::cout<<"  y:  "<<force[1]<<std::endl;
    std::cout<<"  z:  "<<force[2]<<std::endl;
    std::cout<<"angular:"<<std::endl;
    std::cout<<"  x:  "<<torque[0]<<std::endl;
    std::cout<<"  y:  "<<torque[1]<<std::endl;
    std::cout<<"  z:  "<<torque[2]<<std::endl;*/

    std::cout<<"- - - - - - - - - - - - - - - - - - - -"<<std::endl;
    std::cout<<"linear:"<<std::endl;
    std::cout<<"  x:  "<<handler.getMinForceMeasure()[0]<<std::endl;
    std::cout<<"  y:  "<<handler.getMinForceMeasure()[1]<<std::endl;
    std::cout<<"  z:  "<<handler.getMinForceMeasure()[2]<<std::endl;
    std::cout<<"angular:"<<std::endl;
    std::cout<<"  x:  "<<handler.getMinTorqueMeasure()[0]<<std::endl;
    std::cout<<"  y:  "<<handler.getMinTorqueMeasure()[1]<<std::endl;
    std::cout<<"  z:  "<<handler.getMinTorqueMeasure()[2]<<std::endl;

    std::cout<<"- - - - - - - - - - - - - - - - - - - -"<<std::endl;
    std::cout<<"linear:"<<std::endl;
    std::cout<<"  x:  "<<handler.getMaxForceMeasure()[0]<<std::endl;
    std::cout<<"  y:  "<<handler.getMaxForceMeasure()[1]<<std::endl;
    std::cout<<"  z:  "<<handler.getMaxForceMeasure()[2]<<std::endl;
    std::cout<<"angular:"<<std::endl;
    std::cout<<"  x:  "<<handler.getMaxTorqueMeasure()[0]<<std::endl;
    std::cout<<"  y:  "<<handler.getMaxTorqueMeasure()[1]<<std::endl;
    std::cout<<"  z:  "<<handler.getMaxTorqueMeasure()[2]<<std::endl;

    std::cout<<handler.getSensorState()<<std::endl;

    std::cout<<"filter parameters: "<<handler.getFilterParameters()[0]<<" "<<handler.getFilterParameters()[1]<<" "<<handler.getFilterParameters()[2]<<" "<<std::endl;
    std::cout<<"tool offset: "<<handler.getToolOffset()[0]<<" "<<handler.getToolOffset()[1]<<" "<<handler.getToolOffset()[2]<<" "<<std::endl;
    
    		ros::spinOnce();
		r.sleep();		
	}

	return 0;
}
	
	
