#include "ros/ros.h"
#include "math.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
/**
 * Esto es nuestro nodo que simula una IMU
 */
//global variables
geometry_msgs::Accel latestAccel;
geometry_msgs::Vector3 RPY;//Roll Pitch Yaw
float phi,theta;
void dataCallback(const geometry_msgs::Accel::ConstPtr & message)
{
  latestAccel = *message;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "IMU");

  
  ros::NodeHandle n;
  ros::Publisher Angle_pub = n.advertise<geometry_msgs::Vector3>("copter_model/EulerAngles", 100);
  ros::Subscriber sub = n.subscribe("copter_model/Accel", 1000, dataCallback);
  ROS_INFO("Node: 'IMU' ready");
  ros::Rate loop_rate(1000);
   while (ros::ok())
	{
  //calculamos los angulos de Euler [rad]: (se le añadira ruido gaussiano mas adelante)
  RPY.x=atan(latestAccel.linear.x/sqrt(pow(latestAccel.linear.y,2)+pow(latestAccel.linear.z,2)));
  RPY.y=atan(latestAccel.linear.y/sqrt(pow(latestAccel.linear.x,2)+pow(latestAccel.linear.z,2)));
  RPY.z=0.0;

  Angle_pub.publish(RPY);
 
  ros::spinOnce();

	loop_rate.sleep();
}

  return 0;
}
