#include "ros/ros.h"
#include "math.h"

#include "std_msgs/String.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Bool.h>

#define PI 3.1415

/**
 * Este nodo es un generador de trayectorias simple para demostrar el funcionamiento 
 * de nuestro sistema siguiendo una trayectoria muy sencilla determinada por una cantidad
 * pequeña de Waypoints. 
 * Realizaremos 4 tipos de experimentos, llamados por los diferentes launch:
 *    +Hovering
 *    +Square
 *    +2 altitudes Square
 *    +4 altitudes Square
 */
//global variables
geometry_msgs::Point Waypoint;//X Y Z references
std_msgs::Float64 Refyaw;
std_msgs::Bool modo_teleop;
int n_exp,n_exp_ant;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "Traj_gen");

  
  ros::NodeHandle n;
  //publicara las referencias de posicion para hacer la trayectoria deseada
  ros::Publisher Traj_pub = n.advertise<geometry_msgs::Point>("copter_control/Waypoints", 100);
  ros::Publisher yaw_pub = n.advertise<std_msgs::Float64>("copter_control/Refyaw", 100);
  ros::Publisher modo_teleop_pub = n.advertise<std_msgs::Bool>("copter_control/modo_teleop", 10);

  //Parameters
  n.param<int>("experiment", n_exp, 0); // se definirá en el launch
  n_exp_ant=n_exp;
  ROS_INFO("Node: 'Traj_gen' ready");
  ros::Rate loop_rate(10);
  //variables para poder tener noción del tiempo
  double currentTime=0;
  double elapsedTime=0;
  double previousTime=(double)ros::Time::now().toSec();

   while (ros::ok())
	{
    //Conteo del tiempo
  currentTime=(double)ros::Time::now().toSec();
	elapsedTime=(double)(currentTime-previousTime);
  n.param<int>("experiment", n_exp, 0);

  if(n_exp!=n_exp_ant) previousTime=(double)ros::Time::now().toSec();
  n_exp_ant=n_exp;
  switch(n_exp){
  case 0://REPOSO
  //Primer punto y último
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=0;
    Refyaw.data=0;
  modo_teleop.data=false;
  
  break;
  //vamos generando los waypoints en funcion del tiempo pasado
  case 1: //cuadrado de 1 metro de altura y 1 metro de lado
  if(elapsedTime<=2){//Punto inicial
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=0;
  }
  if(elapsedTime>2 && elapsedTime<=7){//Primer vertice
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-1;
  }
  if(elapsedTime>7 && elapsedTime<=13){//segundo
    Waypoint.x=1;
    Waypoint.y=0;
    Waypoint.z=-1;
  }
  if(elapsedTime>18 && elapsedTime<=23){//Tercero
    Waypoint.x=1;
    Waypoint.y=1;
    Waypoint.z=-1;
  }
  if(elapsedTime>28 && elapsedTime<=33){//Cuarto
    Waypoint.x=0;
    Waypoint.y=1;
    Waypoint.z=-1;
  }
  if(elapsedTime>38 && elapsedTime<=43){//Primero
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-1;
  }
  modo_teleop.data=false;
  break;

  case 2://cuadrado con dos vértices más elevados
    if(elapsedTime<=2){//Punto inicial
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=0;
  }
  if(elapsedTime>=2 && elapsedTime<=20){//Primer vertice
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-5;
  }
  if(elapsedTime>20 && elapsedTime<=40){//segundo
    Waypoint.x=2;
    Waypoint.y=0;
    Waypoint.z=-5;
  }
  if(elapsedTime>40 && elapsedTime<=60){//Tercero
    Waypoint.x=2;
    Waypoint.y=2;
    Waypoint.z=-7;
  }
  if(elapsedTime>60 && elapsedTime<=80){//Cuarto
    Waypoint.x=0;
    Waypoint.y=2;
    Waypoint.z=-7;
  }
  if(elapsedTime>80 && elapsedTime<=100){//Primero
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-5;
  }
  modo_teleop.data=false;
  break;
  case 3://cuatro vértices de alturas distintas todas
    if(elapsedTime<=5){//Punto de partida
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=0;
  }
  if(elapsedTime>=5 && elapsedTime<=15){//Primer punto
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-5;
  }
  if(elapsedTime>15 && elapsedTime<=25){//segundo
    Waypoint.x=3;
    Waypoint.y=0;
    Waypoint.z=-10;
  }
  if(elapsedTime>25 && elapsedTime<=35){//Tercero
    Waypoint.x=3;
    Waypoint.y=3;
    Waypoint.z=-3;
  }
  if(elapsedTime>35 && elapsedTime<=45){//Cuarto
    Waypoint.x=0;
    Waypoint.y=3;
    Waypoint.z=-8;
  }
  if(elapsedTime>45 && elapsedTime<=55){//Primero
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-2;
    //Refyaw.data=PI/2;
  }
  modo_teleop.data=false;
  break;
  case 4:
    if(elapsedTime<5){//Punto de partida
    Waypoint.x=0;
    Waypoint.y=5;
    Waypoint.z=-4;
    Refyaw.data=-PI/2;
  }
  if(elapsedTime>8 && elapsedTime<=15){//Primero
    Waypoint.x=0;
    Waypoint.y=4;
    Waypoint.z=-2;
    Refyaw.data=PI/2;//debe ser negativo para que se vea positivo en la simulación
  }
  if(elapsedTime>15 && elapsedTime<=18){//Primero
    Waypoint.x=0;
    Waypoint.y=4;
    Waypoint.z=-2;
    Refyaw.data=0;
  }
   if(elapsedTime>19){//Primero
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=-5;
    Refyaw.data=-3*PI/4;
  }
  modo_teleop.data=false;
  break;
  case 5:  //teleoperacion  
    Waypoint.x=0;
    Waypoint.y=0;
    Waypoint.z=0;
    Refyaw.data=0;
  modo_teleop.data=true;
  break;
  }
  
  modo_teleop_pub.publish(modo_teleop);
  Traj_pub.publish(Waypoint);
  yaw_pub.publish(Refyaw);
  ros::spinOnce();

	loop_rate.sleep();
}

  return 0;
}
