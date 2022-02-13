//inclusion de las librerÃ­as necesarias para ros y los tipos de mensajes usados en el paquete
/*
    Este nodo nos servira para comunicarle al modelo de fuerzas y momentos
	las fuerzas y momentos que se deben aplicar.
	Se subscribira al controlador para obtener las acciones de control, con estas 
	generara las fuerzas y momentos, parametros que debe publicar.
	07/01/22: Preparado para controlar, recibir las señales del control
	09/01/22: Control de altura realizado
	10/01/22: Cambio la IMU para obtener Roll, Pitch y Yaw sacándola del modelo
	11/01/22: Control de velocidad implementado.
    12/01/22: Dame implementa el control en posicion.
*/
#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <sstream>

#define PI 3.1415

// Programa de multiplicacion de dos matrices
using namespace std;
const int n = 3, q = 2, m = 3, p = 3;



// Variables globales:
	geometry_msgs::Vector3 Fuerza;
	geometry_msgs::Vector3 Momento;
	//las que recibe del control:
	std_msgs::Float64 W;
	geometry_msgs::Vector3 Control;
	std_msgs::Float64 yawControl;
//constantes:
	float g=9.8;
	float masa =0.095; //0.280;//masa del helicoptero según el TFG, nos interesa la de la tesis: 0.095
	float RollArmLength=0.055;
	float PitchArmLength=0.091;//obtenidos de la tesis
//Funciones callback para actualizar los valores a recibir:
void controlCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  Control.x = message->x;
  Control.y = message->y;
  Control.z = 0;
}
void zCallback(const std_msgs::Float64::ConstPtr & message)
{
  W = *message;
}
void yawCallback(const std_msgs::Float64::ConstPtr & message)
{
  yawControl = *message;
}
//Funciones para le depuración en un compilador de C normal:

void muestra_v(float *v)
{
    for(int i = 0; i < m; ++i) {
    cout << v[i] << " ";
		cout << endl;	
    }
}

int main(int argc, char **argv){
	
	//Entradas:
	float w1=500,w2=500;//velocidades angulares de los rotores
	float alfax=0,alfay=0;//angulos de euler del rotor inferior con respecto al chasis
	/*
	Tras hacer pruebas se observa que el alfax conotrola la posicion y velocidad en y y viceversa
	*/
	//b (Factor de empuje) y d(factor de arrastre) son calculados a partir de w1 y w2.
	float b=0.00000254133;//lo suponemos constante: 0.0875⁴*0.0138*pi
	float d=0.05*b;
	//salidas:
	float F[n];
	float M[n];
	//variables necesarias:
	float vaux[n];
	float T1,T2,T2e,T3,T4;
	float DT1,DT2,DT2e,DT3,DT4;
	
	

	//Pasamos a ROS:
	ros::init(argc, argv, "F_M_generator");
 	ros::NodeHandle n;
	//publicadores de las fuerzas y momentos:
	ros::Publisher F_pub = n.advertise<geometry_msgs::Vector3>("copter_model/Fuerza", 100);
	ros::Publisher M_pub = n.advertise<geometry_msgs::Vector3>("copter_model/Momento", 100);
	
	//Para obtener las variables de control:
	ros::Subscriber AngleSub = n.subscribe("copter_control/AngleControl", 1000, controlCallback);
	ros::Subscriber Zsub = n.subscribe("copter_control/ZControl", 1000, zCallback);
	ros::Subscriber yawsub = n.subscribe("copter_control/yawControl", 1000,yawCallback);
    // Frecuencia de 1 KHz
	ros::Rate loop_rate(1000);
	ROS_INFO("Node: F_M_generator ready");
	while (ros::ok())
	{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*///Descomentar las lineas siguientes para controlar:
	w1=W.data/2;w2=w1;//Primero control de altura
	alfay=Control.x;
	alfax=Control.y;
	//Calculo Fuerzas y Pares:
	//Fuerza generada por el rotor superior:
	T1=b*pow(w1,2);
	//Fuerza generada por el rotor inferior:
	T2=b*pow(w2,2);
	//Fuerza efectiva ascendente:
	T2e=T2*cos(alfax)*cos(alfay);
	//Fuerza en el ejex:
	T3=T2*cos(alfax)*sin(alfay);
	//Fuerza en el ejey:
	T4=T2*cos(alfay)*sin(alfax);
	
	//Pares de arrastre sobre el solido:
	//Par generado por el rotor superior:
	DT1=d*pow(w1,2);
	//Pares generado por el rotor inferior:
		//par de guiñada:
		DT2=d*pow(w2,2);
		//Par efectivo ascendente:
		DT2e=DT2*cos(alfax)*cos(alfay);
		//Par en el ejex:
		DT3=DT2*cos(alfax)*sin(alfay);
		//Par en el ejey:
		DT4=DT2*cos(alfay)*sin(alfax);
	
		
	/*  Ecuaciones para obtener el sumatorio de FyM:
		Fuerzas=[-T3 ;
				  T4;
				  g*m -T1_T2e];
		Momentos=[ T4*RollArmLength-DT3 ;
				  -T3*PitchArmLength+DT4;
					DT1-DT2e];
		(RollArmLenght y PitchArmLength es la distancia desde el rotor inferior al
        centro de masas que produce un par debido al empuje.)
	*/
	F[0]=-T3;
	F[1]=T4;
	F[2]=-T1-T2e;
	
	M[0]=T4*RollArmLength-DT3 ;
	M[1]=-T3*PitchArmLength+DT4;
	M[2]=DT1-DT2e+yawControl.data;
	
	//rellenamos la aceleracion con los valores calculados:
	Fuerza.x = F[0];
	Fuerza.y = F[1];
	Fuerza.z = F[2];
	Momento.x = M[0];
	Momento.y = M[1];
	Momento.z = M[2];
	

	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	F_pub.publish(Fuerza);
	M_pub.publish(Momento);

	ros::spinOnce();

	loop_rate.sleep();
	}

    return 0;
}


