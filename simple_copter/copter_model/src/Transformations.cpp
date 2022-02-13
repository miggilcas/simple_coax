//inclusion de las librerías necesarias para ros y los tipos de mensajes usados en el paquete
/*
    Para obtener las coordenadas en el sistema de referencia no inercial necesitamos de una transformacion
	mediante una matriz de rotacion con angulos de euler. Con este nodo se consiguen esas coordenadas con
	el nombre NED donde N expresa la coordenada X, E la coordenada Y y D la coordenada Z.
	Es necesario comentar que al poseer el sistema unos ángulos de euler tan pequeños, acabarán siendo 
	más o menos iguales que los parámetros expresados en el sistema de referencia inercial.

*/
 

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <sstream>



// Programa de multiplicacion de dos matrices
using namespace std;
const int n = 3, q = 2, m = 3, p = 3;


// definicion de variables globales:
	//Definicion de mensajes a utilizar: 
		geometry_msgs::Vector3 RPY;
		geometry_msgs::Vector3 xyz;

		geometry_msgs::Vector3 NED;

        
	

//nos subscribimos al modelo para obtener los angulos de euler
float phi=1,theta=1,psi=0;
//Transformaciones de sistemas de referencia:
float R[n][m]={{1,sin(phi)*tan(theta),cos(phi)*tan(theta)},
                {0,cos(phi),          -sin(phi)},
                {0,sin(phi)/cos(theta),cos(phi)/cos(theta)}};

float A[n][m]={{cos(theta)*cos(psi),                              cos(theta)*sin(phi),                            -sin(theta)},
                {-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi),  sin(phi)*cos(theta)},
                {sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi),  cos(phi)*cos(theta)}};;
float AT[n][m]={{cos(theta)*cos(psi),-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi),  sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)},
                {cos(theta)*sin(phi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)},
                {        -sin(theta),                            sin(phi)*cos(theta),  cos(phi)*cos(theta)}};;



//Funciones:
//Operaciones con matrices:
void matmult(float mat1[][n], float mat2[][n], float mat3[][n])
{
//Comprobacion de compatibilidad de dimensiones
if (n != p) {
cout << " Dimensiones incorrectas. No se pueden multiplicar las matrices" << endl;
exit(1);
}
//Multiplicacion de matrices
	for (int i = 0; i < m; i++)
		for (int j = 0; j < m; j++)
		mat3[i][j] = 0.;
	for (int i = 0; i < m; i++)
		for (int j = 0; j < m; j++)
			for (int k = 0; k < m; k++)
			mat3[i][j] += mat1[i][k] * mat2[k][j];
			
			
}
void extrae_vect(float m1[][n],float *v){
	for (int i = 0; i < m; i++)
		for (int j = 0; j < m; j++)
		{
			if(m1[i][j]!=0){
				v[i]=m1[i][j];
			}
		}
}
void crea_matriz(float maux[][n],float *v){
	for (int i = 0; i < m; i++)
		for (int j = 0; j < m; j++)
		{
			if(j==0){
				maux[i][j]=v[i];
			}else{
				maux[i][j]=0;
			}
		}
}

//operaciones con vectores:
void prod_vec (float *v1, float *v2,float *v3)
{
v3[0] = v1[1]*v2[2]-v1[2]*v2[1];
v3[1] = v1[2]*v2[0]-v1[0]*v2[2];
v3[2] = v1[0]*v2[1]-v1[1]*v2[0];
}
void sum_vec (float *v1, float *v2,float *v3)
{
	for(int i=0; i < m; ++i)
	v3[i]=v1[i]+v2[i];
}

void mult_v_num(float x,float*v){
	for(int i=0; i < m; ++i)
	v[i]=x*v[i];
}

//Funciones callback:
void EulerCallback(const geometry_msgs::Vector3::ConstPtr & message);
void xyzCallback(const geometry_msgs::Vector3::ConstPtr & message);

int main(int argc, char **argv){
	//variable local
	float xyz_[n],NED_[n];
	float maux[n][m];
	float maux2[n][m];
	//adaptacion a ROS:
    ros::init(argc, argv, "Transformations");
 	ros::NodeHandle n;
	
	//Defino las subscripciones y donde publica:
	ros::Publisher ned_pub = n.advertise<geometry_msgs::Vector3>("copter_model/NED", 1000);
	ros::Subscriber Euler_sub = n.subscribe("copter_model/EulerModel", 1000, EulerCallback);
	ros::Subscriber xyz_sub = n.subscribe("copter_model/Pose", 1000, xyzCallback);

	// lo hacemos mil veces por segundo	
	ros::Rate loop_rate(1000);
	
	 int count = 0;
	ROS_INFO("Node: Transformations ready");
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	
	while (ros::ok())
	{
	
	//Actualizacion de angulos:
	phi=RPY.x;
	theta=RPY.y;
	psi=RPY.z;
	//Actualizacion de la matriz de transformacion:
	//Fila 1
	AT[0][0]= cos(theta)*cos(psi);
	AT[0][1]=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
	AT[0][2]= sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
	//Fila 2
	AT[1][0]= cos(theta)*sin(phi);
	AT[1][1]= cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
	AT[1][2]=-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
	//Fila 3
	AT[2][0]=-sin(theta);
	AT[2][1]=sin(phi)*cos(theta);
	AT[2][2]=cos(phi)*cos(theta);

	//metemos la posicion del sistema inercial en variables locales:
	xyz_[0]=xyz.x;
	xyz_[1]=xyz.y;
	xyz_[2]=xyz.z;
	
	//Calculos:
	crea_matriz(maux,xyz_);
	matmult(AT, maux, maux2);
	extrae_vect(maux2,NED_);

	//Resultado
	NED.x=NED_[0];
	NED.y=NED_[1];
	NED.z=NED_[2];

	
	ned_pub.publish(NED);
	
	
	ros::spinOnce();

	loop_rate.sleep();
}
    return 0;
}
void EulerCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  RPY.x=message->x;
  RPY.y=message->y;
  RPY.z=message->z;
  
}
void xyzCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  xyz.x=message->x;
  xyz.y=message->y;
  xyz.z=message->z;
  
}
