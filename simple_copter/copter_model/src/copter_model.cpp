//inclusion de las librerías necesarias para ros y los tipos de mensajes usados en el paquete
/*
    El modelo recibe como parametros de entrada:
    - roll, pitch y yaw, finalmente los generamos nosotros integrando
    + velocidades lineares->integrarla de la salida del modelo
    + velocidades angulares->integrarla de la salida del modelo
    + F
    + M

	*Publica:
		+aceleracion(lineal y angular)->copter_model/Accel (conseguido)
		+deberia publicar tambien posteriormente velocidades lineales(ya lo hace) y la altura para el control(da la posicion completa)
	*Se subscribe:
		+F y M->copter_model/FM (conseguido)
		-Angulos de euler (IMU nos lo aporta, hecha) subscribirse al topic donde publica, Esto lo cambiamos y trabajamos con los ángulos
		generados por el modelo,

*/
 

#include "ros/ros.h"

#include "std_msgs/String.h"

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



// Programa de multiplicacion de dos matrices
using namespace std;
const int n = 3, q = 2, m = 3, p = 3;


// definicion de variables globales:
	//Definicion de mensajes a utilizar: 
		geometry_msgs::Vector3 Fuerza;
		geometry_msgs::Vector3 Momento;

        geometry_msgs::Accel Acceleration;
		geometry_msgs::Twist Velocity;
		geometry_msgs::Point Pos;
		geometry_msgs::Vector3 RPY;//Roll Pitch Yaw

		//debug:
		std_msgs::Float64 Tpasado;
	//Variables para realizar la integracion:
		unsigned long currentTime,previousTime;
		double elapsedTime;
//Tensor inercial:
float I[n][m]={{1.12*pow(10,-4),0,0},{0,2.43*pow(10,-4),0},{0,0,2.66*pow(10,-5)}};
float I_inv[n][m]={{0.8929*pow(10,-4),0,0},{0,0.4115*pow(10,-4),0},{0,0,3.7594*pow(10,-4)}};

//por ahora tomamos como constantes los angulos de euler:
float phi=0,theta=0,psi=0;//son sustituidos por los recibidos de la IMU o no.
//Transformaciones de sistemas de referencia:
float R[n][m]={{1,sin(phi)*tan(theta),cos(phi)*tan(theta)},
                {0,cos(phi),          -sin(phi)},
                {0,sin(phi)/cos(theta),cos(phi)/cos(theta)}};
float masa = 0.095;//masa=0.280 [kg] del helicóptero segun el TFG, necesitamos la de la tesis:0.095
float g=9.8;
//Posicion:
float xyz[n]={0,0,0};
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
//Funciones para debug en un compilador normal:
void muestra(float v[][n])
{
    for(int i = 0; i < m; ++i) {
        for(int j = 0; j < n; ++j) 
            cout << v[i][j] << " ";
			cout << endl;
        
    }
}
void muestra_v(float *v)
{
    for(int i = 0; i < m; ++i) {
    cout << v[i] << " ";
		cout << endl;	
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
void sum_rest_vec (float *v1, float *v2,float *v3,int op) //para sumas op=1 para restas -1
{
	if(op==1){
	for(int i=0; i < m; ++i)
	v3[i]=v1[i]+v2[i];
	}
	if(op==-1){
	for(int i=0; i < m; ++i)
	v3[i]=v1[i]-v2[i];
	}
	
}
void mult_v_num(float x,float*v){
	for(int i=0; i < m; ++i)
	v[i]=x*v[i];
}
//Funciones callback:
void FCallback(const geometry_msgs::Vector3::ConstPtr & message);
void MCallback(const geometry_msgs::Vector3::ConstPtr & message);
void EulerCallback(const geometry_msgs::Vector3::ConstPtr & message);

int main(int argc, char **argv){
	
	/*Ecuaciones que interpretan el modelo:
	En matlab:
    	rpy_dot=R*pqr;

	uvw_dot=(1/m)*F-cross(pqr,uvw);
	pqr_dot=I^-1*(M-cross(pqr,(I*pqr)));
	*/
	//salidas:
	float rpy_dot[n]={0,0,0},pqr_dot[n]={0,0,0},uvw_dot[n]={0,0,0};
	//primera ec:
	float pqr[n]={0,0,0};//la obtendremos integrando, condiciones iniciales 0
	
	float maux[n][m];
	float maux2[n][m];
	//segunda ec:
	
	float F[n];
	float uvw[n]={0,0,0};//la obtendremos integrando, condiciones iniciales 0
	float vaux[n];
	//tercera ec:
	float M[n];
	
	
	//cout << " pqr_dot:" << endl;
	//muestra_v(pqr_dot);

	//adaptacion a ROS:
    ros::init(argc, argv, "model");
 	ros::NodeHandle n;
	
	//Defino las subscripciones y donde publica:
	ros::Publisher acc_pub = n.advertise<geometry_msgs::Accel>("copter_model/Accel", 100);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("copter_model/Vel", 100);
	ros::Publisher pos_pub = n.advertise<geometry_msgs::Point>("copter_model/Pose", 100);

	//debug:
	ros::Publisher time_pub = n.advertise<std_msgs::Float64>("copter_model/ElapsedTime", 100);
	ros::Publisher Angle_pub = n.advertise<geometry_msgs::Vector3>("copter_model/EulerModel", 100);


	ros::Subscriber F_sub = n.subscribe("copter_model/Fuerza", 1000, FCallback);
	ros::Subscriber M_sub = n.subscribe("copter_model/Momento", 1000, MCallback);
	ros::Subscriber Eul_sub = n.subscribe("copter_model/EulerAngles", 1000, EulerCallback);

	// Frecuencia de 1 KHz
	ros::Rate loop_rate(1000);
	
	 int count = 0;
	ROS_INFO("Node: copter_model ready");
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	previousTime=0;//la inicializamos antes de entrar en el bucle
	while (ros::ok())
	{
	currentTime=(unsigned long)ros::Time::now().toSec();
	elapsedTime=(double)(currentTime-previousTime);

	//Actualizamos las integrales (velocidades) para hacer los calculos de las aceleraciones:
	uvw[0]+=uvw_dot[0]*(float)elapsedTime;
	uvw[1]+=uvw_dot[1]*(float)elapsedTime;
	uvw[2]+=uvw_dot[2]*(float)elapsedTime;

	pqr[0]+=pqr_dot[0]*(float)elapsedTime;
	pqr[1]+=pqr_dot[1]*(float)elapsedTime;
	pqr[2]+=pqr_dot[2]*(float)elapsedTime;
	//Posicion:
	xyz[0]+=uvw[0]*(float)elapsedTime;
	xyz[1]+=uvw[1]*(float)elapsedTime;
	xyz[2]+=uvw[2]*(float)elapsedTime;
	
	//Angulos de Euler:
	//se hace ya que si no se obtienen ángulos de 90º o -90º
	phi+=rpy_dot[0]*(float)elapsedTime;
	theta+=rpy_dot[1]*(float)elapsedTime;
	psi+=rpy_dot[2]*(float)elapsedTime;
	RPY.x=phi;
	RPY.y=theta;
	RPY.z=psi;
	//RPY.z=0;//tambien se podría obtener pero simplificamos de esta forma
	//Rellenamos Fuerzas y Momentos con lo recibido por la subscricion:
	F[0]=Fuerza.x;
	F[1]=Fuerza.y;
	F[2]=Fuerza.z+g*masa ;//lo modificamos para meter el modelo en un campo gravitatorio
	
	M[0]=Momento.x;
	M[1]=Momento.y;
	M[2]=Momento.z;
	//Actualizamos la matriz de transformacion (con los ángulos integrados):
	/*R[0][1]=sin(RPY.x)*tan(RPY.y);
	R[0][2]=cos(RPY.x)*tan(RPY.y);
    R[1][1]=cos(RPY.x);
	R[1][2]=-sin(RPY.x);
    R[2][1]=sin(RPY.x)/cos(RPY.y);
	R[2][2]=cos(RPY.x)/cos(RPY.y);*/
    R[0][1]=sin(phi)*tan(theta);
	R[0][2]=cos(phi)*tan(theta);
    R[1][1]=cos(phi);
	R[1][2]=-sin(phi);
    R[2][1]=sin(phi)/cos(theta);
	R[2][2]=cos(phi)/cos(theta);
	//Calculo de las salidas:
	//Primera ecuacion modelo FyM:
	crea_matriz(maux,pqr);
	//muestra(maux);
	matmult(R, maux, maux2);
	//rpy_dot:
	extrae_vect(maux2,rpy_dot);
	//cout << " rpy_dot:" << endl;
	//muestra_v(rpy_dot); de aquí podemos obtener los ángulos de euler sin necesidad de hacer la IMU
	
	//Segunda ecuacion modelo FyM
	mult_v_num(1.0/masa,F);
	prod_vec(pqr,uvw,vaux);
	
	//uvw_dot:ros::Rate loop_rate(1);
	sum_rest_vec(F,vaux,uvw_dot,-1);
	//cout << " uvw_dot:" << endl;
	//muestra_v(uvw_dot);
	
	//Tercera ecuacion:
	matmult(I, maux, maux2);
	extrae_vect(maux2,vaux);
	prod_vec(pqr,vaux,vaux);
	sum_rest_vec(M,vaux,vaux,-1);
	crea_matriz(maux,vaux);
	matmult(I_inv,maux,maux2);
	extrae_vect(maux2,pqr_dot);
	//rellenamos la aceleracion con los valores calculados:
	Acceleration.linear.x = uvw_dot[0];
	Acceleration.linear.y = uvw_dot[1];
	Acceleration.linear.z = uvw_dot[2];
	Acceleration.angular.x = pqr_dot[0];
	Acceleration.angular.y = pqr_dot[1];
	Acceleration.angular.z = pqr_dot[2];
	//rellenamos la velocidad con los valores calculados:
	Velocity.linear.x = uvw[0];
	Velocity.linear.y = uvw[1];
	Velocity.linear.z = uvw[2];
	Velocity.angular.x = pqr[0];
	Velocity.angular.y = pqr[1];
	Velocity.angular.z = pqr[2];
	//rellenamos posiciones:
	Pos.x=xyz[0];
	Pos.y=xyz[1];
	Pos.z=xyz[2];
	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	//Saco el tiempo elapsed time para depurar errores:

	Tpasado.data=elapsedTime;
	acc_pub.publish(Acceleration);
	vel_pub.publish(Velocity);
	pos_pub.publish(Pos);
	time_pub.publish(Tpasado);
	//Angulos de euler del modelo
	Angle_pub.publish(RPY);
	previousTime=currentTime;//actualizamos el valor de previousTime
	ros::spinOnce();

	loop_rate.sleep();
}
    return 0;
}
void FCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  Fuerza.x=message->x;
  Fuerza.y=message->y;
  Fuerza.z=message->z;
  
}
void MCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  Momento.x=message->x;
  Momento.y=message->y;
  Momento.z=message->z;
  
}
void EulerCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  RPY.x=message->x;
  RPY.y=message->y;
  RPY.z=message->z;
  
}
