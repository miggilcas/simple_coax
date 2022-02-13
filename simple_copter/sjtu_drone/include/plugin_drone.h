#ifndef ARDRONE_SIMPLE_CONTROL_H
#define ARDRONE_SIMPLE_CONTROL_H

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "ignition/math4/ignition/math.hh"
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

//copter

#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <math.h>
#include <sstream>
#include <tf/tf.h>
////////////////////////

#include <sensor_msgs/Imu.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include "pid_controller.h"

#define LANDED_MODEL        0
#define FLYING_MODEL        1
#define TAKINGOFF_MODEL     2
#define LANDING_MODEL       3
#define PI                  3.141592
#define EPS 1E-6

namespace gazebo
{
class DroneSimpleController : public ModelPlugin
{
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  void UpdateDynamics(double dt);
  void UpdateState(double dt);
  virtual void Reset();

private:
  double m_timeAfterCmd;
  bool m_posCtrl;
  bool m_velMode;
  unsigned int navi_state;



  //copter
  

// Programa de multiplicacion de dos matrices



        // Variables globales:
	geometry_msgs::Vector3 Fuerza;
	geometry_msgs::Vector3 Momento;
	//las que recibe del control:
	geometry_msgs::Vector3 Euler_copter;
	geometry_msgs::Vector3 NED_copter;
	geometry_msgs::Twist Vel_copter;

        //constantes:
	float g=9.8;
	float masa = 0.095;//masa del helicÃ³ptero
	float RollArmLength=0.051;
	float PitchArmLength=0.051;
	//Entradas:
	float w1=500,w2=500;//velocidades angulares de los rotores
	float alfax=0,alfay=0;//angulos de euler del rotor inferior con respecto al chasis
	/*
	Tras hacer pruebas se observa que el alfax conotrola la posicion y velocidad en y y viceversa
	*/
	//b (Factor de empuje) y d(factor de arrastre) son calculados a partir de w1 y w2.
	float b=0.00000254133;//lo suponemos constantemasa*g/(pow(w1,2)+pow(w2,2)*cos(alfax)*cos(alfay));//cambiar por el valor calculado
	float d=0.05*b;
        float Vmin=2*sqrt((masa*9.8)/(2*b));//~1469.5;
	float T1,T2,T2e,T3,T4;
	float DT1,DT2,DT2e,DT3,DT4;
	void EulerModelCallback(const geometry_msgs::Vector3::ConstPtr&);
	void NEDCallback(const geometry_msgs::Vector3::ConstPtr&);
	void VelCallback(const geometry_msgs::Twist::ConstPtr&);
	int flag=1;

	//Para obtener las variables de control:
	//ros::Subscriber AngleSub;
	//ros::Subscriber Zsub;

  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber cmd_subscriber_;
  ros::Subscriber posctrl_subscriber_;
  ros::Subscriber imu_subscriber_;
  
  // extra robot control command
  ros::Subscriber takeoff_subscriber_;
  ros::Subscriber land_subscriber_;
  ros::Subscriber reset_subscriber_;
  ros::Subscriber switch_mode_subscriber_;
  //copter
  ros::Subscriber EulerModel_subscriber;
  ros::Subscriber  NED_subscriber;
  ros::Subscriber Vel_subscriber;
  
  ros::Publisher pub_gt_pose_;   //for publishing ground truth pose
  ros::Publisher pub_gt_vec_;   //ground truth velocity in the body frame
  ros::Publisher pub_gt_acc_;   //ground truth acceleration in the body frame


  geometry_msgs::Twist cmd_val;
  // callback functions for subscribers
  void CmdCallback(const geometry_msgs::TwistConstPtr&);
  void PosCtrlCallback(const std_msgs::BoolConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void TakeoffCallback(const std_msgs::EmptyConstPtr&);
  void LandCallback(const std_msgs::EmptyConstPtr&);
  void ResetCallback(const std_msgs::EmptyConstPtr&);
  void SwitchModeCallback(const std_msgs::BoolConstPtr&);
 
 
  ros::Time state_stamp;
  ignition::math::Pose3d pose;
  ignition::math::Vector3d euler, velocity, acceleration, angular_velocity, position;

  std::string link_name_;
  std::string cmd_normal_topic_;
  std::string switch_mode_topic_;
  std::string posctrl_topic_;
  std::string imu_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string reset_topic_;
  std::string gt_topic_;    //ground truth
  //copter
  std::string EulerModel_topic;
  std::string  NED_topic;
  std::string  Vel_topic;

  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
    PIDController alpha_x;
    PIDController alpha_y;
    PIDControllerZ W_z;
  } controllers_;

  ignition::math::Vector3d inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif // ARDRONE_SIMPLE_CONTROL_H
