#include "plugin_drone.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo {

DroneSimpleController::DroneSimpleController()
{ 
  navi_state = LANDED_MODEL;
  m_posCtrl = false;
  m_velMode = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DroneSimpleController::~DroneSimpleController()
{
  // Deprecated since Gazebo 8.
  //event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  if(!ros::isInitialized()){
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
  }
  
  world = _model->GetWorld();
  ROS_INFO("The drone plugin is loading!");
  
  //load parameters
  cmd_normal_topic_ = "/cmd_vel";
  takeoff_topic_ = "drone/takeoff";
  land_topic_ = "drone/land";
  reset_topic_ = "drone/reset";
  posctrl_topic_ = "drone/posctrl";
  gt_topic_ = "drone/gt_pose";
  switch_mode_topic_ = "/drone/vel_mode";


  ///////////////////////// copter //////////////////////////////////
  EulerModel_topic = "/copter_model/EulerModel";
  NED_topic = "/copter_model/NED";
  Vel_topic = "/copter_model/Vel"; 
  ///////////////////////////////////////////////////////////////////
  

	/*ros::Subscriber AngleSub = node_handle_ -> subscribe("copter_model/AngleControl", 1000, DroneSimpleController::controlCallback);
	ros::Subscriber Zsub = node_handle_ -> subscribe("copter_model/ZControl", 1000, DroneSimpleController::zCallback);*/
 
  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
  
  
  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();


  // get inertia and mass of quadrotor body
  inertia = link->GetInertial()->PrincipalMoments();
  mass = link->GetInertial()->Mass();

  node_handle_ = new ros::NodeHandle;
  
  
  // subscribe command: control command
  if (!cmd_normal_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      cmd_normal_topic_, 1,
      boost::bind(&DroneSimpleController::CmdCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    cmd_subscriber_ = node_handle_->subscribe(ops);
    
    if( cmd_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using cmd_topic %s.", cmd_normal_topic_.c_str());
    else
        ROS_INFO("cannot find the command topic!");
  }
  
  if (!posctrl_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
      posctrl_topic_, 1,
      boost::bind(&DroneSimpleController::PosCtrlCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    posctrl_subscriber_ = node_handle_->subscribe(ops);
    
    if( posctrl_subscriber_.getTopic() != "")
        ROS_INFO("find the position control topic!");
    else
        ROS_INFO("cannot find the position control topic!");
  }
  

  // subscribe imu
  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&DroneSimpleController::ImuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);
    
    if(imu_subscriber_.getTopic() !="")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
    else
        ROS_INFO("cannot find the IMU topic!");
  }

  // subscribe command: take off command
  if (!takeoff_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      takeoff_topic_, 1,
      boost::bind(&DroneSimpleController::TakeoffCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    takeoff_subscriber_ = node_handle_->subscribe(ops);
    if( takeoff_subscriber_.getTopic() != "")
        ROS_INFO("find the takeoff topic");
    else
        ROS_INFO("cannot find the takeoff topic!");
  }

  // subscribe command: land command
  if (!land_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      land_topic_, 1,
      boost::bind(&DroneSimpleController::LandCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    land_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: reset command
  if (!reset_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      reset_topic_, 1,
      boost::bind(&DroneSimpleController::ResetCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    reset_subscriber_ = node_handle_->subscribe(ops);
  }
  
  if (!gt_topic_.empty()){
      pub_gt_pose_ = node_handle_->advertise<geometry_msgs::Pose>("drone/gt_pose",1024);    
  }
  
  pub_gt_vec_ = node_handle_->advertise<geometry_msgs::Twist>("drone/gt_vel", 1024);
  pub_gt_acc_ = node_handle_->advertise<geometry_msgs::Twist>("drone/gt_acc", 1024);
  
  
  if (!switch_mode_topic_.empty()){
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
        switch_mode_topic_, 1,
        boost::bind(&DroneSimpleController::SwitchModeCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
      switch_mode_subscriber_ = node_handle_->subscribe(ops);
  }
      

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////// copter ////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (!EulerModel_topic.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
      EulerModel_topic, 100,
      boost::bind(&DroneSimpleController::EulerModelCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    EulerModel_subscriber = node_handle_->subscribe(ops);
    if( EulerModel_subscriber.getTopic() != "")
        ROS_INFO("find the EulerModel topic");
    else
        ROS_INFO("cannot find the EulerModel topic!");
  }

 
  if (!NED_topic.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
      NED_topic, 100,
      boost::bind(&DroneSimpleController::NEDCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    NED_subscriber= node_handle_->subscribe(ops);
    
    if(NED_subscriber.getTopic() !="")
        ROS_INFO("find the NED topic");
    else
        ROS_INFO("cannot find the NEDtopic!");
  }


  if (!Vel_topic.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      Vel_topic, 100,
      boost::bind(&DroneSimpleController::VelCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    Vel_subscriber= node_handle_->subscribe(ops);
    
    if(Vel_subscriber.getTopic() !="")
        ROS_INFO("find the Vel topic");
    else
        ROS_INFO("cannot find the Vel topic!");
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

  LoadControllerSettings(_model, _sdf);
  
  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DroneSimpleController::Update, this));
}

void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");
    
    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");
    controllers_.alpha_x.Load(_sdf, "alphaX");
    controllers_.alpha_y.Load(_sdf, "alphaY");
    controllers_.W_z.Load(_sdf, "Wz");
    
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////copter//////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Funciones callback para actualizar los valores a recibir:
void DroneSimpleController::EulerModelCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  Euler_copter.x = message->x;
  Euler_copter.y = message->y;
  Euler_copter.z = message->z;
}

void DroneSimpleController::NEDCallback(const geometry_msgs::Vector3::ConstPtr & message)
{
  NED_copter.x = message->x;
  NED_copter.y = message->y;
  NED_copter.z = message->z;
}

void DroneSimpleController::VelCallback(const geometry_msgs::Twist::ConstPtr & message)
{
  Vel_copter = *message;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////




void DroneSimpleController::CmdCallback(const geometry_msgs::TwistConstPtr& cmd)
{
  cmd_val = *cmd;


  static common::Time last_sim_time = world->SimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  common::Time cur_sim_time = world->SimTime();
  double dt = (cur_sim_time - last_sim_time).Double();
  // save last time stamp
  last_sim_time = cur_sim_time;

  // generate noise
  if(time_counter_for_drift_noise > motion_drift_noise_time_)
  {
    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);

}

void DroneSimpleController::PosCtrlCallback(const std_msgs::BoolConstPtr& cmd){
    m_posCtrl = cmd->data;
}

void DroneSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  //directly read the quternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void DroneSimpleController::TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
{
  if(navi_state == LANDED_MODEL)
  {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    ROS_INFO("%s","\nCoaxial copter takes off!!");
  }
}

void DroneSimpleController::LandCallback(const std_msgs::EmptyConstPtr& msg)
{

}

void DroneSimpleController::ResetCallback(const std_msgs::EmptyConstPtr& msg)
{
  ROS_INFO("%s","\nReset Coaxial copter!!");
}

void DroneSimpleController::SwitchModeCallback(const std_msgs::BoolConstPtr& msg){
    m_velMode = msg->data;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void DroneSimpleController::Update()
{
    
    // Get new commands/state
    callback_queue_.callAvailable();
  
    // Get simulator time
    common::Time sim_time = world->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0) return;
    
    UpdateState(dt);
    UpdateDynamics(dt);
    
    // save last time stamp
    last_time = sim_time;   
}

void DroneSimpleController::UpdateState(double dt){
    if(navi_state == TAKINGOFF_MODEL){
        m_timeAfterCmd += dt;
        if (m_timeAfterCmd > 0.5){
            navi_state = FLYING_MODEL;
            std::cout << "Entering flying model!" << std::endl;
        }
    }
}


void DroneSimpleController::UpdateDynamics(double dt){
   
    // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  //  if (imu_subscriber_.getTopic()=="")
    {
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
	
    }
   // if (state_topic_.empty())
    {
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }
    
    
    //publish the ground truth pose of the drone to the ROS topic
    geometry_msgs::Pose gt_pose;
    gt_pose.position.x = pose.Pos().X();
    gt_pose.position.y = pose.Pos().Y();
    gt_pose.position.z = pose.Pos().Z();
    
    gt_pose.orientation.w = pose.Rot().W();
    gt_pose.orientation.x = pose.Rot().X();
    gt_pose.orientation.y = pose.Rot().Y();
    gt_pose.orientation.z = pose.Rot().Z();
    pub_gt_pose_.publish(gt_pose);
    
    //convert the acceleration and velocity into the body frame
    ignition::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
    ignition::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);
      euler = pose.Rot().RotateVector(euler);
    
    //publish the velocity
    geometry_msgs::Twist tw;
    tw.linear.x = body_vel.X();
    tw.linear.y = body_vel.Y();
    tw.linear.z = body_vel.Z();
    pub_gt_vec_.publish(tw);
    
    //publish the acceleration
    tw.linear.x = body_acc.X();
    tw.linear.y = body_acc.Y();
    tw.linear.z = body_acc.Z();
    pub_gt_acc_.publish(tw);
    

    ignition::math::Pose3d pose_pub;
    ignition::math::Vector3d vel_pub;
    ignition::math::Vector3d velang_pub;
    tf::Quaternion q;
	
     // process robot velocity information
    vel_pub.X() = Vel_copter.linear.x;
    vel_pub.Y() = Vel_copter.linear.y;
    vel_pub.Z() = -Vel_copter.linear.z;//el eje z en nuestro modelo está invertido con respecto al mundo de Gazebo
    velang_pub.X() = -Vel_copter.angular.x;//como consecuencia de lo anterior
    velang_pub.Y() = Vel_copter.angular.y;
    velang_pub.Z() = -Vel_copter.angular.z;

    if(navi_state == LANDED_MODEL)
    {
      q.setRPY(0,0,0);
    pose_pub.Pos().X() = 0.0;
    pose_pub.Pos().Y() = 0.0;
    pose_pub.Pos().Z() = 0.5;
    pose_pub.Rot().X() = q[0];
    pose_pub.Rot().Y() = q[1];
    pose_pub.Rot().Z() =  q[2];
    pose_pub.Rot().W() =  q[3];

	link -> SetWorldPose(pose_pub);
    }
    else if(navi_state == FLYING_MODEL)
    {
	link -> SetWorldTwist(vel_pub,velang_pub);
    }
    else if(navi_state == TAKINGOFF_MODEL)
    {
	link -> SetWorldTwist(vel_pub,velang_pub);
    }
}
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void DroneSimpleController::Reset()
{
    //copter
    ignition::math::Pose3d pose_pub;
    ignition::math::Vector3d vel_pub;
    ignition::math::Vector3d velang_pub;
    tf::Quaternion q;
      q.setRPY(0,0,0);
    pose_pub.Pos().X() = 0.0;
    pose_pub.Pos().Y() = 0.0;
    pose_pub.Pos().Z() = 0.1;
    pose_pub.Rot().X() = q[0];
    pose_pub.Rot().Y() = q[1];
    pose_pub.Rot().Z() =  q[2];
    pose_pub.Rot().W() =  q[3];

    vel_pub.X() = 0.0;
    vel_pub.Y() = 0.0;
    vel_pub.Z() = 0.0;
    velang_pub.X() = 0.0;
    velang_pub.Y() = 0.0;
    velang_pub.Z() = 0.0;

	link -> SetWorldTwist(vel_pub,velang_pub);
	link -> SetWorldPose(pose_pub);

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo
