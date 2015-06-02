#include "Pid.h"

double initial_thrust;

// ############################################################################
// PidParams Class Functions
// ############################################################################
//----- PidParams::setParams ----------------------------------------------------/*{{{*//*{{{*/
// @parameters: doubles for kp, ki, kd
//
// @brief: set PID constants
//-----------------------------------------------------------------------------/*}}}*//*}}}*/
void PidParams::setParams(double kp, double ki, double kd) /*{{{*/
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}/*}}}*/

//----- PidParams::compute ----------------------------------------------------/*{{{*/
// @parameters: 
//
// @brief: computes pid
//-----------------------------------------------------------------------------/*}}}*/
double PidParams::compute(double input)/*{{{*/
{
  // Compute time
  double now = ros::Time::now().toSec(); 
  double timeChange = (now - lastTime);

  // Compute errors
  double error = target - input;

  errorSum += error * timeChange;
  double dError = (error - lastError) / timeChange;

  // update the remembered variables
  lastTime = now;
  lastError = error;

  //ROS_INFO("error = %lf,   dError = %lf,   errorSum = %lf", error, dError, errorSum);
  ROS_INFO("error = %lf,   dError = %lf ", error, dError);
  ROS_INFO("result = %lf", kp*error + ki*errorSum + kd*dError);

  // return output
  return kp*error + ki*errorSum + kd*dError;
}/*}}}*/

// ############################################################################
// Pid Class Functions
// ############################################################################
//----- Pid::Pid --------------------------------------------------------------/*{{{*/
// @parameters: handler : ros NodeHandler, used to initiate member variables
//
// @brief: initialize ServiceServer, Publisher, Subscriber
//-----------------------------------------------------------------------------/*}}}*/
Pid::Pid(ros::NodeHandle handler) : velocity(0), last_time(ros::Time::now().toSec())/*{{{*/
{
  // Initiate ROS stuff
  server = handler.advertiseService("/updateTargetFD", &Pid::updateTarget, this);
  pub = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  sub_yaw = handler.subscribe("/stabilize", 100, &Pid::pid_yaw, this);
  sub = handler.subscribe("/copter_center_stamped_3d", 100, &Pid::pid, this);
  sub_imu = handler.subscribe("/imu", 10, &Pid::get_velocity, this);

  this->msg.linear.z = initial_thrust; // initial thrust
  //MAX_THRUST = initial_thrust;

  // Initialize PID params, set the pid constants
  // PITCH 
  pids[PITCH].target = 0.0; 
  //pids[PITCH].setParams(7.5, 0, 0);
  pids[PITCH].setParams(4.95, 0, 0);

  // ROLL
  pids[ROLL].target = 2.5;
  //pids[ROLL].setParams(3.5, 0, 0);
  pids[ROLL].setParams(1.95, 0, 0);

  // THRUST
  pids[THRUST].target = -0.2; 
  //pids[THRUST].setParams(436, 0, 0);
  pids[THRUST].setParams(291, 0, 0);

  // YAW
  pids[YAW].target = 0;
  pids[YAW].setParams(1.95, 0, 0);
}  /*}}}*/

//----- Pid::updateTarget -----------------------------------------------------/*{{{*/
// @parameters: const ptr for new target
// @brief: callback function for the target service
//-----------------------------------------------------------------------------/*}}}*/
bool Pid::updateTarget(crazyflie::UpdateTargetFD::Request &newTarget,/*{{{*/
                       crazyflie::UpdateTargetFD::Response &res)
{
  // change the UpdateTargetFD to use PointStamped
  // and get x, y, z for targets
  // pitch : x
  // roll : z
  // thrust : y

  //pids[PITCH].target = newTarget.pitch; 
  //pids[ROLL].target = newTarget.roll;
  //pids[YAW].target = newTarget.yaw; 
  //pids[THRUST].target = newTarget.thrust; 
  pids[PITCH].target = newTarget.x; 
  pids[ROLL].target = newTarget.z;
  pids[THRUST].target = newTarget.y; 
  //pids[YAW].target = newTarget.yaw; 
  return true;
}/*}}}*/

//----- Pid::get_velocity ---------------------------------------------------------/*{{{*/
// @parameters: const pointer to sensor_msgs::Imu
// @brief: callback function for the subcriber
//         gets velocity from the imu data
//-----------------------------------------------------------------------------/*}}}*/
void Pid::get_velocity(const sensor_msgs::Imu &imu)
{
  // Compute time
  double now = ros::Time::now().toSec(); 
  double timeChange = (now - this->last_time);
  this->last_time = now;

  // calculate velocity
  double acc = imu.linear_acceleration.z - 10;
  if(acc > 0.5 || acc < -0.5) {
    if((this->velocity > 0.4 && acc < 0) || (this->velocity < -0.4 && acc > 0))
      this->velocity = 0;
    else
      this->velocity += acc * timeChange;
  }
}

//----- Pid::pid_yaw ---------------------------------------------------------/*{{{*/
// @parameters: const pointer to crazyflie::Stabilize
// @brief: callback function for the subcriber
//         calculates pid for yaw
//-----------------------------------------------------------------------------/*}}}*/
void Pid::pid_yaw(const crazyflie::Stabilize &stabilizer)
{
  ROS_INFO("######## YAW #######");
  this->msg.angular.z = -pids[YAW].compute(stabilizer.yaw);

  // Max Values
  this->msg.angular.z = (this->msg.angular.z > MAX_YAW) ? MAX_YAW : this->msg.angular.z;

  // Min Values
  this->msg.angular.z = (this->msg.angular.z < MIN_YAW) ? MIN_YAW : this->msg.angular.z;
}

//----- Pid::pid ---------------------------------------------------------/*{{{*/
// @parameters: const pointer
// @brief: callback function for the subcriber
//         calculates adjustment values with PID and publishes new msg
//-----------------------------------------------------------------------------/*}}}*/
void Pid::pid(const geometry_msgs::PointStamped  &input)/*{{{*/
{
  // change it so that the input value is a input 3D point
  // and the input value for pid computation is the 3D point of the quadcopter

  // from 3D point  
  // pitch : x
  // roll : z
  // thrust : y

  // get the current flight dynamic values
  //msg.linear.x = stabilizer.pitch;
  //msg.linear.y = stabilizer.roll;
  //msg.linear.z = stabilizer.thrust;
  //msg.angular.z = stabilizer.yaw;
  ROS_INFO("######## 3D POINT #######");
  ROS_INFO("x: %lf    y: %lf    z: %lf",  input.point.x, input.point.y, input.point.z);

  // calculate pid
  ROS_INFO("######## pitch #######");
  this->msg.linear.x = pids[PITCH].compute(input.point.x);
  ROS_INFO("######## roll #######");
  this->msg.linear.y = -pids[ROLL].compute(input.point.z);
  ROS_INFO("######## thrust #######");
  double pidThrustResult = pids[THRUST].compute(input.point.y);

  ROS_INFO("******************* Velocity *****************");
  ROS_INFO("Velocity %lf", this->velocity);

  if(this->velocity > 0.1) { // when quadcopter is moving up
    if(pidThrustResult < 0) // motion and correction in the same direction
      this->msg.linear.z -= 0.2 * pidThrustResult;
    else // motion and correction in the opposite direction
      this->msg.linear.z -= 1.025 * pidThrustResult;

  } else if(this->velocity < -0.1) { // when quadcopter is movign down
    if(pidThrustResult < 0) // motion and correction in the opposite direction
      this->msg.linear.z -= 3.5 * pidThrustResult;
    else // motion and correction in the same direction
      this->msg.linear.z -= 0.05 * pidThrustResult;

  } else { // when velocity is considered too small
    if((input.point.y > pids[THRUST].target && pidThrustResult > 0) ||
       (input.point.y < pids[THRUST].target && pidThrustResult < 0)) // when changin directions
      this->msg.linear.z -= pidThrustResult;
    else
      this->msg.linear.z -= 0.15 * pidThrustResult;
  }

 
  // Max Values
  this->msg.linear.x = (this->msg.linear.x > MAX_PITCH) ? MAX_PITCH : this->msg.linear.x;
  this->msg.linear.y = (this->msg.linear.y > MAX_ROLL) ? MAX_ROLL : this->msg.linear.y;
  //this->msg.linear.z = (this->msg.linear.z > MAX_THRUST) ? MAX_THRUST : this->msg.linear.z;
  // max value for the thrust is set in the python api

  // Min Values
  this->msg.linear.x = (this->msg.linear.x < MIN_PITCH) ? MIN_PITCH : this->msg.linear.x;
  this->msg.linear.y = (this->msg.linear.y < MIN_ROLL) ? MIN_ROLL : this->msg.linear.y;
  this->msg.linear.z = (this->msg.linear.z < MIN_THRUST) ? MIN_THRUST : this->msg.linear.z;

  ROS_INFO("pitch: %lf    roll: %lf    thrust: %lf",  this->msg.linear.x, this->msg.linear.y, this->msg.linear.z);
}/*}}}*/

//----- Pid::setParams ---------------------------------------------------------/*{{{*/
// @parameters: dynamic reconfigure config, level 
// @brief: callback function for dynamic reconfigure
//         sets new PID constants dynamically
//-----------------------------------------------------------------------------/*}}}*/
void Pid::setParams(crazyflie::SetPidParamsConfig &config, uint32_t level) /*{{{*/
{
  // update the pid constants
  // call setParams functions of the PidParams class

  // PITCH 
  pids[PITCH].setParams(config.pitch_kp, config.pitch_ki, config.pitch_kd);

  // ROLL
  pids[ROLL].setParams(config.roll_kp, config.roll_ki, config.roll_kd);

  // YAW
  pids[YAW].setParams(config.yaw_kp, config.yaw_ki, config.yaw_kd);

  // THRUST
  // pids[THRUST].setParams(config.thrust_kp, config.thrust_ki, config.thrust_kd);

  initial_thrust = config.initial_thrust;

  ROS_INFO("PITCH %lf %lf %lf", pids[PITCH].kp, pids[PITCH].ki, pids[PITCH].kd);
  ROS_INFO("ROLL %lf %lf %lf", pids[ROLL].kp, pids[ROLL].ki, pids[ROLL].kd);
  ROS_INFO("YAW %lf %lf %lf", pids[YAW].kp, pids[YAW].ki, pids[YAW].kd);
}/*}}}*/

void Pid::publish() 
{
  // publish the flight dynamics
  pub.publish(this->msg);
}

// ############################################################################
// Main Function
// ############################################################################
int main(int argc, char **argv)/*{{{*/
{
  ros::init(argc, argv, "pid");
  ros::NodeHandle handler;
  ros::NodeHandle nh("~");
  ros::Rate rate(10);

  nh.param<double>("initial_thrust", initial_thrust, 37000);
  //nh.getParam("initial_thrust", initial_thrust);

  Pid pidNode(handler);

  dynamic_reconfigure::Server<crazyflie::SetPidParamsConfig> server;
  dynamic_reconfigure::Server<crazyflie::SetPidParamsConfig>::CallbackType cb;
  cb = boost::bind(&Pid::setParams, pidNode, _1, _2);
  server.setCallback(cb);

  while(ros::ok()) {
    pidNode.publish();
    ros::spinOnce();
    rate.sleep();
  }
   
}/*}}}*/
