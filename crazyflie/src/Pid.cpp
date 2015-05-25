#include "Pid.h"

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
Pid::Pid(ros::NodeHandle handler)/*{{{*/
{
  // Initiate ROS stuff
  server = handler.advertiseService("/updateTargetFD", &Pid::updateTarget, this);
  //sub = handler.subscribe("/stabilize", 100, &Pid::pid, this);
  pub = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  sub = handler.subscribe("/copter_center_stamped_3d", 100, &Pid::pid, this);

  // Initialize PID params
  // TODO:
  // i might NOT need to do this... bc the pid params are already initialized
  // set the pid constants

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

//----- Pid::pid ---------------------------------------------------------/*{{{*/
// @parameters: const pointer
// @brief: callback function for the subcriber
//         calculates adjustment values with PID and publishes new msg
//-----------------------------------------------------------------------------/*}}}*/
void Pid::pid(const geometry_msgs::PointStamped  &target)/*{{{*/
{
  // change it so that the target value is a target 3D point
  // and the input value for pid computation is the 3D point of the quadcopter

  // pitch : x
  // roll : z
  // thrust : y

  // get the current flight dynamic values
  //msg.linear.x = stabilizer.pitch;
  //msg.linear.y = stabilizer.roll;
  //msg.linear.z = stabilizer.thrust;
  //msg.angular.z = stabilizer.yaw;

  // calculate pid
  msg.linear.x += pids[PITCH].compute(target.point.x);
  msg.linear.y += pids[ROLL].compute(target.point.z);
  msg.linear.z += pids[THRUST].compute(target.point.y);
  //msg.angular.z += pids[YAW].compute(stabilizer.yaw);
 
  // TODO: check for max values and min values
  // publish Twist msg
  pub.publish(msg);
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

  ROS_INFO("PITCH %lf %lf %lf", pids[PITCH].kp, pids[PITCH].ki, pids[PITCH].kd);
  ROS_INFO("ROLL %lf %lf %lf", pids[ROLL].kp, pids[ROLL].ki, pids[ROLL].kd);
  ROS_INFO("YAW %lf %lf %lf", pids[YAW].kp, pids[YAW].ki, pids[YAW].kd);
}/*}}}*/

// ############################################################################
// Main Function
// ############################################################################
int main(int argc, char **argv)/*{{{*/
{
  ros::init(argc, argv, "pid");
  ros::NodeHandle handler;

  Pid pidNode(handler);

  dynamic_reconfigure::Server<crazyflie::SetPidParamsConfig> server;
  dynamic_reconfigure::Server<crazyflie::SetPidParamsConfig>::CallbackType cb;
  cb = boost::bind(&Pid::setParams, pidNode, _1, _2);
  server.setCallback(cb);

  ros::spin();
}/*}}}*/
