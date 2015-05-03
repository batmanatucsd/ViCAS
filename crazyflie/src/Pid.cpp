#include "Pid.h"

// ############################################################################
// PidParams Class Functions
// ############################################################################
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
  sub = handler.subscribe("/stabilize", 100, &Pid::pid, this);
  pub = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Initialize PID params
  // TODO:
  // set the pid constants

}  /*}}}*/

//----- Pid::updateTarget -----------------------------------------------------/*{{{*/
// @parameters: const ptr for new target
// @brief: callback function for the target service
//-----------------------------------------------------------------------------/*}}}*/
bool Pid::updateTarget(crazyflie::UpdateTargetFD::Request &newTarget,/*{{{*/
                       crazyflie::UpdateTargetFD::Response &res)
{
  pids[PITCH].target = newTarget.pitch; 
  pids[ROLL].target = newTarget.roll;
  pids[YAW].target = newTarget.yaw; 
  pids[THRUST].target = newTarget.thrust; 
  return true;
}/*}}}*/

//----- Pid::pid ---------------------------------------------------------/*{{{*/
// @parameters: const pointer
// @brief: callback function for the subcriber
//         calculates adjustment values with PID and publishes new msg
//-----------------------------------------------------------------------------/*}}}*/
void Pid::pid(const crazyflie::Stabilize &stabilizer)/*{{{*/
{
  // get the current flight dynamic values
  msg.linear.x = stabilizer.pitch;
  msg.linear.y = stabilizer.roll;
  msg.linear.z = stabilizer.thrust;
  msg.angular.z = stabilizer.yaw;

  // calculate pid
  msg.linear.x += pids[PITCH].compute(stabilizer.pitch);
  msg.linear.y += pids[ROLL].compute(stabilizer.roll);
  msg.linear.z += pids[THRUST].compute(stabilizer.thrust);
  msg.angular.z += pids[YAW].compute(stabilizer.yaw);
 
  // publish Twist msg
  pub.publish(msg);
}/*}}}*/

// ############################################################################
// Main Function
// ############################################################################
int main(int argc, char **argv)/*{{{*/
{
  ros::init(argc, argv, "pid");
  ros::NodeHandle handler;

  Pid pidNode(handler);
  ros::spin();
}/*}}}*/
