#ifndef PID_H
#define PID_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dynamic_reconfigure/server.h"
#include "crazyflie/Stabilize.h"
#include "crazyflie/UpdateTargetFD.h"
#include "crazyflie/SetPidParamsConfig.h"

enum FD {PITCH=0, ROLL, YAW, THRUST};

// ############################################################################
// Pid Struct
// ############################################################################
class PidParams/*{{{*/
{
  public:
    // Member variables
    double lastTime;
    double kp, ki, kd,
           errorSum, lastError,
           target;

    // Class Functions
    PidParams() : kp(1), ki(0), kd(0), errorSum(0), lastError(0), lastTime(0) {}
    void setParams(double, double, double);
    double compute(double);
};/*}}}*/

// ############################################################################
// Pid Node Class
// ############################################################################
class Pid/*{{{*/
{
    // ROS stuff
    ros::ServiceServer server;
    ros::Subscriber sub;
    ros::Publisher pub;

    // PID 
    geometry_msgs::Twist msg;
    PidParams pids[4];

    // Functions
    bool updateTarget(crazyflie::UpdateTargetFD::Request &,
                      crazyflie::UpdateTargetFD::Response &);
    void pid(const crazyflie::Stabilize &);

  public:
    Pid(ros::NodeHandle);
    void setParams(crazyflie::SetPidParamsConfig &, uint32_t);
};/*}}}*/

#endif // PID_H
