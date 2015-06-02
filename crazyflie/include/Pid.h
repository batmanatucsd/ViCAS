#ifndef PID_H
#define PID_H

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "crazyflie/Stabilize.h"
#include "crazyflie/UpdateTargetFD.h"
#include "crazyflie/SetPidParamsConfig.h"

#define MAX_PITCH 5
#define MAX_ROLL 5
#define MAX_YAW 50

#define MIN_PITCH -5
#define MIN_ROLL -5
#define MIN_YAW -50
#define MIN_THRUST 30000

//double MAX_THRUST = 60000;

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
    ros::Subscriber sub_yaw;
    ros::Subscriber sub_imu;
    ros::Publisher pub;

    // Member variables 
    PidParams pids[4];
    geometry_msgs::Twist msg;
    //geometry_msgs::Twist velocity;
    double velocity;
    double last_time;


    // Functions
    bool updateTarget(crazyflie::UpdateTargetFD::Request &,
                      crazyflie::UpdateTargetFD::Response &);
    void pid(const geometry_msgs::PointStamped &);
    void pid_yaw(const crazyflie::Stabilize &);
    void get_velocity(const sensor_msgs::Imu &);

  public:
    Pid(ros::NodeHandle);
    void setParams(crazyflie::SetPidParamsConfig &, uint32_t);
    void publish();
};/*}}}*/

#endif // PID_H
