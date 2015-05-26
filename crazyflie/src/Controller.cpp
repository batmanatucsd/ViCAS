#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/Twist.h"
#include "crazyflie/Stabilize.h"
#include "crazyflie/Commands.h"
#include "crazyflie/SetControlsConfig.h"

geometry_msgs::Twist msg;
geometry_msgs::Twist zero;
char command;
int inc;

class Controller 
{
  
};

void callback(const crazyflie::Stabilize &data)/*{{{*/
{
  msg.linear.x = data.pitch;
  msg.linear.y = data.roll;
  msg.angular.z = data.yaw;
}/*}}}*/

void get_commands(const crazyflie::Commands &data)/*{{{*/
{
  command = data.command; 
}/*}}}*/

void setControls(crazyflie::SetControlsConfig &config, uint32_t level)/*{{{*/
{
  msg.linear.z = config.thrust;
  inc = config.inc;
  
}/*}}}*/

int main(int argc, char **argv) /*{{{*/
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle handler;

  dynamic_reconfigure::Server<crazyflie::SetControlsConfig> server;
  dynamic_reconfigure::Server<crazyflie::SetControlsConfig>::CallbackType cb;
  cb = boost::bind(setControls, _1, _2);
  server.setCallback(cb);

  ros::Publisher pub = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber sub = handler.subscribe("/stabilize", 100, callback);
  ros::Subscriber commands = handler.subscribe("/commands", 10, get_commands);

  ros::Rate rate(10);

  ros::Duration(2.0).sleep();

  while(ros::ok()) {
    switch(command) {
      case 'a':
        msg.linear.z = 38000;
        pub.publish(msg);
        break;

      case 'b':
        msg.linear.z = 35000;
        pub.publish(msg);
        break;

      case 'u':
        msg.linear.z += inc;
        pub.publish(msg);
        command = 'k';
        break;

      case 'd':
        msg.linear.z -= inc;
        pub.publish(msg);
        command = 'k';
        break;

      case 'c':
        pub.publish(zero);
        break;

      case 'k':
        //pub.publish(zero);
        break;

      default:
        break;
    } // @end of switch

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}/*}}}*/
