#include "ros/ros.h"
#include <termios.h>
#include "crazyflie/Commands.h"

int getch()
{
  static struct termios oldt, newt; 
  tcgetattr( STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON); // disable bufferings
  tcsetattr( STDIN_FILENO, TCSANOW, &newt); // apply new settings 

  int c = getchar(); // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // resrtore old settingsc
  return c;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_commands");
  ros::NodeHandle handler;

  ros::Publisher pub = handler.advertise<crazyflie::Commands>("/commands", 100);
  crazyflie::Commands update_command;

  while(ros::ok()) {
    //pub.publish(update_command);
    int c = getch();

    update_command.command = c;
    
    ROS_INFO("Sending %c", c);
    pub.publish(update_command);
  }

  return 0;
}
