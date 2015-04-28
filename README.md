# ViCAS: Vision Controlled Autonomous System
####CSE 145 Class Project

Our goal is to have a ground vehicle track and pilot an aerial vehicle indoors and act as a single autonomous system. The system is being designed as an experimental platform for multi robot autonomous control.

Our system contains two vehicles, a ground robot, and an aerial robot.

Ground robot: Turtlebot v1 (https://www.willowgarage.com/turtlebot)

Aerial Vehicle: Crazyflie v1 (https://www.bitcraze.io/crazyflie/)

Being a lightwight indoor copter, the crazyflie does not have external sensors to navigate and fly autonomously. It is the turtlebot's job to navigate the copter well as itself. The turtlebot will first localize itself within the room, then track and localize the copter with respect to itself. This allows the turtlebot to know where the copter is within the wolrd frame (the room). With the 3D coordinate of the copter being known, control systems can be implimented to pilot the copter.
