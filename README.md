# Line follower using PID
This project involves design and implementation of a PID controller for the Spark V robot to make it follow continuous lines (Black lines on White paper), with sharp turns, loops, 90 degree cross paths.

## Line Following Bot:
A line following robot follows a path made of black lines on white surface or vice versa. Its objective is to sense the line and stay on the line while constantly correcting its position by means of a feedback mechanism. A feedback system makes it a closed loop system.

## Hardware Specifications:

The bot used here is a Spark V robot.  It is jointly designed by NEX Robotics with Department of Computer Science and Engineering, IIT Bombay. Spark V robot is based on ATMEGA16A microcontroller. Robot comes with rechargeable 7.2V 600mA NiMH Battery and onboard intelligent battery charger. 
It has 
* 3 analog white line sensors
* 3 analog IR Proximity sensors
* 3 directional light intensity sensors
* battery voltage sensing
* TSOP1738 IR receiver for TV remote control and Position encoders
* has support for 3 MaxBotix EZ series ultrasonic range sensors
* has support for the servo mounted sensor pod which can be used to make 180 degrees scan for the map making

Robot is powered by 6 cell 7.2V 600mA rechargeable NiMH batteries which gives about one hour battery operation. Robot has built-in Smart Battery Controller which charges the battery in intelligent way and also monitors the battery charge level when robot is in operation. Robot has 2x16 alphanumeric LCD, Lots of LED indicators for quick debugging, Buzzer etc. Motors are controlled by L293D motor driver. Robot gives top speed of 15cm to 20cm per second depending on the model.
