# Introduction #

This page describes, how to create an Arduino based robot and control it wirelessly using ROS (Robot Operating System).

I am using "Arduino Fio" board on the Robot. Advantage of Fio board is that it comes with easy XBee integration.


# Details #



Arduino Fio Setup:
Xbee Setup:

The XBee communication is not set up to do programming, and I program Arduino Fio using serial programmer from Sparkfun FTDI Basic.

http://www.sparkfun.com/products/9873

The Xbee is setup using simple configuration using X-CTU.

Parameter     Xbee on Computer     Xbee on Robot

BD            6 (57600)            6 (57600)
ID            3000                 3000
MY            3001                 3002
DL            3002                 3001

MOTOR CONTROL:

The robot have simple DC motors, and hence the speed is controlled using Sparkfun 1A Dual Motor Driver TB6612FNG

http://www.sparkfun.com/products/9457

Pin Connections for Dual Motor driver and Arduino Fio:





To get commands from ROS over XBEE module, a simple Arduino Subscriber code is written, also see (ROS Arduino) for more details: