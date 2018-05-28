/*
Screwdriver node


Simple interface for turning a screwdriver on/off, by the use of rosserial_arduino.


Copyright (c) 2018 Soren Bendtsen - Aalborg University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ros.h>    // Include the ROS library. This needs to be the very first dependency to include
#include <std_msgs/Int8.h>    // Include the Int8 library of the ROS type std_msgs

ros::NodeHandle  nh;    // The node handle is initiated. This allows the program to create subscriber, publishers, and handles serial communication

void messageCb( const std_msgs::Int8& msg){   // A callback function for the subscriber. The argument is of the type std_msgs::Int8.
  if(msg.data > 1.0)    // If an integer above 1 is published, pin 9 is set high
    analogWrite(9, 255);   // Turn on the screwdriver
  else
    analogWrite(9, 0);     // Turn off the screwdriver
}

ros::Subscriber<std_msgs::Int8> sub("screwdriver", &messageCb );    // The subscriber is initiated. It subscribes to the topic "screwdriver" - the 2nd argument is the callback function it should use

void setup()
{ 
  pinMode(9, OUTPUT);   // Sets pin 9 as output
  nh.initNode();    // Initialise the ROS node handle
  nh.subscribe(sub);    // Advertise the subscriber
}

void loop()
{  
  nh.spinOnce();    // All ROS communication callbacks are handled here
  delay(1);
}
