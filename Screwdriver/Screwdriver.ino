/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Int8& msg){
  if(msg.data > 1.0)
    analogWrite(9, 255);   // Turn on the screwdriver
  else
    analogWrite(9, 0);     // Turn off the screwdriver
}

ros::Subscriber<std_msgs::Int8> sub("screwdriver", &messageCb );

void setup()
{ 
  pinMode(9, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

