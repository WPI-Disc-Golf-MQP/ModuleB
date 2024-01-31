/*
 * rosserial Example Node: LED
 */

#define NODE_NAME String("led")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>

std_msgs::Bool feedback_msg;
String _feedback_topic(NODE_NAME + "_feedback");
ros::Publisher feedback_pub(_feedback_topic.c_str(), &feedback_msg);

void set_led(bool state)
{
  digitalWrite(LED_BUILTIN, state);
  feedback_msg.data = state;
  feedback_pub.publish(&feedback_msg);
}

void request_callback(int msg_data) {
  set_led(msg_data == REQUEST::REQUEST__STANDARD ? HIGH : LOW);
}

void setup() {
  init_std_node();
  nh.advertise(feedback_pub);
  set_request_callback(request_callback);

  pinMode(LED_BUILTIN, OUTPUT);
  
  loginfo("setup() Complete");
}

void loop() {
  periodic_status();
  nh.spinOnce();
  delay(10);
}