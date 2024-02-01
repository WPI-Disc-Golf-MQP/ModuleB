/*
 * rosserial Example Node: LED
 */



#define NODE_NAME String("led")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <Arduino.h>
 
// define pins 
int BEAM_BREAK_PIN = D9; 
int SPEED_PIN = A7;
int INVERT_PIN = D13;
// 


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
  set_led(msg_data == REQUEST::START_MOTION ? HIGH : LOW);
}







// conveyor functions 
void move_forward(int speed) {
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
  Serial.println("move forward");
}

void move_backward(int speed) {
  digitalWrite(INVERT_PIN, HIGH);
  analogWrite(SPEED_PIN, speed); // start
  Serial.println("move backward");
}

void stop() {
  analogWrite(SPEED_PIN, 0); // stop
}

void move_one_section(){
  move_forward(255);
  delay(1000); // so that it moves off of the beam breaker
  while (true) {
    delay(100); 
    Serial.println("moving...");
    if (digitalRead(BEAM_BREAK_PIN) == 0) {
      // it is broken, so stop the conveyor
      stop(); 
      break; 
    } 
  }
}



// loop setup functions 
void setup() {
  init_std_node();
  nh.advertise(feedback_pub);
  set_request_callback(request_callback);

  pinMode(LED_BUILTIN, OUTPUT);
  
  // conveyor pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;


  loginfo("setup() Complete");
}

void loop() {
  move_forward(255); 
  delay(3000);
  stop();
  delay(500);


  // periodic_status();
  // nh.spinOnce();
  // delay(10);
}