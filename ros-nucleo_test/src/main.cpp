#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>

 
// conveyor pins 
int BEAM_BREAK_PIN = D9; 
int SPEED_PIN = A0;
int INVERT_PIN = D13;
// 

// flex pins 

// scale pins 
// #define SCALE_RELAY__POWER_PIN D11
// #define SCALE_RELAY__TARE_PIN D12
// #define SCALE_SERIAL__RX_PIN D4
// #define SCALE_SERIAL__TX_PIN D1 //not used
// 

std_msgs::Float32 weight_msg;
String _weight_topic(NODE_NAME + "_feedback__weight");
ros::Publisher weight_feedback_pub(_weight_topic.c_str(), &weight_msg);

// -- global things 
long start_measure_time = millis();
bool measure_complete = false;




// -- conveyor functions 
std_msgs::Int8 conveyor_msg;
String _conveyor_topic(NODE_NAME + "_feedback__conveyor");
ros::Publisher conveyor_feedback_pub(_conveyor_topic.c_str(), &conveyor_msg);

long last_conveyor_center_time = millis();
enum CONVEYOR_STATE {
    CONVEYOR_IDLE = 0, 
    MOVING_TO_CENTER = 1, 
    MOVING_TO_NEXT_DISC = 2, 
    BACKUP = 3};

CONVEYOR_STATE conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE; 

void move_forward(int speed = 255) {
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move forward");
}

void move_backward(int speed = 255) {
  digitalWrite(INVERT_PIN, HIGH);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move backward");
}

void stop() {
  analogWrite(SPEED_PIN, 0); // stop
}

bool beam_broken() {
  return (digitalRead(BEAM_BREAK_PIN) == 0);
}

void start_conveyor() {
  loginfo("start_conveyor");
  conveyor_state = CONVEYOR_STATE::MOVING_TO_CENTER;
  move_forward();
}

void check_conveyor() {
  switch (conveyor_state){
    case CONVEYOR_STATE::MOVING_TO_CENTER:
      if (beam_broken()) {
        conveyor_state = CONVEYOR_STATE::MOVING_TO_NEXT_DISC;
        last_conveyor_center_time = millis();
      } 
      break;
    case CONVEYOR_STATE::MOVING_TO_NEXT_DISC:
      if (beam_broken() && last_conveyor_center_time+1000 < millis()) {
        move_backward();
        conveyor_state = CONVEYOR_STATE::BACKUP;
        last_conveyor_center_time = millis();
      } 
      break;
    case CONVEYOR_STATE::BACKUP:
      if (last_conveyor_center_time+1000 < millis()) {
        stop();
        conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
      }
      break;
    case CONVEYOR_STATE::CONVEYOR_IDLE:
      stop();
      set_status(NODE_STATUS::MOTION_COMPLETE);
      break;
    default:
      logwarn("Invalid conveyor state");
      break;
  }

  // publish state to pi 
  if (conveyor_msg.data != conveyor_state) {
    conveyor_msg.data = conveyor_state;
    conveyor_feedback_pub.publish(&conveyor_msg);
  }
}

bool verify_motion_complete() {
  return conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE;
}

// -- loop/setup functions 
void setup() {
  init_std_node();
  nh.advertise(weight_feedback_pub);
  nh.advertise(conveyor_feedback_pub);
  
  // conveyor pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;

  // scale pins
  // scaleSerial.begin(9600);
  // pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(SCALE_RELAY__POWER_PIN, OUTPUT);
  // pinMode(SCALE_RELAY__TARE_PIN, OUTPUT);

  loginfo("setup() Complete");
}


void loop() {
  periodic_status();
  nh.spinOnce();
  check_conveyor();
  delay(5);
  // check_measure();
  // parseIncomingData();
}
