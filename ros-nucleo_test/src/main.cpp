#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <HardwareSerial.h> //for scale

// ----- MAIN CONVEYOR -----

int BEAM_BREAK_PIN = D2; 
int SPEED_PIN = A0;
int INVERT_PIN = D13;

// -- conveyor feedback 
std_msgs::Int8 conveyor_msg;
String _conveyor_topic(NODE_NAME + "_feedback__conveyor");
ros::Publisher conveyor_feedback_pub(_conveyor_topic.c_str(), &conveyor_msg);

enum CONVEYOR_STATE {
    CONVEYOR_IDLE = 0, 
    MOVING_TO_CENTER = 1, 
    MOVING_TO_NEXT_DISC = 2, 
    BACKUP = 3};

CONVEYOR_STATE conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE; 
long last_conveyor_center_time = millis();

void move_forward(int speed = 230) {
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move forward");
}

void move_backward(int speed = 230) {
  digitalWrite(INVERT_PIN, HIGH);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move backward");
}

void stop() {
  analogWrite(SPEED_PIN, 0); // stop
  if (conveyor_state != CONVEYOR_STATE::CONVEYOR_IDLE) {
    loginfo("stop");
    conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
  }
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
      if (last_conveyor_center_time+500 < millis()) {
        stop();
        conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
        send_status(NODE_STATUS::MOTION_COMPLETE);
        status.data = NODE_STATUS::IDLE;
      }
      break;
    case CONVEYOR_STATE::CONVEYOR_IDLE:
      stop();
      break;
    default:
      logwarn("Invalid conveyor state");
      break;
  }

  // publish state to pi 
  if (conveyor_msg.data != (int) conveyor_state) {
      conveyor_msg.data = conveyor_state;
      loginfo("publishing conveyor state");
      conveyor_feedback_pub.publish(&conveyor_msg);
  }
}

bool verify_motion_complete() {
  return conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE;
}

// ----- SCALE -----

#define SCALE_SERIAL__RX_PIN D4
#define SCALE_SERIAL__TX_PIN D5 //not used

#define SCALE_RELAY__POWER_PIN D11 // D6
#define SCALE_RELAY__TARE_PIN D12 // D7

HardwareSerial scaleSerial(SCALE_SERIAL__RX_PIN, SCALE_SERIAL__TX_PIN); // RX, TX
const byte numChars = 16;
float lastWeight = 0.0;
char receivedChars[numChars];

std_msgs::Float32 weight_msg;
String _weight_topic(NODE_NAME + "_feedback__weight");
ros::Publisher weight_feedback_pub(_weight_topic.c_str(), &weight_msg);

enum SCALE_STATE {
    SCALE_IDLE = 0,
    MEASURING = 1, 
    TARING = 2,
    POWERING_ON = 3,
    POWERING_OFF = 4};
SCALE_STATE scale_state = SCALE_STATE::SCALE_IDLE;
unsigned long last_scale_data_time = millis();
unsigned long start_scale_action_time = millis();


void toggleScalePower() {
    digitalWrite(SCALE_RELAY__POWER_PIN, HIGH);
    delay(2000);
    digitalWrite(SCALE_RELAY__POWER_PIN, LOW);
}

void toggleScaleTare() {
    digitalWrite(SCALE_RELAY__TARE_PIN, HIGH);
    delay(2000);
    digitalWrite(SCALE_RELAY__TARE_PIN, LOW);
}

void start_measure() {
  loginfo("start_measure");
  start_scale_action_time = millis();
  scale_state = SCALE_STATE::MEASURING;
}

void check_measure() {
  switch (scale_state) {
  case SCALE_STATE::MEASURING:
    if (start_scale_action_time+1500 < millis()) { //measurement complete
      weight_msg.data = lastWeight;
      send_status(NODE_STATUS::MEASURE_COMPLETE);
      scale_state = SCALE_STATE::SCALE_IDLE;
    }
    break;
  case SCALE_STATE::TARING:
    if (start_scale_action_time+2000 < millis()) { //button press complete
      digitalWrite(SCALE_RELAY__TARE_PIN, LOW);
      scale_state = SCALE_STATE::SCALE_IDLE;
    }
    break;
  case SCALE_STATE::POWERING_ON:
    if (start_scale_action_time+2000 < millis()) { //button press complete
      digitalWrite(SCALE_RELAY__POWER_PIN, LOW);
      scale_state = SCALE_STATE::SCALE_IDLE;
    }
    break;
  case SCALE_STATE::POWERING_OFF:
    if (start_scale_action_time+2000 < millis()) { //button press complete
      digitalWrite(SCALE_RELAY__POWER_PIN, LOW);
      scale_state = SCALE_STATE::SCALE_IDLE;
    }
    break;
  case SCALE_STATE::SCALE_IDLE:
    break;
  default:
    break;
  }

  if (last_scale_data_time+1000 < millis()) { //If we haven't heard from the scale, turn it on!
    scale_state = SCALE_STATE::POWERING_ON;
  }
}

bool verify_measure_complete() {
  return scale_state == SCALE_STATE::SCALE_IDLE;
}

void parseIncomingData() {
    static bool recvInProgress = false;
    static byte ndx = 0;
    char rc;
    char startMarker = '+';
    char endMarker = '\n'; 
 
    while (scaleSerial.available() > 0) {
        rc = scaleSerial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) ndx = numChars - 1;
            } else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                if (atoff(receivedChars) != lastWeight) {
                    lastWeight = atoff(receivedChars);
                    if (scale_state == SCALE_STATE::MEASURING) {
                      weight_msg.data = lastWeight;
                      weight_feedback_pub.publish(&weight_msg);
                    }
                }
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// ----- loop/setup functions -----
void setup() {
  init_std_node();
  set_request_callbacks(
    start_measure, 
    verify_measure_complete, 
    start_conveyor, 
    verify_motion_complete, 
    stop);
  
  //Register ROS publishers
  nh.advertise(weight_feedback_pub);
  nh.advertise(conveyor_feedback_pub);
  
  // conveyor pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;

  // scale pins
  scaleSerial.begin(9600);
  pinMode(SCALE_RELAY__POWER_PIN, OUTPUT);
  pinMode(SCALE_RELAY__TARE_PIN, OUTPUT);

  loginfo("setup() Complete");
}


void loop() {
  periodic_status();
  nh.spinOnce();
  check_conveyor();
  parseIncomingData();
  check_measure();
}
