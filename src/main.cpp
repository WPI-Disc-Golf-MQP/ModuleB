#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <HardwareSerial.h> //for scale

// ----- MAIN CONVEYOR -----

MODULE* main_conveyor_module;
int EDGE_BEAM_BREAK_PIN = D2; 
int CENTER_BEAM_BREAK_PIN = D3; //TODO: VERIFY PIN NUMBER
int SPEED_PIN = A0;
int INVERT_PIN = D13;

enum CONVEYOR_STATE {
    CONVEYOR_IDLE = 0, 
    ADVANCING_TO_NEXT_DISC_EDGE = 1,
    WAITING_FOR_INTAKE = 2,
    MOVING_TO_CENTER = 3, 
    BACKUP = 4};
CONVEYOR_STATE conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE; 

long last_conveyor_center_time = millis();

void move_forward(int speed = 230) {
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("conveyor moving forward");
}

void move_backward(int speed = 230) {
  digitalWrite(INVERT_PIN, HIGH);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("conveyor moving backward");
}

bool edge_beam_broken() {
  return (digitalRead(EDGE_BEAM_BREAK_PIN) == 0);
}

bool center_beam_broken() {
  return (digitalRead(CENTER_BEAM_BREAK_PIN) == 0);
}

void start_conveyor() {
  if (conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE) {
    loginfo("start_conveyor in IDLE --> moving to center");
    conveyor_state = CONVEYOR_STATE::ADVANCING_TO_NEXT_DISC_EDGE;
    move_forward();
  } else if (conveyor_state == CONVEYOR_STATE::WAITING_FOR_INTAKE) {
    loginfo("start_conveyor in WAITING FOR INTAKE --> advancing to next disc edge");
    conveyor_state = CONVEYOR_STATE::MOVING_TO_CENTER;
    move_forward();
  } else {
    logwarn("start_conveyor called in invalid state, " + String((int)conveyor_state));
  
  }
  
}

void stop_conveyor() {
  analogWrite(SPEED_PIN, 0); // stop
  if (conveyor_state != CONVEYOR_STATE::CONVEYOR_IDLE) {
    loginfo("stop");
    conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
  }
}

void calibrate_conveyor() {
  loginfo("calibrate conveyor; TODO"); //TODO: Implement calibration
}

void check_conveyor() {
  switch (conveyor_state){
    case CONVEYOR_STATE::ADVANCING_TO_NEXT_DISC_EDGE:
      if (edge_beam_broken()) {
        stop_conveyor();
        conveyor_state = CONVEYOR_STATE::WAITING_FOR_INTAKE;
      }
      break;
    case CONVEYOR_STATE::WAITING_FOR_INTAKE:
      // next state is triggered by signal from intake module via Pi
      break;
    case CONVEYOR_STATE::MOVING_TO_CENTER:
      if (center_beam_broken()) {
        conveyor_state = CONVEYOR_STATE::BACKUP;
        move_backward();
      }
      break;
    case CONVEYOR_STATE::BACKUP:
      if (edge_beam_broken()) {
        stop_conveyor();
        conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
        main_conveyor_module->publish_status(MODULE_STATUS::COMPLETE);
      }
      break;
    case CONVEYOR_STATE::CONVEYOR_IDLE:
      stop_conveyor();
      break;
    default:
      logwarn("Invalid conveyor state");
      break;
  }
  main_conveyor_module->publish_state((int) conveyor_state);
}

bool verify_conveyor_complete() {
  return conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE;
}

// ----- SCALE -----

MODULE* scale_module;
#define SCALE_SERIAL__RX_PIN D4
#define SCALE_SERIAL__TX_PIN D5 //not used

#define SCALE_RELAY__POWER_PIN D11 // D6
#define SCALE_RELAY__TARE_PIN D12 // D7

HardwareSerial scaleSerial(SCALE_SERIAL__RX_PIN, SCALE_SERIAL__TX_PIN); // RX, TX
const byte numChars = 16;
float lastWeight = 0.0;
char receivedChars[numChars];

std_msgs::Float32 weight_msg;
String _weight_topic("scale_feedback__weight");
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

void start_scale() {
  loginfo("start scale");
  start_scale_action_time = millis();
  scale_state = SCALE_STATE::MEASURING;
}

void stop_scale() {
  loginfo("stop scale");
  scale_state = SCALE_STATE::SCALE_IDLE;
}

void calibrate_scale() {
  loginfo("calibrate scale; TODO"); //TODO: Implement calibration
}

void check_scale() {
  switch (scale_state) {
  case SCALE_STATE::MEASURING:
    if (start_scale_action_time+1500 < millis()) { //measurement complete
      weight_msg.data = lastWeight;
      scale_module->publish_status(MODULE_STATUS::COMPLETE);
      scale_state = SCALE_STATE::SCALE_IDLE;
      loginfo("scale measurement complete");
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
  scale_module->publish_state((int) scale_state);

  //TODO: Implement scale power on
  // if (last_scale_data_time+1000 < millis()) { //If we haven't heard from the scale, turn it on!
  //   scale_state = SCALE_STATE::POWERING_ON;
  // }
}

bool verify_scale_complete() {
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
                    //if (scale_state == SCALE_STATE::MEASURING) {
                      weight_msg.data = lastWeight;
                      weight_feedback_pub.publish(&weight_msg);
                    //}
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
  scale_module = init_module("scale",
    start_scale, 
    verify_scale_complete, 
    stop_scale,
    calibrate_scale);
  main_conveyor_module = init_module("main_conveyor",
    start_conveyor, 
    verify_conveyor_complete, 
    stop_conveyor,
    calibrate_conveyor);
  
  //Register ROS publishers
  nh.advertise(weight_feedback_pub);
  
  // conveyor pins 
  pinMode(EDGE_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(CENTER_BEAM_BREAK_PIN, INPUT_PULLUP) ;
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
  parseIncomingData();
  check_scale();
  check_conveyor();

  // ----- testing ----- 
  // if (verify_motion_complete()) {
  //   loginfo("debugging test reset");
  //   delay(5000);
  //   start_conveyor();
  // }
}
