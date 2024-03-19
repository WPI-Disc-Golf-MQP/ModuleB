#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <HardwareSerial.h> //for scale

// ----- MAIN CONVEYOR -----

MODULE* main_conveyor_module;
int BACKUP_BEAM_BREAK_PIN = D2; // Verified this pin as the black beam break 
int CENTER_BEAM_BREAK_PIN = D3; //Verified this pin as the green beam break
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

bool backup_beam_broken() {
  return (digitalRead(BACKUP_BEAM_BREAK_PIN) == 0);
}

bool center_beam_broken() {
  return (digitalRead(CENTER_BEAM_BREAK_PIN) == 0);
}

unsigned long started_advancing_time = millis();
void start_conveyor() {
  if (conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE) {
    loginfo("start_conveyor in IDLE --> advancing disc");
    conveyor_state = CONVEYOR_STATE::ADVANCING_TO_NEXT_DISC_EDGE;
    started_advancing_time = millis();
    move_forward();
  } else if (conveyor_state == CONVEYOR_STATE::WAITING_FOR_INTAKE) {
    loginfo("start_conveyor in WAITING FOR INTAKE --> centering discs");
    conveyor_state = CONVEYOR_STATE::MOVING_TO_CENTER;
    move_forward();
  } else {
    logwarn("start_conveyor called in invalid state, " + String((int)conveyor_state));
  }  
}

void stop_conveyor() {
  analogWrite(SPEED_PIN, 0); // stop
}

void calibrate_conveyor() {
  loginfo("calibrate conveyor; TODO"); //TODO: Implement calibration
}

void check_conveyor() {
  switch (conveyor_state){
    case CONVEYOR_STATE::ADVANCING_TO_NEXT_DISC_EDGE:
      if (backup_beam_broken() && started_advancing_time+1000 < millis()) { // go forward at least one second, then beam broken again
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
      if (backup_beam_broken()) {
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

// ----- FLEX ----- 

MODULE* flex_module;
int dir_pin = D9; 
int step_pin = D10; 
// int sleep_pin = D6; // Verify this pin  // FIX LAST ONE
int UPPER_LIMIT_SWITCH_PIN = A7; 
int LOWER_LIMIT_SWITCH_PIN = A6; 

enum FLEX_STATE {
  FLEX_IDLE = 0,
  FLEX_RAISING = 1,
  FLEX_LOWERING = 2
};
FLEX_STATE flex_state = FLEX_STATE::FLEX_IDLE;


bool upper_limit_switched() { // FINISH THIS FUNCTION after wiring
  return (digitalRead(UPPER_LIMIT_SWITCH_PIN) == 1);
}

bool lower_limit_switched() { // FINISH THIS FUNCTION after wiring
  return (digitalRead(LOWER_LIMIT_SWITCH_PIN) == 1);
}

bool verify_flex_complete() {
  return flex_state == FLEX_STATE::FLEX_IDLE;
}

bool run_yaxis_motor = false;
long yaxis_motor_last_step = millis();
bool yaxis_motor_last_digital_write = false;

bool run_spin_motor = false; 
long spin_motor_last_step = millis();
bool spin_motor_last_digital_write = false;


void start_flex() {
  flex_state = FLEX_STATE::FLEX_RAISING;
  run_yaxis_motor = true; 
  yaxis_motor_last_step = millis();
}

void stop_flex() {
  run_yaxis_motor = false; 
  run_spin_motor = false; 
  flex_state = FLEX_STATE::FLEX_IDLE;
}

void calibrate_flex() {
  loginfo("calibrate flex; TODO"); //TODO: Implement calibration
}

void check_flex() {

  // sleep_pin 

  // drive the motor if the flag has been set to run it // TODO implimet the sleep pin as well 
  if ((yaxis_motor_last_step+2 < millis()) && run_yaxis_motor == true) {
    loginfo("triggered correctly");

    // digitalWrite(step_pin, !yaxis_motor_last_digital_write);
    digitalWrite(step_pin, HIGH);
    delay(2);
    digitalWrite(step_pin, LOW);
    delay(2); 

    yaxis_motor_last_digital_write = !yaxis_motor_last_digital_write; 
    yaxis_motor_last_step = millis(); 
  }

  if ((spin_motor_last_step+2 < millis()) && run_spin_motor == true) {
    digitalWrite(step_pin, !spin_motor_last_digital_write);
    spin_motor_last_digital_write = !spin_motor_last_digital_write; 
    spin_motor_last_step = millis();
  }


  switch (flex_state) {
    case FLEX_STATE::FLEX_IDLE:
      
      break;
    case FLEX_STATE::FLEX_RAISING:

      if (upper_limit_switched() == true) {
        run_yaxis_motor = false; 
        flex_state = FLEX_STATE::FLEX_LOWERING; 
        run_spin_motor = true; 
        spin_motor_last_step = millis();
      }
      
      break;
    case FLEX_STATE::FLEX_LOWERING:
      if (lower_limit_switched()) {
        run_yaxis_motor = false; 
        run_spin_motor = false; 
        flex_state = FLEX_STATE::FLEX_IDLE; 
        flex_module->publish_status(MODULE_STATUS::COMPLETE);
      }
      
      break;
  }
  flex_module->publish_state((int) flex_state);
};

// ----- HEIGHT -----
MODULE* height_module;
int HEIGHT_SENSOR_PIN = A1; //TODO: VERIFY PIN NUMBER

enum HEIGHT_STATE {
    HEIGHT_IDLE = 0,
};
HEIGHT_STATE height_state = HEIGHT_STATE::HEIGHT_IDLE;

bool height_sensor_switched() {
  return (digitalRead(HEIGHT_SENSOR_PIN) == 1);
}

void start_height() {
  loginfo("start height; TODO"); //TODO: Implement height
}

void stop_height() {
  loginfo("stop height; TODO"); //TODO: Implement height
}

void calibrate_height() {
  loginfo("calibrate height; TODO"); //TODO: Implement height
}

void verify_height_complete() {
  loginfo("verify height complete; TODO"); //TODO: Implement height
}

void check_height() {
  loginfo("check height; TODO"); //TODO: Implement height
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
  
  flex_module = init_module("flex",
    start_flex, 
    verify_flex_complete, 
    stop_flex,
    calibrate_flex);

  // height_module = init_module("height",
  //   start_height,
  //   verify_height_complete,
  //   stop_height,
  //   calibrate_height);

  //Register ROS publishers
  nh.advertise(weight_feedback_pub);
  
  // conveyor pins 
  pinMode(BACKUP_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(CENTER_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;

  // scale pins
  scaleSerial.begin(9600);
  pinMode(SCALE_RELAY__POWER_PIN, OUTPUT);
  pinMode(SCALE_RELAY__TARE_PIN, OUTPUT);

  // flex pins
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  // pinMode(sleep_pin, OUTPUT);
  pinMode(UPPER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LOWER_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // height pins
  //pinMode(HEIGHT_SENSOR_PIN, INPUT_PULLUP);

  loginfo("setup() Complete");
}


void loop() {
  periodic_status();
  nh.spinOnce();
  parseIncomingData();
  check_scale();
  check_conveyor();
  check_flex();
  //check_height();

  // ----- testing ----- 
  // if (verify_motion_complete()) {
  //   loginfo("debugging test reset");
  //   delay(5000);
  //   start_conveyor();
  // }
}
