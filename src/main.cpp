#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms
// #define Serial SerialUSB

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <HardwareSerial.h> //for scale


// ----- FLEX ----- 

MODULE* flex_module;
int dir_pin = 6;
int step_pin = 7;
int sleep_pin = 9; // Verify this pin  // FIX LAST ONE //un commented, hope it works >.<
int UPPER_LIMIT_SWITCH_PIN = 12; 
int LOWER_LIMIT_SWITCH_PIN = 11; 

enum FLEX_STATE {
  FLEX_IDLE = 0,
  FLEX_RAISING = 1,
  FLEX_MEASURING = 2,
  FLEX_LOWERING = 3
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
unsigned long yaxis_motor_last_step = millis();
bool yaxis_motor_last_digital_write = false;

bool run_spin_motor = false; 
unsigned long spin_motor_last_step = millis();
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
      digitalWrite(sleep_pin, HIGH); //A logic high allows normal operation of the A4988 by removing from sleep
      break;
    case FLEX_STATE::FLEX_RAISING:

      if (upper_limit_switched() == true) {
        run_yaxis_motor = false; 
        flex_state = FLEX_STATE::FLEX_MEASURING; 
        run_spin_motor = true; 
        spin_motor_last_step = millis();
      }
      
      break;
      case FLEX_STATE::FLEX_MEASURING:
      break;      
    case FLEX_STATE::FLEX_LOWERING:
      if (lower_limit_switched()) {
        digitalWrite(sleep_pin, LOW);
        // run_yaxis_motor = false; 
        // run_spin_motor = false; 
        // flex_state = FLEX_STATE::FLEX_IDLE; 
        // flex_module->publish_status(MODULE_STATUS::COMPLETE);
      }
      
      break;
  }
  flex_module->publish_state((int) flex_state);
};


// ----- loop/setup functions -----
void setup() {
   init_std_node();

   
  flex_module = init_module("flex",
    start_flex, 
    verify_flex_complete, 
    stop_flex,
    calibrate_flex);


  // flex pins
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(sleep_pin, OUTPUT);
  pinMode(UPPER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LOWER_LIMIT_SWITCH_PIN, INPUT_PULLUP);


  loginfo("setup() Complete");
}


void loop() {
  periodic_status();
  nh.spinOnce();

  
  check_flex();


  // ----- testing ----- 
  // if (verify_motion_complete()) {
  //   loginfo("debugging test reset");
  //   delay(5000);
  //   start_conveyor();
  // }
}
