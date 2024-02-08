/*
 * rosserial Example Node: LED
 */



#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms
#define SCALE_MEASURE_DURATION 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>

#define SCALE_SERIAL__RX_PIN D4
#define SCALE_SERIAL__TX_PIN D5 //not used

#define SCALE_RELAY__POWER_PIN D6
#define SCALE_RELAY__TARE_PIN D7

/* ------------- *
*   Connections
* -------------- *
* Nucleo <-> RS232 converter
* D4 <-> RX
* 5v <-> VCC
* GND <-> GND
* (D5/TX are not used)
* 
* Nucleo <-> Power Relay
* D6 <-> Signal IN (Purple)
* GND <-> Signal GND (Blue)
* 
* Nucleo <-> Tare Relay
* D7 <-> Signal IN (Grey)
* GND <-> Signal GND (Grey)
*/

/* Scale Serial Configuration */
HardwareSerial scaleSerial(SCALE_SERIAL__RX_PIN, SCALE_SERIAL__TX_PIN); // RX, TX
const byte numChars = 16;

/* Scale Globals */
float lastWeight = 0.0;
char receivedChars[numChars];
 
// define pins 
int BEAM_BREAK_PIN = D9; 
int SPEED_PIN = A0;
int INVERT_PIN = D13;
// 

// scale pins 
#define SCALE_RELAY__POWER_PIN D11
#define SCALE_RELAY__TARE_PIN D12
#define SCALE_SERIAL__RX_PIN D4
#define SCALE_SERIAL__TX_PIN D1 //not used
// 

std_msgs::Float32 weight_msg;
String _weight_topic(NODE_NAME + "_feedback__weight");
ros::Publisher weight_feedback_pub(_weight_topic.c_str(), &weight_msg);

long start_move_time = millis();
bool conveyor_moving = false;
bool conveyor_in_progress = false;

long start_measure_time = millis();
bool measure_complete = false;

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




// conveyor functions 

void move_forward(int speed) {
  digitalWrite(INVERT_PIN, LOW);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move forward");
  start_move_time = millis();
  conveyor_moving = true;
}

void move_backward(int speed) {
  digitalWrite(INVERT_PIN, HIGH);
  analogWrite(SPEED_PIN, speed); // start
  loginfo("move backward");
  conveyor_moving = true;
}

void stop() {
  analogWrite(SPEED_PIN, 0); // stop
  conveyor_moving = false;
}

void is_conveyor_done_huh() {
  return (digitalRead(BEAM_BREAK_PIN) == 0);
}

void move_to_starting_position(int speed, bool delay_bb) {
  move_forward(speed);
  if (delay_bb) {
    delay(1000); // so that it moves off of the beam breaker
  }

  while (true) {
    // delay(100); 
    if (is_conveyor_done_huh()) {
      // it is done moving, so stop the conveyor
      stop(); 
      break; 
    } 
  }
}

void conveyor_start_section(int speed){ // conveyor_start_section(255)
  conveyor_in_progress = true;
  move_to_starting_position(speed, false); // this will reset it to the beam break
  move_to_starting_position(speed, true); // this one will now go to the next section

  move_backward(speed);
  delay(1000);
  stop();
  conveyor_in_progress = false;
}



// void start_motion() {
//   loginfo("start_motion");
//   move_forward(255);
//   loginfo("motion complete");
// }

// void check_conveyor() {
//   if (start_move_time+1000 < millis() && digitalRead(BEAM_BREAK_PIN) == 0) { 
//     //wait 1 second for the conveyor to move off the beam breaker
//     stop();
//     loginfo("beam broken");
//   }
//   //feedback_msg.data = 
//   //feedback_pub.publish(&feedback_msg);
// }

// bool verify_motion_complete() {
//   return !conveyor_moving && digitalRead(BEAM_BREAK_PIN) == 0;
// }









// scale code 


void start_measure() {
  loginfo("start_measure");
  start_measure_time = millis();
  measure_complete = false;
}

void check_measure() {
  if (start_measure_time+SCALE_MEASURE_DURATION < millis()) {
    loginfo("measure complete");
    measure_complete = true;
  }
}

bool verify_measure_complete() {
  return measure_complete;
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
                    if (!measure_complete) {
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



// scale functions 



// loop setup functions 
void setup() {
  init_std_node();
  nh.advertise(weight_feedback_pub);
  set_request_callbacks(start_measure, verify_measure_complete, start_motion, verify_motion_complete);

  pinMode(LED_BUILTIN, OUTPUT);
  
  // conveyor pins 
  pinMode(BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(SPEED_PIN,OUTPUT) ;
  pinMode(INVERT_PIN, OUTPUT) ;

  // scale pins

  scaleSerial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SCALE_RELAY__POWER_PIN, OUTPUT);
  pinMode(SCALE_RELAY__TARE_PIN, OUTPUT);

  loginfo("setup() Complete");
}





void loop() {
  periodic_status();
  nh.spinOnce();
  check_conveyor();
  check_measure();
  parseIncomingData();

  // // conveyor code
  // move_forward(255); 
  // delay(3000);
  // stop();
  // delay(500);

  // // scale code 



  // // ros code 
  // periodic_status();
  // nh.spinOnce();
  // delay(10);
}