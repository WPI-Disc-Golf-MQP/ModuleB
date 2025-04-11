#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <HardwareSerial.h> //for scale

#define Serial SerialUSB

// ----- SCALE -----

MODULE *scale_module;
#define SCALE_SERIAL_RX_PIN D4
#define SCALE_SERIAL_TX_PIN D5 // not used

#define SCALE_RELAY_POWER_PIN D11
#define SCALE_RELAY_TARE_PIN D12

HardwareSerial scaleSerial(SCALE_SERIAL_RX_PIN, SCALE_SERIAL_TX_PIN);
const byte numChars = 16;
float lastWeight = 0.0;
char receivedChars[numChars];

std_msgs::Float32 weight_msg;
String _weight_topic("scale_feedback__weight");
ros::Publisher weight_feedback_pub(_weight_topic.c_str(), &weight_msg);

enum SCALE_STATE
{
    SCALE_IDLE = 0,
    MEASURING = 1,
    TARING = 2,
    POWERING_ON = 3,
    POWERING_OFF = 4
};
SCALE_STATE scale_state = SCALE_STATE::SCALE_IDLE;

unsigned long last_scale_data_time = millis();
unsigned long start_scale_action_time = millis();

// ---------- ---------- SCALE FUNCTIONS ---------- ----------

void scale_toggle_power()
{
    digitalWrite(SCALE_RELAY_POWER_PIN, HIGH);
    delay(2000);
    digitalWrite(SCALE_RELAY_POWER_PIN, LOW);
}

void scale_toggle_tare()
{
    digitalWrite(SCALE_RELAY_TARE_PIN, HIGH);
    delay(2000);
    digitalWrite(SCALE_RELAY_TARE_PIN, LOW);
}

void scale_serial_parse_data()
{
    static bool recvInProgress = false;
    static byte ndx = 0;
    char rc;
    char startMarker = '+';
    char endMarker = '\n';

    while (scaleSerial.available() > 0)
    {
        rc = scaleSerial.read();
        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                    ndx = numChars - 1;
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                if (atoff(receivedChars) != lastWeight)
                {
                    lastWeight = atoff(receivedChars);
                    // if (scale_state == SCALE_STATE::MEASURING) {
                    weight_msg.data = lastWeight;
                    weight_feedback_pub.publish(&weight_msg);
                    //}
                }
            }
        }
        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

// ---------- ---------- ROS SCALE FUNCTIONS ---------- ----------

void handle_scale_start()
{
    loginfo("start scale");
    start_scale_action_time = millis();
    scale_state = SCALE_STATE::MEASURING;
}

void handle_scale_stop()
{
    loginfo("stop scale");
    scale_state = SCALE_STATE::SCALE_IDLE;
}

bool verify_scale_complete()
{
    return scale_state == SCALE_STATE::SCALE_IDLE;
}

void calibrate_scale()
{
    loginfo("calibrate scale; TODO"); // TODO: Implement calibration
}

// ---------- ---------- SCALE TIMER CHECK & HANDLE ---------- ----------



// ---------- ---------- SCALE LOOP ---------- ----------

void check_scale()
{
    scale_serial_parse_data();

    switch (scale_state)
    {
    case SCALE_STATE::MEASURING:
        if (start_scale_action_time + 1500 < millis())
        { // measurement complete
            weight_msg.data = lastWeight;
            scale_module->publish_status(MODULE_STATUS::COMPLETE);
            scale_state = SCALE_STATE::SCALE_IDLE;
            loginfo("scale measurement complete");
        }
        break;
    case SCALE_STATE::TARING:
        if (start_scale_action_time + 2000 < millis())
        { // button press complete
            digitalWrite(SCALE_RELAY_TARE_PIN, LOW);
            scale_state = SCALE_STATE::SCALE_IDLE;
        }
        break;
    case SCALE_STATE::POWERING_ON:
        if (start_scale_action_time + 2000 < millis())
        { // button press complete
            digitalWrite(SCALE_RELAY_POWER_PIN, LOW);
            scale_state = SCALE_STATE::SCALE_IDLE;
        }
        break;
    case SCALE_STATE::POWERING_OFF:
        if (start_scale_action_time + 2000 < millis())
        { // button press complete
            digitalWrite(SCALE_RELAY_POWER_PIN, LOW);
            scale_state = SCALE_STATE::SCALE_IDLE;
        }
        break;
    case SCALE_STATE::SCALE_IDLE:
        break;
    default:
        break;
    }
    scale_module->publish_state((int)scale_state);

    // TODO: Implement scale power on
    //  if (last_scale_data_time+1000 < millis()) { //If we haven't heard from the scale, turn it on!
    //    scale_state = SCALE_STATE::POWERING_ON;
    //  }
}


// ----- loop/setup functions -----
void setup()
{
    init_std_node();
    scale_module = init_module("scale",
                               handle_scale_start,
                               verify_scale_complete,
                               handle_scale_stop,
                               calibrate_scale);

    // Register ROS publishers
    nh.advertise(weight_feedback_pub);

    // scale pins
    scaleSerial.begin(9600);
    pinMode(SCALE_RELAY_POWER_PIN, OUTPUT);
    pinMode(SCALE_RELAY_TARE_PIN, OUTPUT);

    loginfo("setup() Complete");
}

void loop()
{
    periodic_status();
    nh.spinOnce();
    check_scale();
}
