#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <std_msgs/Int32MultiArray.h>
#include "wiring_private.h"

#define Serial SerialUSB

// ----- SCALE -----

MODULE *scale_module;

#define SCALE_RELAY_POWER_PIN A0
#define SCALE_RELAY_TARE_PIN 13

Uart serial4(&sercom4, A2, A1, SERCOM_RX_PAD_1, UART_TX_PAD_0); // A2 is the RX; A1 is the TX.
void SERCOM4_Handler()
{
    serial4.IrqHandler();
}
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
    POWERING_ON = 3
};
SCALE_STATE scale_state = SCALE_STATE::SCALE_IDLE;

unsigned long start_scale_action_time = millis();

// ---------- ---------- SCALE FUNCTIONS ---------- ----------

void scale_powering_on_start()
{
    digitalWrite(SCALE_RELAY_POWER_PIN, HIGH);
    
}

void scale_powering_on_stop() 
{
    digitalWrite(SCALE_RELAY_POWER_PIN, LOW);
}

void scale_tare_on()
{
    digitalWrite(SCALE_RELAY_TARE_PIN, HIGH);
}

void scale_tare_off()
{
    digitalWrite(SCALE_RELAY_TARE_PIN, LOW);
}

void scale_serial_parse_data()
{
    static bool recvInProgress = false;
    static byte ndx = 0;
    char rc;
    char startMarker = '+';
    char endMarker = '\n';

    while (serial4.available() > 0)
    {
        rc = serial4.read();
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
    start_scale_action_time = millis();
    scale_state = SCALE_STATE::POWERING_ON;
    loginfo("calibrating scale");
}

// ---------- ---------- SCALE TIMER CHECK & HANDLE ---------- ----------

bool check_scale_timer()
{
    return start_scale_action_time + 2000 < millis();
}

void handle_scale_timer()
{
    if (scale_state == SCALE_STATE::MEASURING)
    {
        weight_msg.data = lastWeight;
        scale_state = SCALE_STATE::SCALE_IDLE;
        scale_module->publish_status(MODULE_STATUS::COMPLETE);
        loginfo("scale measurement complete: " + String(lastWeight) + " g");
    }
    else if (scale_state == SCALE_STATE::TARING)
    {
        scale_tare_off();
        scale_state = SCALE_STATE::SCALE_IDLE;
    }
    else if (scale_state == SCALE_STATE::POWERING_ON) 
    {
        scale_powering_on_stop();
        scale_tare_on();
        start_scale_action_time = millis();
        scale_state = SCALE_STATE::TARING;
    }
}


// ---------- ---------- SCALE LOOP ---------- ----------

void scale_loop() 
{
    scale_serial_parse_data();

    if (check_scale_timer())
        handle_scale_timer();

    scale_module->publish_state((int)scale_state);
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
    // Adding Sercom pins
    // pinPeripheral(A1, PIO_SERCOM_ALT); // not needed, for TX
    pinPeripheral(A2, PIO_SERCOM_ALT);
    pinMode(SCALE_RELAY_POWER_PIN, OUTPUT);
    pinMode(SCALE_RELAY_TARE_PIN, OUTPUT);

    loginfo("setup() Complete");
}

void loop()
{
    periodic_status();
    nh.spinOnce();
    scale_loop();
}
