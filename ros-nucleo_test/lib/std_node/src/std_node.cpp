/*
 * rosserial Standard Node Interface
 */

#include <Arduino.h>
#include <ros.h>
#include "HardwareSerial.h"
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <functional>

#ifndef NODE_NAME
    #define NODE_NAME String("std_node")
#endif
#ifndef STATUS_FREQ
    #define STATUS_FREQ 1500 // ms
#endif

enum NODE_STATUS {
    INITIALIZING_NODE = 0, 
    IDLE = 1, 
    MOTION_IN_PROGRESS = 2, 
    MOTION_COMPLETE = 3, 
    MEASURE_IN_PROGRESS = 4,
    MEASURE_COMPLETE = 5,
    INVALID_MEASURE__REPEAT = 6, 
    ERROR__REBOOT = 7};

enum REQUEST {
    INITIALIZING = 0, 
    WAITING = 1, 
    START_MOTION = 2, 
    VERIFY__MOTION_COMPLETE = 3, 
    START_MEASURE = 4, 
    VERIFY__MEASURE_COMPLETE = 5, 
    REBOOT = 6};

HardwareSerial hserial(PA_15, PA_2); // NUCLEO-F303K8 RX, TX
#define Serial1 hserial // This will overwrite the current Serial1 serial port and will use hserial port.
#define USE_STM32_HW_SERIAL
#define __STM32F3xxxx__

ros::NodeHandle nh;
std_msgs::Int8 request;
std::function<void(int)> _ref__request_callback;
std_msgs::Int8 status;
String _status_topic(NODE_NAME + "_status");
ros::Publisher status_pub(_status_topic.c_str(), &status);
String _request_topic(NODE_NAME + "_request");
void process_request_callback(const std_msgs::Int8& msg);
ros::Subscriber<std_msgs::Int8> request_sub(_request_topic.c_str(), &process_request_callback);
unsigned long last_status = 0;

void init_std_node() {
    nh.initNode();
    nh.advertise(status_pub);
    nh.subscribe(request_sub);
    nh.setSpinTimeout(100);
    request.data = REQUEST::INITIALIZING;
    status.data = NODE_STATUS::IDLE;
    status_pub.publish(&status);
    last_status = millis();
}

String get_node_tag() { return String("[")+NODE_NAME+String("] "); };
void loginfo(String msg) { nh.loginfo((get_node_tag()+msg).c_str()); };
void logwarn(String msg) { nh.logwarn((get_node_tag()+msg).c_str()); };
void logerr(String msg) { nh.logerror((get_node_tag()+msg).c_str()); };

void set_request_callback(std::function<void(int)> _request_callback) { 
    _ref__request_callback = std::bind(_request_callback, std::placeholders::_1); 
    };

            
void publish_status() {
    status_pub.publish(&status);
    last_status = millis();
}

void periodic_status() {
    if (millis() - last_status > STATUS_FREQ) {
        publish_status();
    }
}

void process_request_callback(const std_msgs::Int8& msg) {
    status.data = NODE_STATUS::MOTION_IN_PROGRESS;
    status_pub.publish(&status);
    loginfo("Request Received, "+String(msg.data));

    _ref__request_callback(msg.data);

    status.data = NODE_STATUS::MOTION_COMPLETE;
    status_pub.publish(&status);
    status.data = NODE_STATUS::IDLE;
}

