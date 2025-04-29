#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms
#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <Arduino.h>
#include <HX711.h>
#include <button.h>

// #define Serial SerialUSB

// ----- FLEX -----

MODULE *flex_module;

int FLEX_MOTOR_DIR_PIN = 7;
int FLEX_MOTOR_STEP_PIN = 8;
int FLEX_MOTOR_SLEEP_PIN = A1;
int FLEX_UPPER_LIMIT_PIN = 4;
int FLEX_LOWER_LIMIT_PIN = 5;
int FLEX_LOAD_CELL_LEFT_DOUT_PIN = 12;
int FLEX_LOAD_CELL_LEFT_SCK_PIN = 13;
int FLEX_LOAD_CELL_RIGHT_DOUT_PIN = A0;
int FLEX_LOAD_CELL_RIGHT_SCK_PIN = A3;

HX711 flex_load_cell_left;
HX711 flex_load_cell_right;
Button flex_upper_limit(FLEX_UPPER_LIMIT_PIN);
Button flex_lower_limit(FLEX_LOWER_LIMIT_PIN);

unsigned long const FLEX_MOTOR_TIME = 1; // 1000 microseconds
unsigned long flex_motor_previous_time;

long const MAX_STEPPER_MOTOR_COUNTER = 2500; // each rotation is 0.085 in, 200 counts per rotation
long flex_motor_counter;
bool flex_motor_pulse;

long const FLEX_LOAD_CELL_LIMIT = 1000000;
long flex_load_cell_reading_left;
long flex_load_cell_reading_right;

enum FLEX_STATE
{
    FLEX_IDLE = 0,
    FLEX_RAISING = 1,
    FLEX_MEASURING = 2,
    FLEX_LOWERING = 3
};
FLEX_STATE flex_state = FLEX_STATE::FLEX_IDLE;

// ---------- ---------- FLEX STEPPER MOTOR FUNCTIONS ---------- ----------

void start_flex_motor_raising()
{
    FLEX_MOTOR_SLEEP_PIN = HIGH;
    FLEX_MOTOR_DIR_PIN = HIGH; // this is a guess for now
    FLEX_MOTOR_STEP_PIN = LOW;
    flex_motor_pulse = true;
    flex_motor_previous_time = millis();
    loginfo("starting stepper motor, raising");
}

void start_flex_motor_lowering()
{
    FLEX_MOTOR_SLEEP_PIN = HIGH;
    FLEX_MOTOR_DIR_PIN = LOW; // this is a guess for now
    FLEX_MOTOR_STEP_PIN = LOW;
    flex_motor_pulse = true;
    flex_motor_previous_time = millis();
    loginfo("starting stepper motor, lowering");
}

void stop_flex_motor()
{
    FLEX_MOTOR_SLEEP_PIN = LOW;
    loginfo("stopping stepper motor");
}

bool check_flex_motor_timer()
{
    return millis() - flex_motor_previous_time >= FLEX_MOTOR_TIME;
}

void handle_flex_motor_timer()
{
    flex_motor_previous_time = millis();
    if (flex_state == FLEX_STATE::FLEX_RAISING || flex_state == FLEX_STATE::FLEX_MEASURING || flex_state == FLEX_STATE::FLEX_LOWERING)
    {
        digitalWrite(FLEX_MOTOR_STEP_PIN, flex_motor_pulse);
        flex_motor_pulse = !flex_motor_pulse;
    }
    if (flex_state == FLEX_STATE::FLEX_MEASURING)
    {
        flex_motor_counter++;
        if (flex_motor_counter > MAX_STEPPER_MOTOR_COUNTER) 
        {
            loginfo("max stepper motor displacement reached");
            flex_state = FLEX_STATE::FLEX_LOWERING;
            loginfo("displacement: " + String(flex_motor_counter));
            loginfo("load cells: " + String(flex_load_cell_reading_left) + " + " + String(flex_load_cell_reading_right));
            stop_flex_motor();
            start_flex_motor_lowering();
        }
    }
}

// ---------- ---------- FLEX LIMIT SWITCH CHECK & HANDLE ---------- ----------

bool check_flex_upper_limit()
{
    return flex_upper_limit.checkButtonPress();
}

void handle_flex_upper_limit()
{
    if (flex_state == FLEX_STATE::FLEX_RAISING)
    {
        flex_state = FLEX_STATE::FLEX_MEASURING;
        loginfo("upper limit switch pressed");
        stepper_motor_counter = 0;
        stop_flex_motor();
        start_flex_motor_raising();
    }
}

bool check_flex_lower_limit()
{
    return flex_lower_limit.checkButtonPress();
}

void handle_flex_lower_limit()
{
    if (flex_state == FLEX_STATE::FLEX_MEASURING)
    {
        flex_state = FLEX_STATE::FLEX_IDLE;
        loginfo("lower limit switch pressed");
        stop_flex_motor();
    }
}

// ---------- ---------- FLEX LOAD CELLS FUNCTIONS ---------- ----------

boolean check_flex_load_cells()
{
    if (flex_load_cell_left.is_ready())
        flex_load_cell_reading_left = flex_load_cell_left.read();
    if (flex_load_cell_right.is_ready())
        flex_load_cell_reading_right = flex_load_cell_right.read();
    return flex_load_cell_reading_left + flex_load_cell_reading_right > FLEX_LOAD_CELL_LIMIT;
}

void handle_flex_load_cells()
{
    if (flex_state == FLEX_STATE::FLEX_MEASURING)
    {
        loginfo("max load cell limit reached");
        flex_state = FLEX_STATE::FLEX_LOWERING;
        loginfo("displacement: " + String(stepper_motor_counter));
        loginfo("load cells: " + String(flex_load_cell_reading_left) + " + " + String(flex_load_cell_reading_right));
        stop_flex_motor();
        start_flex_motor_lowering();
    }
}

// ---------- ---------- ROS FLEX FUNCTIONS ---------- ----------

void handle_flex_start()
{
    flex_state = FLEX_STATE::FLEX_RAISING;
    start_flex_motor_raising();
}

void handle_flex_stop()
{
    flex_state = FLEX_STATE::FLEX_IDLE;
    stop_flex_motor();
}

bool verify_flex_complete()
{
    return flex_state == FLEX_STATE::FLEX_IDLE;
}

void calibrate_flex()
{
    // TODO: probably start lowering until the lower limit switch is hit
    loginfo("calibrate flex; TODO");
}

// ---------- ---------- FLEX LOOP ---------- ----------

void flex_loop()
{
    if (check_flex_upper_limit())
        handle_flex_upper_limit();
    if (check_flex_lower_limit())
        handle_flex_lower_limit();
    if (check_flex_motor_timer())
        handle_flex_motor_timer();
    if (check_flex_load_cells())
        handle_flex_load_cells();

    flex_module->publish_state((int)flex_state);
}

// ----- loop/setup functions -----
void setup()
{
    // Serial.begin(57600);
    
    flex_module = init_module("flex",
                              handle_flex_start,
                              verify_flex_complete,
                              handle_flex_stop,
                              calibrate_flex);

    flex_load_cell_left.begin(FLEX_LOAD_CELL_LEFT_DOUT_PIN, FLEX_LOAD_CELL_LEFT_SCK_PIN);
    flex_load_cell_right.begin(FLEX_LOAD_CELL_RIGHT_DOUT_PIN, FLEX_LOAD_CELL_RIGHT_SCK_PIN);
    flex_upper_limit.init();
    flex_lower_limit.init();

    pinMode(FLEX_MOTOR_DIR_PIN, OUTPUT);
    pinMode(FLEX_MOTOR_STEP_PIN, OUTPUT);
    pinMode(FLEX_MOTOR_SLEEP_PIN, OUTPUT);

    loginfo("setup() Complete");
}

void loop()
{
    periodic_status();
    nh.spinOnce();
    flex_loop();
}
