#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms
// #define Serial SerialUSB

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>
#include <HX711.h>

// ----- FLEX -----

MODULE *flex_module;

HX711 flex_load_cell_left;
HX711 flex_load_cell_right;

int FLEX_MOTOR_DIR_PIN = 7;
int FLEX_MOTOR_STEP_PIN = 8;
int FLEX_MOTOR_SLEEP_PIN = A1;
int FLEX_UPPER_LIMIT_PIN = 4;
int FLEX_LOWER_LIMIT_PIN = 5;
int FLEX_LOAD_CELL_LEFT_DOUT_PIN = 12;
int FLEX_LOAD_CELL_LEFT_SCK_PIN = 13;
int FLEX_LOAD_CELL_RIGHT_DOUT_PIN = A0;
int FLEX_LOAD_CELL_RIGHT_SCK_PIN = A3;

enum FLEX_STATE
{
    FLEX_IDLE = 0,
    FLEX_RAISING = 1,
    FLEX_MEASURING = 2,
    FLEX_LOWERING = 3
};
FLEX_STATE flex_state = FLEX_STATE::FLEX_IDLE;

// ---------- ---------- FLEX STEPPER MOTOR FUNCTIONS ---------- ----------

unsigned long FLEX_MOTOR_TIME = 1; // 1000 microseconds

unsigned long flex_motor_previous_time;
bool check_flex_motor_timer()
{
    return millis() - flex_motor_previous_time >= FLEX_MOTOR_TIME;
}

bool flex_motor_pulse;
int stepper_motor_counter;
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
        stepper_motor_counter++;
    }
}

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

// ---------- ---------- FLEX LIMIT SWITCH CHECK & HANDLE ---------- ----------

bool upper_limit_prev_val = 0;
bool check_flex_upper_limit()
{
    bool upper_limit_val = digitalRead(FLEX_UPPER_LIMIT_PIN); // read upper limit switch pin
    if (upper_limit_val != upper_limit_prev_val)
        loginfo("Flex upper limit switch changed to: " + String(upper_limit_val)); // logging function
    bool Upper_limit_switched = upper_limit_val == 0 && upper_limit_prev_val == 1;
    upper_limit_prev_val = upper_limit_val; // set previous value to current value
    return Upper_limit_switched;
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

bool lower_limit_prev_val = 0;
bool check_flex_lower_limit()
{
    bool lower_limit_val = digitalRead(FLEX_LOWER_LIMIT_PIN); // read upper limit switch pin
    if (lower_limit_val != lower_limit_prev_val)
        loginfo("Flex lower limit switch changed to: " + String(lower_limit_val)); // logging function
    bool lower_limit_switched = lower_limit_val == 0 && lower_limit_prev_val == 1;
    lower_limit_prev_val = lower_limit_val; // set previous value to current value
    return lower_limit_switched;
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

long flex_load_cell_reading_left;
long flex_load_cell_reading_right;
long FLEX_LOAD_CELL_LIMIT = 500000;

boolean check_flex_load_cells()
{
    if (flex_load_cell_left.is_ready())
        flex_load_cell_reading_left = flex_load_cell_left.read();
    if (flex_load_cell_right.is_ready())
        flex_load_cell_reading_right = flex_load_cell_right.read();
    return flex_load_cell_reading_left > FLEX_LOAD_CELL_LIMIT || flex_load_cell_reading_right > FLEX_LOAD_CELL_LIMIT;
}

void handle_flex_load_cells()
{
    if (flex_state == FLEX_STATE::FLEX_MEASURING)
    {
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
    flex_module = init_module("flex",
                              handle_flex_start,
                              verify_flex_complete,
                              handle_flex_stop,
                              calibrate_flex);

    flex_load_cell_left.begin(FLEX_LOAD_CELL_LEFT_DOUT_PIN, FLEX_LOAD_CELL_LEFT_SCK_PIN);
    flex_load_cell_right.begin(FLEX_LOAD_CELL_RIGHT_DOUT_PIN, FLEX_LOAD_CELL_RIGHT_SCK_PIN);

    pinMode(FLEX_MOTOR_DIR_PIN, OUTPUT);
    pinMode(FLEX_MOTOR_STEP_PIN, OUTPUT);
    pinMode(FLEX_MOTOR_SLEEP_PIN, OUTPUT);
    pinMode(FLEX_UPPER_LIMIT_PIN, INPUT_PULLUP);
    pinMode(FLEX_LOWER_LIMIT_PIN, INPUT_PULLUP);

    loginfo("setup() Complete");
}

void loop()
{
    periodic_status();
    nh.spinOnce();
    flex_loop();
}
