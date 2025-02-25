#define NODE_NAME String("module_b")
#define STATUS_FREQ 1500 // ms

#include <Arduino.h>
#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <HardwareSerial.h> //for scale

#define Serial SerialUSB

// ----- MAIN CONVEYOR -----

MODULE *conveyor_module;
int BACKUP_BEAM_BREAK_PIN = 3; // Verified this pin as the green beam break
int CENTER_BEAM_BREAK_PIN = 2; // Verified this pin as the black beam break

enum CONVEYOR_STATE
{
    CONVEYOR_IDLE = 0,
    ADVANCING = 1,
    WAITING_FOR_INTAKE = 2,
    CENTERING = 3,
    BACKING_UP = 4
};
CONVEYOR_STATE conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
unsigned long CONVEYOR_ADVANCING_TIME = millis();

// ---------- ---------- CONVEYOR MOTOR FUNCTIONS ---------- ----------

int CONVEYOR_PIN_SPEED = 11;
int CONVEYOR_PIN_INVERT = 4;

void start_conveyor_forward(int speed = 170)
{
    digitalWrite(CONVEYOR_PIN_INVERT, LOW);
    analogWrite(CONVEYOR_PIN_SPEED, speed); // start
    loginfo("conveyor moving forward");
}

void start_conveyor_backward(int speed = 170)
{
    digitalWrite(CONVEYOR_PIN_INVERT, HIGH);
    analogWrite(CONVEYOR_PIN_SPEED, speed); // start
    loginfo("conveyor moving backward");
}

void stop_conveyor()
{
    analogWrite(CONVEYOR_PIN_SPEED, 0);
    loginfo("conveyor stopping");
}

// ---------- ---------- ROS CONVEYOR FUNCTIONS ---------- ----------

void handle_conveyor_start()
{
    if (conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE)
    {
        loginfo("start_conveyor in IDLE --> advancing disc");
        conveyor_state = CONVEYOR_STATE::ADVANCING;
        CONVEYOR_ADVANCING_TIME = millis();
        start_conveyor_forward();
    }
    else if (conveyor_state == CONVEYOR_STATE::WAITING_FOR_INTAKE)
    {
        loginfo("start_conveyor in WAITING FOR INTAKE --> centering discs");
        conveyor_state = CONVEYOR_STATE::CENTERING;
        start_conveyor_forward();
    }
    else
    {
        logwarn("start_conveyor called in invalid state, " + String((int)conveyor_state));
    }
}

void handle_conveyor_stop()
{
    conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
    stop_conveyor();
}

bool verify_conveyor_complete()
{
    return conveyor_state == CONVEYOR_STATE::CONVEYOR_IDLE;
}

void calibrate_conveyor()
{
    // starts CONVEYOR_BACKING_UP state so that the conveyor moves to the back up beam break
    // should be called before starting the conveyor
    loginfo("calibrating conveyor");
    conveyor_state = CONVEYOR_STATE::CENTERING;
    handle_center_beam_broken();
}

// ---------- ---------- CONVEYOR TIMER CHECK & HANDLE ---------- ----------

bool check_conveyor_timer()
{
    return CONVEYOR_ADVANCING_TIME + 1000 < millis();
}

void handle_conveyor_timer()
{
    if (conveyor_state == CONVEYOR_STATE::ADVANCING)
    {
        conveyor_state = CONVEYOR_STATE::WAITING_FOR_INTAKE;
        stop_conveyor();
    }
}

// ---------- ---------- CONVEYOR BEAM BREAKS CHECK & HANDLE ---------- ----------

bool center_beam_break_prev = 0;
bool check_center_beam_broken()
{
    bool center_beam_break_val = digitalRead(CENTER_BEAM_BREAK_PIN); // read beam break pin
    if (center_beam_break_val != center_beam_break_prev)
        loginfo("Center beam break changed state to: " + String(center_beam_break_val)); // logging function
    bool beam_broken = center_beam_break_val == 0 && center_beam_break_prev == 1;
    center_beam_break_prev = center_beam_break_val; // set previous value to current value
    return beam_broken;
}

void handle_center_beam_broken()
{
    if (conveyor_state == CONVEYOR_STATE::CENTERING)
    {
        conveyor_state = CONVEYOR_STATE::BACKING_UP;
        start_conveyor_backward();
    }
}

bool backup_beam_break_prev = 0;
bool check_backup_beam_broken()
{
    bool backup_beam_break_val = digitalRead(BACKUP_BEAM_BREAK_PIN); // read beam break pin
    if (backup_beam_break_val != backup_beam_break_prev)
        loginfo("Backup beam break changed state to: " + String(backup_beam_break_val)); // logging function
    bool beam_broken = backup_beam_break_val == 0 && backup_beam_break_prev == 1;
    backup_beam_break_prev = backup_beam_break_val; // set previous value to current value
    return beam_broken;
}

void handle_backup_beam_broken()
{
    if (conveyor_state == CONVEYOR_STATE::BACKING_UP)
    {
        conveyor_state = CONVEYOR_STATE::CONVEYOR_IDLE;
        stop_conveyor();
        conveyor_module->publish_status(MODULE_STATUS::COMPLETE);
    }
}

// ---------- ---------- CONVEYOR LOOP ---------- ----------

void conveyor_loop()
{
    if (check_conveyor_timer())
        handle_conveyor_timer();
    if (check_center_beam_broken())
        handle_center_beam_broken();
    if (check_backup_beam_broken())
        handle_backup_beam_broken();
    conveyor_module->publish_state((int)conveyor_state);
}

// ---------- ---------- SETUP ---------- ----------

void setup()
{
    init_std_node();

    conveyor_module = init_module("main_conveyor",
                                  handle_conveyor_start,
                                  verify_conveyor_complete,
                                  handle_conveyor_stop,
                                  calibrate_conveyor);

    // conveyor pins
    pinMode(BACKUP_BEAM_BREAK_PIN, INPUT_PULLUP);
    pinMode(CENTER_BEAM_BREAK_PIN, INPUT_PULLUP);
    pinMode(CONVEYOR_PIN_SPEED, OUTPUT);
    pinMode(CONVEYOR_PIN_INVERT, OUTPUT);

    loginfo("setup() Complete");
}

// ---------- ---------- LOOP ---------- ----------

void loop()
{
    periodic_status();
    nh.spinOnce();
    conveyor_loop();
}
