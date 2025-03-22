#include "door_state_manager.h"
#include "door_state_machine.h"

#include <stdio.h>

static void calibration_initial_door_state(void);
static void intermediate_distance_sensor_check(void);
static void check_sensors(void);


void door_state_manager_init(void)
{
    printf("door_state_manager_init\n");

    // TODO LIST:
    // init door state machine
    door_state_machine_init();
    // register callback to door state machine when there is change in door state
    // init ambient light sensor
    // init motion sensor
    // init distance sensor
    // set interrupt threshold for motion sensor
    // set interrupt threshold for light sensor
    // determine initial door state
    calibration_initial_door_state();
    // set initial door state into door state machine
    // optional, return initial state?
}

door_state_t door_state_manager_get_state(void)
{
    // TODO LIST:
    // while true loop
    // if current state is state confirm with distance sensor (meaning door is moving from either open or close state)
    // call intermediate_distance_sensor_check
    // add continue so we skip check_sensor to finish the intermediate_distance_sensor_check and exit state confirm with distance sensor

    // check sensors
    check_sensors();
    // process event that trigger state change
    // if door state is closed, what to do
    // if door state is opened, what to do
    // return current door state based on state machine to break from while loop
    return DOOR_CLOSED;
}

static void calibration_initial_door_state(void)
{
    // TODO LIST:
    // determine initial door state
    // set initial door state into door state machine
}

static void intermediate_distance_sensor_check(void)
{
    
}

static void check_sensors(void)
{
    // get current state from door state machine
    // save this DOOR OPENED or DOOR CLOSED state for comparison at the end

    // check for motion sensor interrupt trigger
    // process this event that trigger state change
    // clear the interrupt it it is not auto clear
    // get current state from door state machine

    // check for light sensor interrupt trigger
    // process this event that trigger state change
    // clear the interrupt it it is not auto clear
    // get current state from door state machine

    // check if current state is state triggered by motion or light sensor
    // activate distance sensor
    // start timer for distance measurement timeout
    // start polling for distance measurement every xxx millisecond until certain threshold is reached or timeout
    // if threshold reached and previous door state is OPENED, then change state to CLOSED
    // process this event that trigger state change
    // if threshold reached and previous door state is CLOSED, then change state to OPENED
    // process this event that trigger state change
    // get current state from door state machine
}