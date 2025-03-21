#include "door_state_manager.h"

#include <stdio.h>

void door_state_manager_init(void)
{
    printf("door_state_manager_init\n");

    // TODO LIST:
    // init door state machine
    // init ambient light sensor
    // register callback to ambient light sensor
    // init motion sensor
    // register callback to motion sensor
    // init distance sensor
    // register callback to distance sensor
    // determine initial door state
    // set initial door state into door state machine
    // optional, return initial state?
}

door_state_t door_state_manager_get_state(void)
{
    // TODO LIST:
    // process event that trigger state change
    // if door state is closed, what to do
    // if door state is opened, what to do
    // return current door state based on state machine
    return DOOR_CLOSED;
}