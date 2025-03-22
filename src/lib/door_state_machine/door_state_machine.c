#include "door_state_machine.h"

#include <stdio.h>
#include <stdbool.h>

static door_state_machine_t current_state = STATE_INIT;

static void (*state_change_callback)(door_state_machine_t old_state, door_state_machine_t new_state) = NULL;


static int handle_state_transition(door_event_t event) {
    // TODO LIST:
    // process event that trigger state change

    return 0;
}

int door_state_machine_init(void)
{
    // TODO LIST:
    // init door state machine
    current_state         = STATE_INIT;
    state_change_callback = NULL;

    return door_state_machine_process_event(EVENT_POWER_ON);
}

int door_state_machine_process_event(door_event_t event)
{
    // TODO LIST:
    // process event that trigger state change
    return handle_state_transition(event);
}

door_state_machine_t door_state_machine_get_state(void)
{
    // TODO LIST:
    // return current door state based on state machine
    return current_state;
}

int door_state_machine_register_callback(void (*callback)(door_state_machine_t old_state, door_state_machine_t new_state))
{
    // TODO LIST:
    // register callback to door state machine when there is change in door state
    if (callback != NULL)
    {
        state_change_callback = callback;

        return 0;
    }

    return -1;
}