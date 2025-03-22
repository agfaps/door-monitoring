#include "door_state_machine.h"

#include <stdio.h>
#include <stdbool.h>

static door_state_machine_t current_state = STATE_INIT;

static void (*state_change_callback)(door_state_machine_t old_state, door_state_machine_t new_state) = NULL;

/**
 * @brief Changes the current state of the door state machine.
 *
 * This function only changes the state if the new state is different from the
 * current state. If the state is changed, it calls the registered state change
 * callback function, if any.
 *
 * @param new_state The new state of the door state machine
 */
static void change_state(door_state_machine_t new_state)
{
    if (new_state != current_state)
    {
        door_state_machine_t old_state = current_state;
        current_state                  = new_state;
        
        if (state_change_callback != NULL)
        {
            state_change_callback(old_state, new_state);
        }
    }
}

/**
 * @brief Handles the state transition for the door state machine based on the given event.
 *
 * This function evaluates the current state of the door state machine and the event 
 * provided to determine the appropriate state transition. It processes events such as 
 * power on, motion sensor interrupts, and light sensor interrupts to transition between 
 * states like STATE_INIT, STATE_DOOR_CLOSED, STATE_DOOR_OPEN, STATE_TRIGGERED_BY_MOTION_SENSOR, 
 * and STATE_CONFIRM_WITH_DISTANCE_SENSOR. If a valid state transition occurs, the state 
 * is updated and the registered state change callback is invoked. If the state is unknown 
 * or invalid, the function returns an error.
 *
 * @param event The event triggering the state transition.
 * @return 0 if the state transition is successful, -1 if the state is invalid.
 */
static int handle_state_transition(door_event_t event)
{
    // process event that trigger state change
    switch (current_state)
    {
        case STATE_INIT:
            if (event == EVENT_POWER_ON)
            {
                // After power on, immediately switch to distance confirmation to determine initial door state if open or closed
                change_state(STATE_CONFIRM_WITH_DISTANCE_SENSOR);

                return 0;
            }
            break;
        case STATE_DOOR_CLOSED:
            if (event == EVENT_MOTION_SENSOR_INTERRUPT)
            {
                change_state(STATE_TRIGGERED_BY_MOTION_SENSOR);

                return 0;
            }
            else if (event == EVENT_LIGHT_SENSOR_INTERRUPT)
            {
                change_state(STATE_TRIGGERED_BY_LIGHT_SENSOR);

                return 0;
            }
            break;
        case STATE_DOOR_OPEN:
            if (event == EVENT_MOTION_SENSOR_INTERRUPT)
            {
                change_state(STATE_TRIGGERED_BY_MOTION_SENSOR);

                return 0;
            }
            else if (event == EVENT_LIGHT_SENSOR_INTERRUPT)
            {
                change_state(STATE_TRIGGERED_BY_LIGHT_SENSOR);

                return 0;
            }
            break;

        case STATE_TRIGGERED_BY_MOTION_SENSOR:
        case STATE_TRIGGERED_BY_LIGHT_SENSOR:
            change_state(STATE_CONFIRM_WITH_DISTANCE_SENSOR);

            return 0;
            break;
        case STATE_CONFIRM_WITH_DISTANCE_SENSOR:
            if (event == EVENT_DOOR_CLOSE_DETECTED)
            {
                change_state(STATE_DOOR_CLOSED);

                return 0;
            }
            else if (event == EVENT_DOOR_OPEN_DETECTED)
            {
                change_state(STATE_DOOR_OPEN);

                return 0;
            }
            break;
        case STATE_UNKNOWN:
        default:
            // Invalid state
            return -1;
            break;
    }

    return 0;
}

/**
 * @brief Initializes the door state machine.
 *
 * This function initializes the door state machine by setting the initial state
 * to STATE_INIT and resetting the state change callback function. It then
 * processes the EVENT_POWER_ON event to transition the state machine to the
 * correct initial state.
 *
 * @return 0 on success, -1 on failure.
 */
int door_state_machine_init(void)
{
    // init door state machine
    current_state         = STATE_INIT;
    state_change_callback = NULL;

    return door_state_machine_process_event(EVENT_POWER_ON);
}

/**
 * @brief Processes an event to trigger a state change in the door state machine.
 *
 * This function takes an event as input and processes it to determine if a state
 * change is necessary in the door state machine. The function handles various events
 * such as power on, door open detected, door close detected, and sensor interrupts
 * to transition the state machine appropriately.
 *
 * @param event The event that triggers a potential state change.
 * @return 0 on successful processing of the event, -1 on failure.
 */
int door_state_machine_process_event(door_event_t event)
{
    // process event that trigger state change
    return handle_state_transition(event);
}

/**
 * @brief Gets the current state of the door state machine.
 *
 * This function returns the current state of the door state machine. The state
 * is one of the values defined in the door_state_machine_t enumeration.
 *
 * @return The current state of the door state machine
 */
door_state_machine_t door_state_machine_get_state(void)
{
    // return current door state based on state machine
    return current_state;
}

/**
 * @brief Registers a callback function for state changes in the door state machine.
 *
 * This function allows the user to register a callback that will be called
 * whenever there is a change in the state of the door state machine. The callback
 * receives the old state and the new state as parameters.
 *
 * @param callback A pointer to the callback function to be registered. The callback
 *                 function should take two parameters: the old state and the new state.
 * @return 0 on successful registration of the callback, -1 if the callback is NULL.
 */
int door_state_machine_register_callback(void (*callback)(door_state_machine_t old_state, door_state_machine_t new_state))
{
    // register callback to door state machine when there is change in door state
    if (callback != NULL)
    {
        state_change_callback = callback;

        return 0;
    }

    return -1;
}