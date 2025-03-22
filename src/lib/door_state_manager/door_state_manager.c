#include <stdio.h>
#include <stdbool.h>

#include <zephyr/kernel.h>

#include "door_state_manager.h"

#include "door_state_machine.h"
#include "ambient_light_sensor.h"
#include "distance_sensor.h"
#include "motion_sensor.h"

// Threshold values for sensors
#define MOTION_THRESHOLD_G          0.5f
#define LIGHT_LOW_THRESHOLD_LUX     18
#define LIGHT_HIGH_THRESHOLD_LUX    20
#define DISTANCE_THRESHOLD_CM       1

static door_state_t door_state             = DOOR_UNKNOWN;
static bool door_state_manager_initialized = false;

static void check_sensors(void);
static void door_state_change_handler(door_state_machine_t old_state, door_state_machine_t new_state);

int door_state_manager_init(void)
{
    printf("door_state_manager_init\n");

    // init ambient light sensor
    ambient_light_sensor_init();
    // set interrupt threshold for light sensor
    ambient_light_sensor_set_threshold(LIGHT_LOW_THRESHOLD_LUX, LIGHT_HIGH_THRESHOLD_LUX);

    // init motion sensor
    motion_sensor_init();
    // set interrupt threshold for motion sensor
    motion_sensor_set_threshold(MOTION_THRESHOLD_G);

    // init distance sensor
    distance_sensor_init();

    // init door state machine
    if (door_state_machine_init() != 0)
    {
        printf("door_state_machine_init failed\n");

        return -1;
    }

    // register callback to door state machine when there is change in door state
    if (door_state_machine_register_callback(door_state_change_handler) != 0)
    {
        printf("door_state_machine_register_callback failed\n");

        return -1;
    }

    door_state_manager_initialized = true;

    return 0;
}

door_state_t door_state_manager_get_state(void)
{
    // make sure door state manager is initialized
    if (door_state_manager_initialized == false)
    {
        door_state_manager_init();
    }

    // check sensors
    check_sensors();

    // door_state can changed or unchanged based on check_sensors()
    return door_state;
}


static void check_sensors(void)
{
    // get current state from door state machine
    door_state_machine_t current_state = door_state_machine_get_state();

    // check for motion sensor interrupt trigger
    if (motion_sensor_is_interrupt_triggered())
    {
        printf("Motion sensor interrupt triggered\n");
        // process this event that trigger state change
        door_state_machine_process_event(EVENT_MOTION_SENSOR_INTERRUPT);
        // clear the interrupt it it is not auto clear
        motion_sensor_clear_interrupt();
        // get current state from door state machine
        current_state = door_state_machine_get_state();
    }

    if (ambient_light_sensor_is_interrupt_triggered())
    {
        printf("Light sensor interrupt triggered\n");
        // process this event that trigger state change
        door_state_machine_process_event(EVENT_LIGHT_SENSOR_INTERRUPT);
        // clear the interrupt it it is not auto clear
        ambient_light_sensor_clear_interrupt();
        // get current state from door state machine
        current_state = door_state_machine_get_state();
    }

    if (current_state == STATE_CONFIRM_WITH_DISTANCE_SENSOR)
    {
        if (!distance_sensor_is_active())
        {
            printf("Activating distance sensor\n");

            // activate distance sensor
            distance_sensor_activate();
        }

        uint32_t distance = 0;
        uint16_t count_distance_above_threshold = 0;
        uint16_t count_distance_below_threshold = 0;

        // add logic to poll distance sensor reading for about 5 seconds
        int64_t start_time = k_uptime_get();
        int64_t timeout_ms = 5000; // 5 seconds in milliseconds

        while ((k_uptime_get() - start_time) < timeout_ms)
        {
            if (distance_sensor_is_ready())
            {
                printf("Distance sensor ready\n");
    
                distance = distance_sensor_get();

                if (distance > DISTANCE_THRESHOLD_CM)
                {
                    printf("Distance above threshold\n");
                    count_distance_above_threshold++;
                }
                else
                {
                    printf("Distance below threshold\n");
                    count_distance_below_threshold++;
                }

                if (count_distance_above_threshold > 0)
                {
                    printf("Door opened\n");
                    door_state_machine_process_event(EVENT_DOOR_OPEN_DETECTED);
                    distance_sensor_deactivate();

                    break;
                }
                else if (count_distance_below_threshold > 0)
                {
                    printf("Door closed\n");
                    door_state_machine_process_event(EVENT_DOOR_CLOSE_DETECTED);
                    distance_sensor_deactivate();

                    break;
                }
            }
        }
    }
}

/**
 * @brief Handles door state change events from the door state machine.
 *
 * This function is registered as a callback to the door state machine to
 * handle state change events. It is called whenever the door state machine
 * changes its state. The function changes the door state variable based on
 * the new state of the door state machine and prints the state change message.
 *
 * @param old_state The old state of the door state machine before the state change.
 * @param new_state The new state of the door state machine after the state change.
 */
static void door_state_change_handler(door_state_machine_t old_state, door_state_machine_t new_state)
{
    printf("State changed: %d -> %d\n", old_state, new_state);

    switch (new_state)
    {
        case STATE_DOOR_CLOSED:
            door_state = DOOR_CLOSED;
            break;
        case STATE_DOOR_OPEN:
            door_state = DOOR_OPEN;
            break;
        case STATE_UNKNOWN:
            door_state = DOOR_UNKNOWN;
            break;

        case STATE_TRIGGERED_BY_MOTION_SENSOR:
        case STATE_TRIGGERED_BY_LIGHT_SENSOR:
        case STATE_CONFIRM_WITH_DISTANCE_SENSOR:
        default:
            break;
    }
}
