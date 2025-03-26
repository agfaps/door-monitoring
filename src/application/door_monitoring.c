#include "door_monitoring.h"

#include <stdbool.h>
#include <stdio.h>

#include <zephyr/kernel.h>
// #include <zephyr/pm/pm.h>
// #include <zephyr/pm/policy.h>
// #include <zephyr/pm/device.h>
// #include <zephyr/pm/state.h>

#include "door_state_manager.h"

void door_monitoring_init()
{
    // printf("door_monitoring_init\n");

    // init door_state_manager
    // printf("init door_state_manager\n");
    door_state_manager_init();
}

void door_monitoring_run()
{
    door_state_t current_state = DOOR_UNKNOWN;
    door_state_t temp_state    = DOOR_UNKNOWN;

    // printf("door_monitoring_run\n");

    // k_sleep(K_SECONDS(10));

    // get initial door state from door_state_manager
    // printf("get initial door state from door_state_manager\n");
    current_state = door_state_manager_get_state();

    /* Configure PM policy to allow entering STOP mode */
    // still not working
    // pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

    // entering forever loop
    // printf("entering forever loop\n");
    while (true)
    {
        // after first starting up or wake up
        // printf("after first starting up or wake up\n");

        // get door state from door_state_manager
        temp_state = door_state_manager_get_state();

        // compare with initial door state
        if (temp_state != current_state)
        {
            // if different, print door state
            printf("DOOR STATE: %s\n", temp_state == DOOR_OPEN ? "OPENED" : "CLOSED");
            current_state = temp_state;
        }
        else
        {
            // if same, print NO CHANGE
            // printf("DOOR STATE: NO CHANGE\n");
        }

        // goes to sleep
        // TODO: update with correct STOP mode to preserve power
        // printf("goes to sleep\n");
        k_sleep(K_SECONDS(1));
        // Immediately enter low-power mode
        // The system will wake up on door state change interrupt
        /* Enter STOP mode - MCU will wake up on button press */
        // k_sleep(K_FOREVER);
        // printf("wakeup!\n");
    }
}