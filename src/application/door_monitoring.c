#include "door_monitoring.h"

#include <stdbool.h>
#include <stdio.h>

#include <zephyr/kernel.h>

void door_monitoring_init()
{
    printf("door_monitoring_init\n");

    // init door_state_manager
    printf("init door_state_manager\n");
    // door_state_manager_init();
}

void door_monitoring_run()
{
    printf("door_monitoring_run\n");

    // get initial door state from door_state_manager
    printf("get initial door state from door_state_manager\n");
    // current_state = door_state_manager_get_state();

    // entering forever loop
    printf("entering forever loop\n");
    while (true)
    {
        // after first starting up or wake up
        printf("after first starting up or wake up\n");

        // get door state from door_state_manager
        // temp_state = door_state_manager_get_state();

        // compare with initial door state
        // if (temp_state != current_state)
        // {
            // if different, print door state
        //     printf("DOOR STATE: %s\n", temp_state == OPENED ? "OPENED" : "CLOSED");
        //     current_state = temp_state;
        // }

        // goes to sleep
        printf("goes to sleep\n");
        k_sleep(K_SECONDS(5));
    }
}