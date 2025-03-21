#include "door_monitoring.h"

#include <stdbool.h>
#include <stdio.h>

#include <zephyr/kernel.h>

void door_monitoring_init()
{
    printf("door_monitoring_init\n");

    // init door_state_manager
    printf("init door_state_manager\n");
}

void door_monitoring_run()
{
    printf("door_monitoring_run\n");

    // get initial door state from door_state_manager
    printf("get initial door state from door_state_manager\n");

    // save initial door state in a variable
    printf("save initial door state in a variable\n");

    // entering forever loop
    printf("entering forever loop\n");
    while (true)
    {
        // after first starting up or wake up
        printf("after first starting up or wake up\n");

        // get door state from door_state_manager
        // compare with initial door state
        // if different, print door state
        printf("after doing door state check\n");

        // goes to sleep
        printf("goes to sleep\n");
        k_sleep(K_SECONDS(5));
    }
}