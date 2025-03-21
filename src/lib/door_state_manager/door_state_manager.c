#include "door_state_manager.h"

#include <stdio.h>

void door_state_manager_init(void)
{
    printf("door_state_manager_init\n");
}

door_state_t door_state_manager_get_state(void)
{
    return DOOR_CLOSED;
}