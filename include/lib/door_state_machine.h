#ifndef DOOR_STATE_MACHINE_H
#define DOOR_STATE_MACHINE_H

#include <stdint.h>

typedef enum
{
    STATE_INIT,
    STATE_DOOR_CLOSED,
    STATE_DOOR_OPEN,
    STATE_TRIGGERED_BY_MOTION_SENSOR,
    STATE_TRIGGERED_BY_LIGHT_SENSOR,
    STATE_CONFIRM_WITH_DISTANCE_SENSOR,
    STATE_UNKNOWN
} door_state_machine_t;

typedef enum
{
    EVENT_POWER_ON,
    EVENT_DOOR_OPEN_DETECTED,
    EVENT_DOOR_CLOSE_DETECTED,
    EVENT_MOTION_SENSOR_INTERRUPT,
    EVENT_LIGHT_SENSOR_INTERRUPT
} door_event_t;

int door_state_machine_init(void);
int door_state_machine_process_event(door_event_t event);
door_state_machine_t door_state_machine_get_state(void);
int door_state_machine_register_callback(void (*callback)(door_state_machine_t old_state, door_state_machine_t new_state));

#endif // DOOR_STATE_MACHINE_H