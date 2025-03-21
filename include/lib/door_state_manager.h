#ifndef DOOR_STATE_MANAGER_H
#define DOOR_STATE_MANAGER_H

typedef enum {
    DOOR_CLOSED = 0,
    DOOR_OPEN = 1,
    DOOR_UNKNOWN = 2,
} door_state_t;

void door_state_manager_init(void);
door_state_t door_state_manager_get_state(void);

#endif // DOOR_STATE_MANAGER_H