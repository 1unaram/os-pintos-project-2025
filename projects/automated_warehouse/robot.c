#include "projects/automated_warehouse/robot.h"

/**
 * A function setting up robot structure
 */
void setRobot(struct robot* _robot, const char* name, int index, int row, int col, int priority) {
    _robot->name = name;
    _robot->index = index;
    _robot->row = row;
    _robot->col = col;
    _robot->current_payload = 0;
    _robot->required_payload = 0;

    _robot->load_location = -1;
    _robot->unload_location = -1;
    _robot->load_location_row = -1;
    _robot->load_location_col = -1;
    _robot->unload_location_row = -1;
    _robot->unload_location_col = -1;

    _robot->priority = priority;
    _robot->is_stopped = 0;
}
