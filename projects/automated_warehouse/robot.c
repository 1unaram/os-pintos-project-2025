#include "projects/automated_warehouse/robot.h"

/**
 * A function setting up robot structure
 */
void setRobot(struct robot* _robot, const char* name, int index, int row, int col, int current_payload, int required_payload, char required_dock){
    _robot->name = name;
    _robot->index = index;
    _robot->row = row;
    _robot->col = col;
    _robot->current_payload = current_payload;
    _robot->required_payload = required_payload;
    _robot->required_dock = required_dock;

}
