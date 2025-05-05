#include "projects/automated_warehouse/robot.h"

/**
 * A function setting up robot structure
 */
void setRobot(struct robot* _robot, const char* name, int index, int row, int col){
    _robot->name = name;
    _robot->index = index;
    _robot->row = row;
    _robot->col = col;
    _robot->current_payload = 0; // Initialize current payload
    _robot->required_payload = 0; // Initialize required payload

    _robot->load_location = -1; // Initialize load location
    _robot->unload_location = -1; // Initialize unload location
    _robot->load_location_row = -1; // Initialize load location row
    _robot->load_location_col = -1; // Initialize load location column
    _robot->unload_location_row = -1; // Initialize unload location row
    _robot->unload_location_col = -1; // Initialize unload location column
}
