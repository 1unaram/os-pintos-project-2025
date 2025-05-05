#ifndef _PROJECTS_PROJECT1_ROBOT_H__
#define _PROJECTS_PROJECT1_ROBOT_H__

/**
 * A Structure representing robot
 */
struct robot {
    const char* name;
    int index;
    int row;
    int col;
    int current_payload;
    int required_payload;

    char load_location;
    char unload_location;

    int load_location_row;
    int load_location_col;
    int unload_location_row;
    int unload_location_col;
};

void setRobot(struct robot* _robot, const char* name, int index, int row, int col);

#endif
