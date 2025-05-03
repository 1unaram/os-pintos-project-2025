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
    char required_dock;
};

void setRobot(struct robot* _robot, const char* name, int index, int row, int col, int current_payload, int required_payload, char required_dock);

#endif
