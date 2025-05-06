#ifndef __PROJECTS_AUTOMATED_WAREHOUSE_H__
#define __PROJECTS_AUTOMATED_WAREHOUSE_H__

#define MAX_PATH_LENGTH (MAP_WIDTH * MAP_HEIGHT)

typedef enum { UP, DOWN, LEFT, RIGHT } move_direction;

typedef struct {
        int row;
        int col;
} Location;

typedef enum {
    CMD_MOVE,
    CMD_LOAD,
    CMD_UNLOAD,
    CMD_WAIT
} CommandType;

void run_automated_warehouse(char **argv);

void init_robots();
void init_message_boxes();
int check_all_robots_done();
void init_robots_threads();
void init_robots_tasks();
void set_destination_to_load_location();
void set_destination_to_unload_location();
void send_command_to_robot(struct robot *r, CommandType cmd, int target_row, int target_col);
void assign_next_move(struct robot *r, int path_type, int step);
int bfs(Location start, Location end, Location* path, struct robot *r);
void find_shortest_path_by_bfs();
int check_collision(struct robot *r, int *steps, int *is_stopped);
void print_robot_current_status(int *current_robot_row, int *current_robot_col,
    int *current_robot_payload, int *current_robot_required_payload,
    int *current_is_stopped);
void cnn_thread();

#endif
