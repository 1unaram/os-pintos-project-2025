#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"

#include "projects/automated_warehouse/automated_warehouse.h"


struct robot* robots;
int robot_count;
char **robot_tasks;
char **robot_names;

struct message_box* boxes_from_central_control_node;
struct message_box* boxes_from_robots;

Location*** shortest_path_by_bfs;

// Initialize robots
void init_robots() {
        robots = malloc(sizeof(struct robot) * robot_count);
        robot_names = malloc(sizeof(char *) * robot_count);
        for (int i = 0; i < robot_count; i++){
                robot_names[i] = malloc(sizeof(char) * 3);
                snprintf(robot_names[i], sizeof(robot_names), "R%d", i + 1);

                // Set robot and place robot to W
                setRobot(&robots[i], robot_names[i], i, 6, 5, i);
        }
}

// Initialize the message boxes
void init_message_boxes() {
        boxes_from_central_control_node = malloc(sizeof(struct message_box) * robot_count);
        boxes_from_robots = malloc(sizeof(struct message_box) * robot_count);
        for (int i = 0; i < robot_count; i++){
                boxes_from_central_control_node[i].dirtyBit = 0;
                boxes_from_robots[i].dirtyBit = 0;
        }
}

// Check if all robots are done for the tasks
int check_all_robots_done() {
        for (int i = 0; i < robot_count; i++) {
                if (robots[i].required_payload == 1) {
                        // Any robot is still working
                        return 0;
                }
        }
        // All robots are done
        return 1;
}

// Thread for robots
void robot_thread(void *aux) {
        struct robot *r = (struct robot *)aux;

        // Main loop
        while(1){

                // [1] Wait until the robot is unblocked
                block_thread();


                // [2] Check message box
                if (boxes_from_central_control_node[r->index].dirtyBit) {
                        struct message msg = boxes_from_central_control_node[r->index].msg;

                        // [2-1] Process the message
                        switch (msg.cmd) {
                                case CMD_MOVE:
                                r->row = msg.target_row;
                                r->col = msg.target_col;
                                r->is_stopped = 0;
                                break;

                                case CMD_LOAD:
                                r->current_payload = 1;
                                r->is_stopped = 1;
                                break;

                                case CMD_UNLOAD:
                                r->current_payload = 0;
                                r->required_payload = 0;
                                r->is_stopped = 1;
                                break;

                                case CMD_WAIT:
                                r->is_stopped = 1;
                                break;
                        }

                        // [2-2] Reset dirty bit
                        boxes_from_central_control_node[r->index].dirtyBit = 0;

                        // [2-3] Process the message
                        boxes_from_robots[r->index].msg.row = r->row;
                        boxes_from_robots[r->index].msg.col = r->col;
                        boxes_from_robots[r->index].msg.current_payload = r->current_payload;
                        boxes_from_robots[r->index].msg.required_payload = r->required_payload;
                        boxes_from_robots[r->index].msg.is_stopped = r->is_stopped;

                        // [2-4] Set dirty bit
                        boxes_from_robots[r->index].dirtyBit = 1;
                }
        }
}

// Initialize robot threads
void init_robots_threads() {
        tid_t* robots_threads = malloc(sizeof(tid_t) * robot_count);
        for (int i = 0; i < robot_count; i++) {
                robots_threads[i] = thread_create(robots[i].name, 0, &robot_thread, &robots[i]);
        }
}

// Initialize robots tasks
void init_robots_tasks() {
        for (int i = 0; i < robot_count; i++) {
                robots[i].load_location = robot_tasks[i][0];
                robots[i].unload_location = robot_tasks[i][1];
                robots[i].required_payload = 1;
        }
}

// Set destination to load location
void set_destination_to_load_location() {

        for (int i = 0; i < robot_count; i++) {
                struct robot *r = &robots[i];
                int flag = 0;

                for (int j = 0; j < MAP_HEIGHT; j++) {
                        for (int k = 0; k < MAP_WIDTH; k++) {
                                if (flag) break;
                                if (map_draw_default[j][k] == r->load_location) {
                                        r->load_location_row = j;
                                        r->load_location_col = k;

                                        flag = 1;
                                        break;
                                }
                        }
                }
        }
}

// Set destination to unload location
void set_destination_to_unload_location() {

        for (int i = 0; i < robot_count; i++) {
                struct robot *r = &robots[i];
                int flag = 0;

                for (int j = 0; j < MAP_HEIGHT; j++) {
                        for (int k = 0; k < MAP_WIDTH; k++) {
                                if (flag) break;

                                if (map_draw_default[j][k] == r->unload_location) {
                                        r->unload_location_row = j;
                                        r->unload_location_col = k;

                                        flag = 1;
                                        break;
                                }
                        }
                }
        }
}

// Send command to robot
void send_command_to_robot(struct robot *r, CommandType cmd, int target_row, int target_col) {
        struct message msg;
        memset(&msg, 0, sizeof(struct message));

        msg.cmd = cmd;
        msg.target_row = target_row;
        msg.target_col = target_col;

        boxes_from_central_control_node[r->index].msg = msg;
        boxes_from_central_control_node[r->index].dirtyBit = 1;
}

// Assign next move to the robot
void assign_next_move(struct robot *r, int path_type, int step) {
        Location next_location = shortest_path_by_bfs[r->index][path_type][step];
        send_command_to_robot(r, CMD_MOVE, next_location.row, next_location.col);
}

// Finding shortest path by BFS algorithm
int bfs(Location start, Location end, Location* path, struct robot *r) {

        // [1] Initialize the queue
        int queue_size = MAP_HEIGHT * MAP_WIDTH;
        Location* queue = malloc(sizeof(Location) * queue_size);
        int front = 0, rear = 0;

        // [2] Initialize visited array
        int visited[MAP_HEIGHT][MAP_WIDTH];
        memset(visited, 0, sizeof(visited));

        // [3] Push the start location to the queue
        queue[rear++] = start;
        visited[start.row][start.col] = 1;

        // [4] Initialize parent array
        Location parent[MAP_HEIGHT][MAP_WIDTH];
        for (int i = 0; i < MAP_HEIGHT; i++) {
                for (int j = 0; j < MAP_WIDTH; j++) {
                        parent[i][j] = (Location){-1, -1};
                }
        }
        // [5] BFS loop
        while (front < rear) {
                Location current = queue[front++];

                // [5-1] Check if reached to the end location
                if (current.row == end.row && current.col == end.col) {
                        int path_length = 0;
                        Location p = end;
                        while (p.row != -1 && p.col != -1) {
                                path[path_length++] = p;
                                p = parent[p.row][p.col];
                        }

                        // [5-2] Save the path in reverse order
                        for (int i = 0; i < path_length / 2; i++) {
                                Location temp = path[i];
                                path[i] = path[path_length - i - 1];
                                path[path_length - i - 1] = temp;
                        }

                        // [5-3] Mark the end of the path
                        path[path_length] = (Location){-1, -1};

                        free(queue);

                        // [5-4] Return the path length
                        return path_length;
                }

                // [5-4] Find the next locations
                int row_vector[] = {-1, 1, 0, 0};
                int col_vector[] = {0, 0, -1, 1};
                for (int i = 0; i < 4; i++) {
                        int next_row = current.row + row_vector[i];
                        int next_col = current.col + col_vector[i];

                        if (next_row >= 0 && next_row < MAP_HEIGHT
                                && next_col >= 0 && next_col < MAP_WIDTH
                                && map_draw_default[next_row][next_col] != 'X'
                                && (map_draw_default[next_row][next_col] == ' '
                                        || map_draw_default[next_row][next_col] == 'S'
                                        || map_draw_default[next_row][next_col] == r->load_location
                                        || map_draw_default[next_row][next_col] == r->unload_location)
                                && !visited[next_row][next_col]) {

                                        queue[rear++] = (Location){next_row, next_col};
                                        visited[next_row][next_col] = 1;
                                        parent[next_row][next_col] = current;

                        }
                }
        }

        free(queue);

        // [6] No path found
        return -1;
}

// Start finding shortest path by BFS
void find_shortest_path_by_bfs() {

        // [1] Initialize the shortest path array
        shortest_path_by_bfs = malloc(sizeof(Location**) * robot_count);
        for (int i = 0; i < robot_count; i++) {
            shortest_path_by_bfs[i] = malloc(sizeof(Location*) * 2);

            for (int j = 0; j < 2; j++) {
                shortest_path_by_bfs[i][j] = malloc(sizeof(Location) * MAX_PATH_LENGTH);

                for (int k = 0; k < MAX_PATH_LENGTH; k++) {
                    shortest_path_by_bfs[i][j][k].row = -1;
                    shortest_path_by_bfs[i][j][k].col = -1;
                }
            }

        }

        // [2] Find the shortest path for each robot
        for (int i = 0; i < robot_count; i++) {
                struct robot *r = &robots[i];

                Location start = {ROW_S, COL_S};
                Location load_location = {r->load_location_row, r->load_location_col};
                Location unload_location = {r->unload_location_row, r->unload_location_col};

                // [2-1] Find the path from start to load location and from load location to unload location
                int is_found_path_1 = bfs(start, load_location, shortest_path_by_bfs[i][0], r);
                int is_found_path_2 = bfs(load_location, unload_location, shortest_path_by_bfs[i][1], r);

                if (is_found_path_1 == -1 || is_found_path_2 == -1) {
                        printf("*** Error: No path found for robot %s to load/unload location ***\n", r->name);
                        printf("*** Shutting down simulator .. ***\n");
                        shutdown_power_off();
                }

        }

}

// Check for collision between robots
int check_collision(struct robot *r, int *steps, int *is_stopped) {


        int path_type = 0;
        if (r->current_payload == 1) path_type = 1;

        // [1] Assign next location for current robot
        int next_row = shortest_path_by_bfs[r->index][path_type][steps[r->index]].row;
        int next_col = shortest_path_by_bfs[r->index][path_type][steps[r->index]].col;
        if (next_row == -1 && next_col == -1) {
                next_row = r->row;
                next_col = r->col;
        }

        // [2] Check for collision with other robots
        for (int i = 0; i < robot_count; i++) {
                struct robot *other_robot = &robots[i];
                if (i == r->index) continue; // Skip the current robot

                int other_robot_next_row;
                int other_robot_next_col;

                // [2-1] Skip when other robot is done for the task
                if (other_robot->current_payload == 0 && other_robot->required_payload == 0) {
                        return 0;
                }

                // [2-2] If other robot is stopped, assign its current location
                if (is_stopped[i] == 1) {
                        other_robot_next_row = other_robot->row;
                        other_robot_next_col = other_robot->col;
                }
                // [2-3] If other robot is moving, assign its next location
                else {
                        int other_path_type = 0;
                        if (other_robot->current_payload == 1) other_path_type = 1;

                        other_robot_next_row = shortest_path_by_bfs[i][other_path_type][steps[i]].row;
                        other_robot_next_col = shortest_path_by_bfs[i][other_path_type][steps[i]].col;

                        // [2-4] If other robot is at the end of its path, assign its current location
                        if (other_robot_next_row == -1 && other_robot_next_col == -1) {
                                other_robot_next_row = other_robot->row;
                                other_robot_next_col = other_robot->col;
                        }
                }

                // [2-5] Collision occurs if next location of current robot is same as next location of other robot
                if (r->priority > other_robot->priority && next_row == other_robot_next_row && next_col == other_robot_next_col) {
                        return 1;
                }


        }

        // [3] No collision
        return 0;
}

// Print robots current status
void print_robot_current_status(int *current_robot_row, int *current_robot_col,
                                int *current_robot_payload, int *current_robot_required_payload,
                                int *current_is_stopped) {
        printf("\nCurrent Robot Status:\n");
        for (int i = 0; i < robot_count; i++) {
                printf("<Robot %d> Row: %d, Col: %d, Payload: %d, Required Payload: %d, Stopped: %d\n",
                       i + 1, current_robot_row[i], current_robot_col[i],
                       current_robot_payload[i], current_robot_required_payload[i],
                       current_is_stopped[i]);
        }
        printf("\n");
}

// Thread for Central Control Node
void cnn_thread() {

        // [1] Create and Set robots
        init_robots();

        // [2] Initialize robots tasks
        init_robots_tasks();

        // [3] Set load and unload locations
        set_destination_to_load_location();
        set_destination_to_unload_location();

        // [4] Initialize shortest path
        find_shortest_path_by_bfs();

        // [5] Create robots threads
        init_robots_threads();

        // [6] Initialize message boxes
        init_message_boxes();

        // [7] Initialize step arrays
        int steps[robot_count];
        int temp_steps[robot_count];
        memset(steps, 0, sizeof(steps));
        memset(temp_steps, 0, sizeof(temp_steps));

        // [8] Initialize robots current status
        int current_robot_row[robot_count];
        int current_robot_col[robot_count];
        int current_robot_payload[robot_count];
        int current_robot_required_payload[robot_count];
        int current_is_stopped[robot_count];
        memset(current_robot_row, 0, sizeof(current_robot_row));
        memset(current_robot_col, 0, sizeof(current_robot_col));
        memset(current_robot_payload, 0, sizeof(current_robot_payload));
        memset(current_robot_required_payload, 0, sizeof(current_robot_required_payload));
        memset(current_is_stopped, 0, sizeof(current_is_stopped));

        // [9] Main loop
        while (!check_all_robots_done()) {

                // [9-1] Process messages from robots
                for (int i = 0; i < robot_count; i++) {
                        if (boxes_from_robots[i].dirtyBit) {
                                struct message msg = boxes_from_robots[i].msg;

                                // Process the message
                                current_robot_row[i] = msg.row;
                                current_robot_col[i] = msg.col;
                                current_robot_payload[i] = msg.current_payload;
                                current_robot_required_payload[i] = msg.required_payload;
                                current_is_stopped[i] = msg.is_stopped;

                                // Reset the dirty bit
                                boxes_from_robots[i].dirtyBit = 0;
                        }
                }

                // [9-2] Print map
                printf("=====================================\n");
                print_map(robots, robot_count);

                // [9-3] Print robot status
                print_robot_current_status(current_robot_row, current_robot_col,
                                           current_robot_payload, current_robot_required_payload,
                                           current_is_stopped);


                // [9-3] Copy steps to temp_steps for collision check
                for (int i = 0; i < robot_count; i++) {
                        temp_steps[i] = steps[i];
                }

                // [9-4] Process each robot
                for (int i = 0; i < robot_count; i++) {
                        struct robot *r = &robots[i];

                        // [9-4-a] When robot is done for the task then wait
                        if (r->required_payload == 0) {
                                send_command_to_robot(r, CMD_WAIT, r->row, r->col);
                                continue;
                        }

                        // [9-4-b] When robot hasn't send message yet
                        if (boxes_from_robots[r->index].dirtyBit != 0) continue;

                        // [9-4-c] When robot is moving to load location
                        if (r->current_payload == 0) {
                                Location next_location = shortest_path_by_bfs[r->index][0][steps[i]];

                                // When robot is at load location, then load the payload
                                if (next_location.row == -1 && next_location.col == -1) {
                                        send_command_to_robot(r, CMD_LOAD, r->row, r->col);
                                        steps[i] = 1; // set to 1 to move to unload location
                                        continue;
                                }

                                // When collision occurs, then wait
                                if(check_collision(r, temp_steps, current_is_stopped)) {
                                        send_command_to_robot(r, CMD_WAIT, r->row, r->col);
                                        current_is_stopped[i] = 1;
                                        continue;
                                }

                                // Assign next move to the robot
                                assign_next_move(r, 0, steps[i]);
                                current_is_stopped[i] = 0;
                                steps[i]++;

                        }
                        // [9-4-d] When robot is moving to unload location
                        else if (r->current_payload == 1) {
                                Location next_location = shortest_path_by_bfs[r->index][1][steps[i]];

                                // When robot is at unload location, then unload the payload
                                if (next_location.row == -1 && next_location.col == -1) {
                                        send_command_to_robot(r, CMD_UNLOAD, r->row, r->col);
                                        continue;
                                }

                                // When collision occurs, then wait
                                if(check_collision(r, temp_steps, current_is_stopped)) {
                                        send_command_to_robot(r, CMD_WAIT, r->row, r->col);
                                        current_is_stopped[i] = 1;
                                        continue;
                                }

                                // Assign next move to the robot
                                assign_next_move(r, 1, steps[i]);
                                current_is_stopped[i] = 0;
                                steps[i]++;
                        }

                }

                // [9-5] Unblock all threads
                unblock_threads();

                // [9-6] Increase step
                increase_step();

                // [9-3] Timer sleep for 1 second
                timer_msleep(1000);
        }

        // [10] All robots are done for the tasks so shutdown the simulator
        printf("\n\n=====================================\n");
        printf("*** All robots are done for tasks ***\n");
        printf("***   Shutting Down Simulator ..  ***\n");
        printf("=====================================\n\n\n");
        shutdown_power_off();
}


// Parse command line arguments and set variables
void parse_parameters(char **argv) {

        // [1] Set number of robots
        robot_count = atoi(argv[1]);

        // [2] Set robots tasks
        robot_tasks = malloc(sizeof(char *) * robot_count);

        char buffer[256];
        strlcpy(buffer, argv[2], sizeof(buffer));
        buffer[sizeof(buffer) - 1] = '\0';

        char *saveptr;
        char *token = strtok_r(buffer, ":", &saveptr);
        int index = 0;

        while (token && index < robot_count) {
                robot_tasks[index] = malloc(3);
                strlcpy(robot_tasks[index], token, 3);
                robot_tasks[index][2] = '\0';
                token = strtok_r(NULL, ":", &saveptr);
                index++;
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this

        printf("\n\n=====================================\n");
        printf("*** Automated Warehouse Simulator ***\n");
        printf("***        20214234 김하람        ***\n");
        printf("=====================================\n\n\n");

        // [1] Parse parameters
        parse_parameters(argv);

        // [2] Create Central Control Node threads
        thread_create("CNT", 0, &cnn_thread, NULL);
}
