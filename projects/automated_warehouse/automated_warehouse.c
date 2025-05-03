#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"

struct robot* robots;
int robot_count;
char **robot_tasks;
char **robot_names;
struct messsage_box* boxes_from_central_control_node;
struct messsage_box* boxes_from_robots;

typedef enum { UP, DOWN, LEFT, RIGHT } move_direction;


// Print each robot's name and task
void print_robot_info(struct robot* robots, int robot_count) {
        printf("==================== * Robots Info * ====================\n");
        for (int i = 0; i < robot_count; i++) {
                printf("| %s | Position: (%d, %d) | Payload: %d/%d | Destination: %c |\n",
                        robots[i].name,
                        robots[i].row, robots[i].col,
                        robots[i].current_payload, robots[i].required_payload,
                        robots[i].required_dock
                );
        }
        printf("=========================================================\n\n");
}


// Move robot
void move_robot(struct robot *r, move_direction dir) {
        switch (dir) {
            case UP:
                if (r->row > 0) r->row--;
                break;
            case DOWN:
                if (r->row < MAP_HEIGHT - 1) r->row++;
                break;
            case LEFT:
                if (r->col > 0) r->col--;
                break;
            case RIGHT:
                if (r->col < MAP_WIDTH - 1) r->col++;
                break;
        }
}


// robot thread
void robot_thread(void *aux){
        struct robot *r = (struct robot *)aux;

        while(1){

                // [1] Wait until the robot is unblocked
                block_thread();

                // [2] Check message box
                if (boxes_from_central_control_node[r->index].dirtyBit) {
                        struct message msg = boxes_from_central_control_node[r->index].msg;
                        printf("Message to %s: Row: %d, Col: %d, Payload: %d/%d, Command: %d\n",
                               r->name, msg.row, msg.col, msg.current_payload,
                               msg.required_payload, msg.cmd);

                        boxes_from_central_control_node[r->index].dirtyBit = 0;
                }

        }
}


// central control node thread
void cnn_thread(){

        // [1] Create and Set robots
        robots = malloc(sizeof(struct robot) * robot_count);
        robot_names = malloc(sizeof(char *) * robot_count);
        for (int i = 0; i < robot_count; i++){
                robot_names[i] = malloc(sizeof(char) * 3);
                snprintf(robot_names[i], sizeof(robot_names), "R%d", i + 1);

                // set robot and place robot at W
                setRobot(&robots[i], robot_names[i], i, 6, 5, 0, robot_tasks[i][0] - '0', robot_tasks[i][1]);
        }

        // [2] Print robot info
        print_robot_info(robots, robot_count);

        // [3] Create message boxes
        boxes_from_central_control_node = malloc(sizeof(struct messsage_box) * robot_count);
        boxes_from_robots = malloc(sizeof(struct messsage_box) * robot_count);
        for (int i = 0; i < robot_count; i++){
                boxes_from_central_control_node[i].dirtyBit = 0;
                boxes_from_robots[i].dirtyBit = 0;
        }

        // [4] Create robots threads
        tid_t* robots_threads = malloc(sizeof(tid_t) * robot_count);
        for (int i = 0; i < robot_count; i++) {
                robots_threads[i] = thread_create(robots[i].name, 0, &robot_thread, &robots[i]);
        }


        while(1){
                printf("[ Central control node is running... ]\n");

                // Check message boxes
                // for (int i = 0; i < robot_count; i++) {
                //         if (boxes_from_robots[i].dirtyBit) {

                //                 struct message msg = boxes_from_robots[i].msg;
                //                 printf("Message from %s: Row: %d, Col: %d, Payload: %d/%d, Command: %d\n",
                //                        robots[i].name, msg.row, msg.col, msg.current_payload,
                //                        msg.required_payload, msg.cmd);
                //                 boxes_from_robots[i].dirtyBit = 0; // Reset dirty bit
                //         }
                // }

                // [5] Move robots
                // for (int i = 0; i < robot_count; i++) {
                //         if (robots[i].current_payload > 0) {
                //                 // Move to the required dock
                //                 if (robots[i].row < ROW_W) {
                //                         move_robot(&robots[i], DOWN);
                //                 } else if (robots[i].col < COL_W) {
                //                         move_robot(&robots[i], RIGHT);
                //                 } else {
                //                         robots[i].current_payload--;
                //                         robots[i].required_payload--;
                //                 }
                //         } else {
                //                 // Move to the source
                //                 if (robots[i].row > ROW_S) {
                //                         move_robot(&robots[i], UP);
                //                 } else if (robots[i].col > COL_S) {
                //                         move_robot(&robots[i], LEFT);
                //                 } else {
                //                         robots[i].current_payload++;
                //                         robots[i].required_payload++;
                //                 }
                //         }
                // }

                print_map(robots, robot_count);
                thread_sleep(1000);
                block_thread();
        }
}


// Parse command line arguments
void parse_parameters(char **argv) {

        // 1) Set number of robots
        robot_count = atoi(argv[1]);

        // 2) Set robot's tasks
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
                robot_tasks[index][sizeof(robot_tasks[index]) - 1] = '\0';
                token = strtok_r(NULL, ":", &saveptr);
                index++;
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this

        printf("implement automated warehouse!\n\n");

        // [1] Parse parameters
        parse_parameters(argv);

        // [2] Create Central Control Node threads
        tid_t thread = malloc(sizeof(tid_t));
        thread = thread_create("CNT", 0, &cnn_thread, NULL);

}
