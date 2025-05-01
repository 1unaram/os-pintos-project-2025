#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"

struct robot* robots;
int robot_count;
char **robot_tasks;
char **robot_names;

void parse_parameters(char **argv) {

        // 1) Number of robots
        robot_count = atoi(argv[1]);

        // 2) Robot tasks
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
                index++;
                token = strtok_r(NULL, ":", &saveptr);
        }
}

// Print each robot's name and task
void print_robot_info() {
        printf("Robot Info:\n");
        for (int i = 0; i < robot_count; i++) {
                printf("%s -> %s\n", robot_names[i], robot_tasks[i]);
        }
        printf("\n");
}


// test code for robot thread
void test_thread(void* aux){
        int idx = *((int *)aux);
        int test = 0;
        while(1){
                printf("thread %d : %d\n", idx, test++);
                thread_sleep(idx * 1000);
        }
}

// test code for central control node thread
void cnn_thread(){
        while(1){
                print_map(robots, robot_count);
                thread_sleep(1000);
                block_thread();
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this


        printf("\n=========== Start ===========\n\n");
        printf("implement automated warehouse!\n\n");


        // [1] Parse parameters
        parse_parameters(argv);


        // [2] Create and Set robots
        robots = malloc(sizeof(struct robot) * robot_count);
        robot_names = malloc(sizeof(char *) * robot_count);

        for (int i = 0; i < robot_count; i++){
                robot_names[i] = malloc(sizeof(char) * 3);
                snprintf(robot_names[i], sizeof(robot_names), "R%d", i + 1);
                setRobot(&robots[i], robot_names[i], 5, 5, 0, 0);
        }

        // [3] Print robot info
        print_robot_info();

        // [4] Create threads
        // 1) Mallocate threads for ccn and robots
        tid_t* threads = malloc(sizeof(tid_t) * (robot_count + 1));

        // 2) Create thread for cnt
        threads[0] = thread_create("CNT", 0, &cnn_thread, NULL);

        // 3) Create threads for robots
        int idxs[6] = {1, 2, 3, 4, 5, 6};
        for (int i = 0; i < robot_count; i++){
                threads[i + 1] = thread_create(robots[i].name, 0, &test_thread, &idxs[i + 1]);
        }

        printf("=========== End ===========\n\n");
}
