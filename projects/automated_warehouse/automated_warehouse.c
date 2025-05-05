#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"


#define MAX_PATH_LENGTH (MAP_WIDTH * MAP_HEIGHT)

struct robot* robots;
int robot_count;
char **robot_tasks;
char **robot_names;

struct message_box* boxes_from_central_control_node;
struct message_box* boxes_from_robots;

typedef enum { UP, DOWN, LEFT, RIGHT } move_direction;

typedef struct {
        int row;
        int col;
} Location;

Location*** shortest_path_by_bfs;

// typedef struct {

// } Task;


// Initialize the robots
void init_robots() {
        robots = malloc(sizeof(struct robot) * robot_count);
        robot_names = malloc(sizeof(char *) * robot_count);
        for (int i = 0; i < robot_count; i++){
                robot_names[i] = malloc(sizeof(char) * 3);
                snprintf(robot_names[i], sizeof(robot_names), "R%d", i + 1);

                // set robot and place robot at W
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


int check_all_robots_done() {
        for (int i = 0; i < robot_count; i++) {
                if (robots[i].required_payload == 1) {
                        return 0; // Not all robots are done
                }
        }
        return 1; // All robots are done
}



// Print each robot's name and task
void print_robot_info() {
        printf("=========================== * Robots Info * ===========================\n");
        for (int i = 0; i < robot_count; i++) {
                struct robot* r = &robots[i];
                printf("| %s | Position: (%d, %d) | Payload: %d/%d | Load Location: (%d,%d) | Unload Location: (%d,%d)\n",
                        r->name,
                        r->row, r->col,
                        r->current_payload, r->required_payload,
                        r->load_location_row, r->load_location_col,
                        r->unload_location_row, r->unload_location_col);
        }
        printf("=======================================================================\n\n");
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

                        // [2-1] Process the message
                        switch (msg.cmd) {
                                case CMD_MOVE:
                                r->row = msg.target_row;
                                r->col = msg.target_col;
                                r->is_stopped = 0; // 로봇이 이동 중임을 나타냄
                                break;

                                case CMD_LOAD:
                                r->current_payload = 1; // 물건을 적재
                                r->is_stopped = 1;
                                break;

                                case CMD_UNLOAD:
                                r->current_payload = 0; // 물건을 하역
                                r->required_payload = 0; // 물건을 하역했으므로 더 이상 필요 없음
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
        return robots_threads;
}




// Set destination to unload location
void set_destination_to_unload_location() {

        // 하역장 위치를 맵에서 찾아서 설정

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

// Set destination to load location
void set_destination_to_load_location() {

        // 적재 위치를 맵에서 찾아서 설정

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


// Initialize robots tasks
void init_robots_tasks() {
        for (int i = 0; i < robot_count; i++) {
                robots[i].load_location = robot_tasks[i][0]; // Assign load location
                robots[i].unload_location = robot_tasks[i][1]; // Assign unload location
                robots[i].required_payload = 1; // Set required payload
        }
}


void send_command_to_robot(struct robot *r, CommandType cmd, int target_row, int target_col) {
        struct message msg;
        memset(&msg, 0, sizeof(struct message)); // Clear the message structure
        msg.cmd = cmd;
        msg.target_row = target_row;
        msg.target_col = target_col;

        // 메시지 박스에 저장
        boxes_from_central_control_node[r->index].msg = msg;
        boxes_from_central_control_node[r->index].dirtyBit = 1; // 메시지 유효화
}


// Assign next move to the robot
void assign_next_move(struct robot *r, int path_type, int step) {
        Location next_location = shortest_path_by_bfs[r->index][path_type][step];
        send_command_to_robot(r, CMD_MOVE, next_location.row, next_location.col);
}



/* Finding Shortest Path by BFS  */
int bfs(Location start, Location end, Location* path) {

        // BFS 초기화
        int queue_size = MAP_HEIGHT * MAP_WIDTH;
        Location* queue = malloc(sizeof(Location) * queue_size);
        int front = 0, rear = 0;

        // 방문 배열 초기화
        int visited[MAP_HEIGHT][MAP_WIDTH];
        memset(visited, 0, sizeof(visited));

        // 시작 위치 큐에 추가
        queue[rear++] = start;
        visited[start.row][start.col] = 1;

        // 부모 노드 저장
        Location parent[MAP_HEIGHT][MAP_WIDTH];
        for (int i = 0; i < MAP_HEIGHT; i++) {
                for (int j = 0; j < MAP_WIDTH; j++) {
                        parent[i][j] = (Location){-1, -1};
                }
        }

        // BFS 탐색
        while (front < rear) {
                Location current = queue[front++];

                // 목표 위치에 도달했는지 확인
                if (current.row == end.row && current.col == end.col) {
                        int path_length = 0;
                        Location p = end;
                        while (p.row != -1 && p.col != -1) {
                                path[path_length++] = p;
                                p = parent[p.row][p.col];
                        }

                        // 경로를 역순으로 저장
                        for (int i = 0; i < path_length / 2; i++) {
                                Location temp = path[i];
                                path[i] = path[path_length - i - 1];
                                path[path_length - i - 1] = temp;
                        }

                        // 경로 끝에 특별한 값 추가
                        path[path_length] = (Location){-1, -1};

                        free(queue);
                        return path_length;
                }

                // 상하좌우 이동
                int row_vector[] = {-1, 1, 0, 0};
                int col_vector[] = {0, 0, -1, 1};

                for (int i = 0; i < 4; i++) {
                int next_row = current.row + row_vector[i];
                int next_col = current.col + col_vector[i];


                if (next_row >= 0 && next_row < MAP_HEIGHT
                        && next_col >= 0 && next_col < MAP_WIDTH
                        && map_draw_default[next_row][next_col] != 'X'
                        && !visited[next_row][next_col]) {
                        queue[rear++] = (Location){next_row, next_col};
                        visited[next_row][next_col] = 1;
                        parent[next_row][next_col] = current;
                }
                }
        }

        free(queue);
        printf("No path found\n");
        return -1; // 경로를 찾지 못함
}

void find_shortest_path_by_bfs() {

        // [1] Initialize the arrays
        // 최상위 배열 할당 (로봇 수)
        shortest_path_by_bfs = malloc(sizeof(Location**) * robot_count);

        for (int i = 0; i < robot_count; i++) {
            // 각 로봇의 경로 유형 배열 할당 (2: start->load, load->unload)
            shortest_path_by_bfs[i] = malloc(sizeof(Location*) * 2);

            for (int j = 0; j < 2; j++) {
                // 각 경로의 최대 길이 할당
                shortest_path_by_bfs[i][j] = malloc(sizeof(Location) * MAX_PATH_LENGTH);

                // 초기화 (필요 시)
                for (int k = 0; k < MAX_PATH_LENGTH; k++) {
                    shortest_path_by_bfs[i][j][k].row = -1;
                    shortest_path_by_bfs[i][j][k].col = -1;
                }
            }

        }

        // [2] Find the shortest path for each robot
        for (int i = 0; i < robot_count; i++) {
                struct robot *r = &robots[i];

                Location start = {ROW_W, COL_W};
                Location load_location = {r->load_location_row, r->load_location_col};
                Location unload_location = {r->unload_location_row, r->unload_location_col};

                // 경로 계산
                bfs(start, load_location, shortest_path_by_bfs[i][0]);
                bfs(load_location, unload_location, shortest_path_by_bfs[i][1]);
        }

}
/* End for finding shortest path by BFS */


// Check for collision
int check_collision(struct robot *r, int steps[]) {


        // 로봇의 다음 위치
        int next_row = shortest_path_by_bfs[r->index][0][steps[r->index]].row;
        int next_col = shortest_path_by_bfs[r->index][0][steps[r->index]].col;

        // 다른 로봇과의 충돌 검사
        for (int i = 0; i < robot_count; i++) {
                struct robot *other_robot = &robots[i];
                if (i == r->index) continue;

                int other_robot_next_row;
                int other_robot_next_col;

                if (other_robot->is_stopped == 1) {
                        // 다른 로봇이 멈춰있으면 현재 위치로 충돌 검사
                        other_robot_next_row = other_robot->row;
                        other_robot_next_col = other_robot->col;
                } else {
                        other_robot_next_row = shortest_path_by_bfs[i][0][steps[i]].row;
                        other_robot_next_col = shortest_path_by_bfs[i][0][steps[i]].col;
                }

                printf("@@@@@ Robot %s(%d) at (%d, %d) and Robot %s(%d) at (%d, %d) ...\n", r->name, r->priority, next_row, next_col, other_robot->name, other_robot->priority, other_robot_next_row , other_robot_next_col);

                if (r->priority > other_robot->priority && next_row == other_robot_next_row && next_col == other_robot_next_col) {
                        printf("Collision detected between %s and %s at (%d, %d)\n", r->name, other_robot->name, next_row, next_col);
                        return 1; // 충돌 발생
                }


        }
        return 0;  // 충돌 없음
}


// central control node thread
void cnn_thread() {
        // [1] Create and Set robots
        init_robots();

        // [2] Initialize robots tasks
        init_robots_tasks();

        // [3] Set load and unload locations
        set_destination_to_unload_location();
        set_destination_to_load_location();

        // [4] Initialize shortest path
        find_shortest_path_by_bfs();

        // print all paths for debug
        for (int i = 0; i < robot_count; i++) {
            printf("\n\n*******************\nRobot %d:\n", i);
            for (int j = 0; j < 2; j++) {
                printf("Path %d: ", j);
                for (int k = 0; shortest_path_by_bfs[i][j][k].row != -1 && shortest_path_by_bfs[i][j][k].col != -1; k++) {
                    printf("(%d, %d) ", shortest_path_by_bfs[i][j][k].row, shortest_path_by_bfs[i][j][k].col);
                }
                printf("\n");
            }
            printf("*******************\n\n");
        }

        // [5] Print robot info
        print_robot_info();

        // [6] Initialize message boxes
        init_message_boxes();

        // [7] Create robots threads
        init_robots_threads();

        // 각 로봇의 현재 경로 단계
        int steps[robot_count];
        int temp_steps[robot_count];
        memset(steps, 0, sizeof(steps));
        memset(temp_steps, 0, sizeof(temp_steps));

        // 각 로봇의 현재 위치를 저장하는 배열
        int current_robot_positions[robot_count][2];
        memset(current_robot_positions, 0, sizeof(current_robot_positions));

        // [8] Main loop
        while (1) {
                if(check_all_robots_done()) {
                        printf("All robots have completed their tasks.\n");
                        break; // 모든 로봇이 작업을 완료하면 종료
                }


                printf("[ Central control node is running... ]\n");

                print_robot_info();

                // [8-0] 로봇 메시지 처리
                for (int i = 0; i < robot_count; i++) {
                        if (boxes_from_robots[i].dirtyBit) {
                                struct message msg = boxes_from_robots[i].msg;
                                current_robot_positions[i][0] = msg.row;
                                current_robot_positions[i][1] = msg.col;

                                boxes_from_robots[i].dirtyBit = 0; // 메시지 처리 완료
                        }
                }

                // [8-1] step 배열 복사
                for (int i = 0; i < robot_count; i++) {
                        temp_steps[i] = steps[i];
                }

                // [8-1] 로봇 상태 확인
                for (int i = 0; i < robot_count; i++) {
                        struct robot *r = &robots[i];

                        // 로봇이 대기 중
                        if (r->required_payload == 0) {
                                send_command_to_robot(r, CMD_WAIT, r->row, r->col);
                                continue;
                        }

                        // 로봇이 보낸 메시지 처리 완료 후(로봇이 작업 완료 후)
                        if (boxes_from_robots[r->index].dirtyBit != 0) continue;

                        // 적재 위치로 이동 중
                        if (r->current_payload == 0) {
                                Location next_location = shortest_path_by_bfs[r->index][0][steps[i]];

                                // 경로 끝에 도달 -> 적재 작업 수행
                                if (next_location.row == -1 && next_location.col == -1) {
                                        send_command_to_robot(r, CMD_LOAD, r->row, r->col);
                                        steps[i] = 0; // 다음 경로로 초기화
                                        continue;
                                }


                                // 충돌 발생 시 대기
                                if(check_collision(r, temp_steps)) {
                                        printf("%s is waiting due to collision...\n", r->name);
                                        send_command_to_robot(r, CMD_WAIT, r->row, r->col);
                                        continue;
                                }

                                // 경로를 따라 이동
                                assign_next_move(r, 0, steps[i]);
                                steps[i]++;

                        }
                        // 하역 위치로 이동 중
                        else if (r->current_payload == 1) {
                                Location next_location = shortest_path_by_bfs[r->index][1][steps[i]];
                                if (next_location.row == -1 && next_location.col == -1) {
                                        // 경로 끝에 도달 -> 하역 작업 수행
                                        send_command_to_robot(r, CMD_UNLOAD, r->row, r->col);
                                        steps[i] = 0; // 다음 작업을 위해 초기화
                                        continue;
                                }

                                // 충돌 발생 시 대기
                                if(check_collision(r, temp_steps)) {
                                        printf("%s is waiting due to collision...\n", r->name);
                                        send_command_to_robot(r, CMD_WAIT, r->row, r->col);
                                        continue;
                                }

                                // 경로를 따라 이동
                                assign_next_move(r, 1, steps[i]);
                                steps[i]++;
                        }


                }

                print_map(robots, robot_count);

                // [8-2] 모든 로봇 스레드 unblock
                unblock_threads();

                // [8-3] 스텝 증가
                increase_step();

                // [8-3] 스레드 주기 제어
                timer_msleep(1000); // 1s 대기
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
                robot_tasks[index][2] = '\0';
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
