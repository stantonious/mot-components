#pragma once

#include <stdbool.h>

#define MAZE_LEN 14
#define MAZE_HEIGHT 14

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220


#define NORTH_BIT 0x0001
#define SOUTH_BIT 0x0002
#define EAST_BIT 0x0004
#define WEST_BIT 0x0008
#define WALL_01_BIT 0x0010
#define WALL_02_BIT 0x0020
#define WALL_STAT_01_BIT 0x0040
#define WALL_STAT_02_BIT 0x0080

#define WALL_STAT_NONE 0
#define WALL_STAT_IO 1
#define WALL_STAT_HIGH 2
#define WALL_STAT_LOW 3

#define WALL_N  0
#define WALL_E  1
#define WALL_S  2
#define WALL_W  3

#define NORTH_DIR 0
#define EAST_DIR 1
#define SOUTH_DIR 2
#define WEST_DIR 3

#define WALL_WIDTH 6 //3
#define WALL_LENGTH 36 //18

#define MAZE_SCALE CANVAS_WIDTH/WALL_LENGTH

#define STATUS_WIDTH 8
#define STATUS_LENGTH 8

#define ACCEL_MAX 1.
#define GYRO_MAX 300.

void get_next_cell(int x_from, int y_from, int *x_to, int *y_to, int dir);
bool can_move(int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int x_from, int y_from,int *x_to, int *y_to, int dir,bool is_jump,bool is_duck);
void get_pos_from_cell(int x_cell, int y_cell, int dir, int wall_length,int wall_width,int *x_pos, int *y_pos);
void get_status_pos_from_cell(int x_cell, int y_cell, int dir, int *x_pos, int *y_pos,int x_map_center,int y_map_center);
void get_static_status_pos_from_cell(int x_cell, int y_cell, int wall_length,int wall_width,int status_length, int status_width, int *x_pos, int *y_pos);
int translate_walls(int wall, int dir);
int translate_wall(int wall, int dir);
void translate(int from_x, int from_y, int *to_x, int *to_y, int dir, int max_x, int max_y);
void translate_pos(int from_x, int from_y, int *to_x, int *to_y, int dir, int max_x, int max_y);
float scale_gyro(float g);
float scale_acc(float g);