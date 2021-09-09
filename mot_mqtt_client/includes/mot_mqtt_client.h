
#ifndef _MOT_CLIENT2_H
#define _MOT_CLIENT2_H

#include "freertos/task.h"

#define OP_ID_LEN 16 
TaskHandle_t MotMqttClientHandle;

extern int8_t op_x,op_y,op_t;
extern char op_id[OP_ID_LEN];
void mot_mqtt_client_init(int game_id,char* player_id);
void send_position(int x, int y, int t,unsigned time);
void send_sample(float **a_samples,float **g_samples,int a_size,int g_size,int type,unsigned time);
void send_stats(unsigned maze_id , unsigned session,unsigned step_count, unsigned capture_count, unsigned caught_count, unsigned time );

#endif
