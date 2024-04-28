#ifndef __MOTOR_H__
#define __MOTOR_H__

#define ROTO_RATIO 44 //编码器线数11*分频系数4
#define REDUTATION_RATIO 30 //电机减速比30

#include "stm32f1xx_hal.h"
//编码器结构体
typedef struct
{
    int encode_old;
    int encode_new;
    float speed;
}ENCODE_TypeDef;
//电机结构体
typedef  struct
{
    float speed;
    float pwm;
}Motor_TypeDef;

extern ENCODE_TypeDef g_encode_left;
extern ENCODE_TypeDef g_encode_right;
extern Motor_TypeDef g_motor_left;
extern Motor_TypeDef g_motor_right;

void run_left_motor(float speed);
void run_right_motor(float speed);
void left_speed_compute(int32_t encode_now,uint8_t ms);
int gtim_get_encode1(void);
int gtim_get_encode2(void);
void motor_circle_left();
void motor_circle_right();
void motor_straight(float speed_data);
void run_motor();

#endif
