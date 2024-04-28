#ifndef PID_H
#define PID_H

#include "stm32f1xx.h"

#define KPL    12.50f
#define KIL    0.35f
#define KDL    0.05f
#define KPR    14.50f
#define KIR    0.38f
#define KDR    0.08f
#define SMAPLSE_PID_SPEED 50  //采样周期

typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* 当前误差 */
    __IO float  LastError;           /* 前一个误差 */
    __IO float  PrevError;           /* 前两个误差 */
}PID_TypeDef;

extern PID_TypeDef g_speed_pid_left;
extern PID_TypeDef g_speed_pid_right;

void pid_init_left(void);
void pid_init_right(void);
int32_t increment_PID_Ctrl(PID_TypeDef *PID,float Feedback_Vale);

#endif
