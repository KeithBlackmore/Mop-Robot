#include "pid.h"
#include "motor.h"

PID_TypeDef g_speed_pid_left;
PID_TypeDef g_speed_pid_right;

void pid_init_left(void)
{
  g_speed_pid_left.SetPoint = 0;       /* 设定目标值 */
  g_speed_pid_left.ActualValue = 0.0;  /* 期望输出值 */
  g_speed_pid_left.SumError = 0.0;     /* 积分值 */
  g_speed_pid_left.Error = 0.0;        /* Error[1] */
  g_speed_pid_left.LastError = 0.0;    /* Error[-1] */
  g_speed_pid_left.PrevError = 0.0;    /* Error[-2] */
  g_speed_pid_left.Proportion = KPL;    /* 比例常数 Proportional Const */
  g_speed_pid_left.Integral = KIL;      /* 积分常数 Integral Const */
  g_speed_pid_left.Derivative = KDL;    /* 微分常数 Derivative Const */
}
void pid_init_right(void)
{
  g_speed_pid_right.SetPoint = 0;       /* 设定目标值 */
  g_speed_pid_right.ActualValue = 0.0;  /* 期望输出值 */
  g_speed_pid_right.SumError = 0.0;     /* 积分值 */
  g_speed_pid_right.Error = 0.0;        /* Error[1] */
  g_speed_pid_right.LastError = 0.0;    /* Error[-1] */
  g_speed_pid_right.PrevError = 0.0;    /* Error[-2] */
  g_speed_pid_right.Proportion = KPR;    /* 比例常数 Proportional Const */
  g_speed_pid_right.Integral = KIR;      /* 积分常数 Integral Const */
  g_speed_pid_right.Derivative = KDR;    /* 微分常数 Derivative Const */
}
int32_t increment_PID_Ctrl(PID_TypeDef *PID,float Feedback_Vale)
{
  PID->Error = (float )(PID->SetPoint - Feedback_Vale);   //计算偏差(目标值减实际值)
  PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))   //比例项
                      + (PID->Integral * PID->Error)                      //积分项
                      + (PID->Derivative * (PID->Error - PID->LastError));//微分项
  PID->PrevError = PID->LastError;
  PID->LastError = PID->Error;

  return ((int32_t)(PID->ActualValue));
}