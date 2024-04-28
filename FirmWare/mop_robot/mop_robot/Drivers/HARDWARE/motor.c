#include "motor.h"
#include "tim.h"
#include "stdio.h"
#include "oled.h"
#include "MPU6050.h"
#include "rs100.h"
#include "pid.h"

//结构体定义
ENCODE_TypeDef g_encode_left;
ENCODE_TypeDef g_encode_right;
Motor_TypeDef g_motor_left;
Motor_TypeDef g_motor_right;
//电机启动
void run_left_motor(float speed)
{
  int ccr = (int )speed;  //定义比较值
  uint8_t dir;  //方向变量
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); //编码器启动
  if(ccr > 0) //方向判断
    dir = 1;
  else {
    dir = 0;
    ccr = -ccr;
  }
  if(dir == 1)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  }
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);  //开启PWM通道
  if(ccr < __HAL_TIM_GET_AUTORELOAD(&htim2) - 0x0F) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
  }
}
void run_right_motor(float speed)
{
  int ccr = (int )speed;
  uint8_t dir;
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  if(ccr > 0)
    dir = 1;
  else {
    dir = 0;
    ccr = -ccr;
  }
  if(dir == 1)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
  }
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  if(ccr < __HAL_TIM_GET_AUTORELOAD(&htim2) - 0x0F) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
  }
}
//encoder_now为定时器编码器模式下的计数值,ms代表每隔多少ms进入一次计算公式
void left_speed_compute(int32_t encode_now,uint8_t ms)
{
  uint8_t i=0,j=0;
  float temp = 0.0; //用于后续冒泡排序
  static uint8_t sp_count=0,k=0;  //用于判断隔多长时间进一次计算,静态变量,不会随着函数调用被刷新
  static float speed_array[10]={0.0}; //上面的k为数组的索引值
  if(sp_count == ms)  //当调用函数次数超过50次即每间隔50ms进入一次计数
  {
    g_encode_left.encode_new = encode_now;  //编码器值更新
    g_encode_left.speed = (g_encode_left.encode_new - g_encode_left.encode_old);  //脉冲变化数计算
    //通过脉冲计算速度,公式:(50ms内记录的脉冲数/50ms*60)/减速比/编码器线数/分频系数 = 每分钟的脉冲数/已知常量
    speed_array[k++] = (float)(g_encode_left.speed*((1000/ms)*60.0)/(REDUTATION_RATIO*ROTO_RATIO));
    g_encode_left.encode_old = g_encode_left.encode_new;  //更新编码器计数值
    //冒泡排序法
    if(k==10)
    {
      for(i=10;i>=1;i--)
      {
        for(j=0;j<(i-1);j++) {
          if (speed_array[j] > speed_array[j + 1]) {
            temp = speed_array[j];
            speed_array[j] = speed_array[j + 1];
            speed_array[j + 1] = temp;
          }
        }
      }
      temp = 0.0; //初始化temp
      //去除两边高低数据
      for(i=2;i<8;i++)
      {
        temp += speed_array[i];
      }
      temp = (float)(temp/6);
      /*
       * 一阶低通滤波 Y(n)= qX(n) + (1-q)Y(n-1)
       * 其中 X(n)为本次采样值;Y(n-1)为上次滤波输出值;Y(n)为本次滤波输出值,
       * q 为滤波系数
      */
      g_motor_left.speed = (float)((g_motor_left.speed)*(float)0.52)+((float)0.48*temp);
      //显示当前速度
      unsigned char Tx_Buf[20];
      sprintf((char *)Tx_Buf,"Left:  %.2f",g_motor_left.speed);
      OLED_ShowString(1,2,Tx_Buf,5);
      k = 0;
    }
    sp_count = 0;
  }
  sp_count++;
}
void right_speed_compute(int32_t encode_now,uint8_t ms)
{
  uint8_t i=0,j=0;
  float temp = 0.0; //用于后续冒泡排序
  static uint8_t sp_count2=0,k2=0;  //用于判断隔多长时间进一次计算,静态变量,不会随着函数调用被刷新
  static float speed_array2[10]={0.0}; //上面的k为数组的索引值
  if(sp_count2 == ms)  //当调用函数次数超过50次即每间隔50ms进入一次计数
  {
    g_encode_right.encode_new = encode_now;  //编码器值更新
    g_encode_right.speed = (g_encode_right.encode_new - g_encode_right.encode_old);  //脉冲变化数计算
    //通过脉冲计算速度,公式:(50ms内记录的脉冲数/50ms*60)/减速比/编码器线数/分频系数 = 每分钟的脉冲数/已知常量
    speed_array2[k2++] = (float)(g_encode_right.speed*((1000/ms)*60.0)/(REDUTATION_RATIO*ROTO_RATIO));
    g_encode_right.encode_old = g_encode_right.encode_new;  //更新编码器计数值
    //冒泡排序法
    if(k2==10)
    {
      for(i=10;i>=1;i--)
      {
        for(j=0;j<(i-1);j++) {
          if (speed_array2[j] > speed_array2[j + 1]) {
            temp = speed_array2[j];
            speed_array2[j] = speed_array2[j + 1];
            speed_array2[j + 1] = temp;
          }
        }
      }
      temp = 0.0; //初始化temp
      //去除两边高低数据
      for(i=2;i<8;i++)
      {
        temp += speed_array2[i];
      }
      temp = (float)(temp/6);
      /*
       * 一阶低通滤波 Y(n)= qX(n) + (1-q)Y(n-1)
       * 其中 X(n)为本次采样值;Y(n-1)为上次滤波输出值;Y(n)为本次滤波输出值,
       * q 为滤波系数
      */
      g_motor_right.speed = (float)((g_motor_right.speed)*(float)0.52)+((float)0.48*temp);
      //显示当前速度
      unsigned char Tx_Buf[20];
      sprintf((char *)Tx_Buf,"Right: %.2f",g_motor_right.speed);
      OLED_ShowString(1,4,Tx_Buf,5);
      k2 = 0;
    }
    sp_count2 = 0;
  }
  sp_count2++;
}
volatile int g_tim1_encode_count = 0; //设置溢出计数变量
volatile int g_tim3_encode_count = 0;
int counter = 0;  //设置定时器计数变量
int counter2 = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //判断编码器的溢出方向
  if(htim->Instance == TIM1)
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
      g_tim1_encode_count--;
    } else {
      g_tim1_encode_count++;
    }
  }
  if(htim->Instance == TIM3)
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
      g_tim3_encode_count--;
    }
    else {
      g_tim3_encode_count++;
    }
  }
  //每间隔1ms更新一次脉冲数
  if(htim->Instance == TIM4)
  {
    counter++;
//    counter2++;
    int Left_encode_now = gtim_get_encode1();
    left_speed_compute(Left_encode_now,50);
    int Right_encode_now = gtim_get_encode2();
    right_speed_compute(Right_encode_now,50);
    if(counter % SMAPLSE_PID_SPEED == 0)
    {
      g_motor_left.pwm = increment_PID_Ctrl(&g_speed_pid_left,g_motor_left.speed);  //PID计算
      g_motor_right.pwm = increment_PID_Ctrl(&g_speed_pid_right,g_motor_right.speed);
      //防止PWM溢出
      if(g_motor_left.pwm >= 8000)
      {
        g_motor_left.pwm = 8000;
      }
      else if(g_motor_left.pwm <= -8000)
      {
        g_motor_left.pwm = -8000;
      }
      if(g_motor_right.pwm >= 8000)
      {
        g_motor_right.pwm = 8000;
      }
      else if(g_motor_right.pwm <= -8000)
      {
        g_motor_right.pwm = -8000;
      }
      counter = 0;
    }
    //计算温度
//    if(counter2 % ANGLE_SAMPLE_RATE == 0)
//    {
//      Temp = MPU_Get_Temperature();
//      sprintf((char *)Temp_Data,"TEMP:%.2f",(float )Temp/100);
//      OLED_ShowString(1,0,Temp_Data,sizeof (Temp_Data));
//      counter2 = 0;
//    }
  }
}
//更新脉冲数
int gtim_get_encode1(void)
{
  return ( int32_t )__HAL_TIM_GET_COUNTER(&htim1) + g_tim1_encode_count * 65536;       /* 当前计数值+之前累计编码器的值=总的编码器值 */
}
int gtim_get_encode2(void)
{
  return ( int32_t )__HAL_TIM_GET_COUNTER(&htim3) + g_tim3_encode_count * 65536;       /* 当前计数值+之前累计编码器的值=总的编码器值 */
}
void motor_circle_left()
{
  g_speed_pid_right.SetPoint = 110;
  g_speed_pid_left.SetPoint = -110;
}
void motor_circle_right()
{
  g_speed_pid_right.SetPoint = -110;
  g_speed_pid_left.SetPoint = 110;
}
void motor_straight(float speed_data)
{
  g_speed_pid_right.SetPoint = speed_data;
  g_speed_pid_left.SetPoint = speed_data;
}
void run_motor()
{
  run_right_motor(g_motor_right.pwm); //电机启动
  run_left_motor(g_motor_left.pwm); //电机启动
}

