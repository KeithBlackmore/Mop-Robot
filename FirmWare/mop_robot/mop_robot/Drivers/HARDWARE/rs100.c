#include "rs100.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"
uint8_t Rx_Buf[2] = {0};  //接收距离缓存区
uint8_t Tx_data = 0x55; //测距指令
uint64_t HData,LData; //接收距离数据
uint64_t Data;
unsigned char Show[20]; //距离显示数组缓存区间
unsigned char Show2[20] = "           "; //清除数据
uint8_t Rx_Flag;  //接收完成标志位
void RS100_Read()
{
  HAL_UART_Transmit(&huart2,&Tx_data,1,100);  //发送指令
  UART_Start_Receive_IT(&huart2,Rx_Buf,2);  //接收数据
  //数据整合
  if(Rx_Flag == 1) {
    Data = (HData << 8) + LData;
    sprintf((char *) Show, "dis:%d", Data);
    OLED_ShowString(1, 6, Show, sizeof(Show));
    Rx_Flag = 0;
  }
}
//接收数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    HData = Rx_Buf[0];
    LData = Rx_Buf[1];
    Rx_Flag = 1;
    HAL_UART_Receive_IT(&huart2,Rx_Buf,2);
    OLED_ShowString(1, 6, Show2, sizeof(Show2));
  }
}
