/*
 * ROS_port.cpp
 *
 *  Created on: Sep 20, 2023
 *      Author: kch93
 */
#include "ros_port.h"
#include "stm32f4xx_hal.h"
#include "STM32Hardware.h"
#include "ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int8.h"

#include "shooter.h"
#include "shooter_base.h"
#include "baseball_intake.h"

//int run_inter0 = 0;
//
//
//
//geometry_msgs::Point angle;

void angle_callback(const geometry_msgs::Point &msg)
{
	target_hz = msg.linear.x;
	target_ev = msg.linear.y;
	target_length = msg.angular.z;
}
void laji_callback(const std_msgs::Int8 &msg){
	laji_cmd = msg.data;
}

void ori_callback(const std_msgs::Int8 &msg){
	flag = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Point> sub_angle("/cmd_angle", angle_callback);
ros::Subscriber<std_msgs::Int8> sub_ori("/cmd_ori", ori_callback);
ros::Subscriber<std_msgs::Int8> sub_laji("cmd_laji", laji_callback);
ros::Publisher pub_laji("laji_ok", &speed);

void ros_setup(void)
{
    nh.initNode();
    nh.subscribe(sub_angle);
    nh.subscribe(sub_ori);
	nh.subscribe(sub_laji);
	nh.advertise(pub_laji);

}
void ros_loop(void)
{
    nh.spinOnce();
//    interPub();
}
/* UART Communication */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart2.Instance = USART1;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if(huart == &huart2){
    // set velocity 0 before uart reinitialization


  HAL_UART_DeInit(&huart2);
  MX_USART1_UART_Init();
  nh.getHardware()->init();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->flush();
}
