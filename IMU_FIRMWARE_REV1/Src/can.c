/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
CAN_TxHeaderTypeDef packetHeader;
CAN_FilterTypeDef canFilterConfigHeader;
CAN_RxHeaderTypeDef canReceivedMessageHeader0;
CAN_RxHeaderTypeDef canReceivedMessageHeader1;
uint32_t packetMailbox;
uint8_t dataPacket[8];
uint8_t canReceivedMessageData0[8];
uint8_t canReceivedMessageData1[8];
CAN_TxHeaderTypeDef header;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void CAN_send(int ID, int16_t firstInt, int16_t secondInt, int16_t thirdInt, int16_t fourthInt, uint8_t dlc_value)
{
	uint32_t mailbox;
	
	header.StdId = ID;
	header.RTR = CAN_RTR_DATA;
	header.IDE = CAN_ID_STD;
	header.DLC = dlc_value;
	
	dataPacket[0] = (uint8_t)((firstInt >> 8) & 0x00FF);
  dataPacket[1] = (uint8_t)(firstInt & 0x00FF);
  dataPacket[2] = (uint8_t)((secondInt >> 8) & 0x00FF);
  dataPacket[3] = (uint8_t)(secondInt & 0x00FF);
	dataPacket[4] = (uint8_t)((thirdInt >> 8) & 0x00FF);
  dataPacket[5] = (uint8_t)(thirdInt & 0x00FF);
  dataPacket[6] = (uint8_t)((fourthInt >> 8) & 0x00FF);
  dataPacket[7] = (uint8_t)(fourthInt & 0x00FF);
	
  HAL_CAN_AddTxMessage(&hcan1, &header, dataPacket, &mailbox);
}


/* Motorola e intel spiegati bene
big endian.........motorola.........most significant byte smallest memory address
little endian......intel............least significant byte smallest memory address
*/
void CAN_send_intel (uint8_t* myArray, uint16_t ID, uint8_t dlc_value)
{
	uint32_t mailbox;

	header.StdId = ID;
	header.RTR = CAN_RTR_DATA;
	header.IDE = CAN_ID_STD;
	header.DLC = dlc_value;

	HAL_CAN_AddTxMessage(&hcan1, &header, myArray, &mailbox);
}

void CAN_send_motorola (uint8_t* myArray, uint16_t ID, uint8_t dlc_value)
{
	uint32_t mailbox;
	uint8_t counter;
	uint8_t intelArray[8];

	header.StdId = ID;
	header.RTR = CAN_RTR_DATA;
	header.IDE = CAN_ID_STD;
	header.DLC = dlc_value;

	for (counter=0; counter<8; counter++)
	{
		intelArray[counter]=myArray[7-counter];
	}

	HAL_CAN_AddTxMessage(&hcan1, &header, intelArray, &mailbox);
}

extern void canStart(void)
{
	canFilterConfig();
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
	return;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canReceivedMessageHeader0, canReceivedMessageData0);
	
	return;
}

static void canFilterConfig(void)
{
	canFilterConfigHeader.FilterBank = 0;
  canFilterConfigHeader.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfigHeader.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfigHeader.FilterIdHigh = (0x000 << 5);
  canFilterConfigHeader.FilterIdLow = 0x0000;
  canFilterConfigHeader.FilterMaskIdHigh = (0x000 << 5);
  canFilterConfigHeader.FilterMaskIdLow = 0x0000;
	canFilterConfigHeader.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfigHeader.FilterActivation = ENABLE;	
  canFilterConfigHeader.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &canFilterConfigHeader);
	return;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
