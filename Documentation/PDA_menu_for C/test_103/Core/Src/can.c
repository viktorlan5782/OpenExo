/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
extern uint8_t rx_buffer[8];
extern int8_t READ_FLAG;
extern uint16_t can_id;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 24;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
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

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Can_Config(void) {
    CAN_FilterTypeDef  CAN_FilterType;
    CAN_FilterType.FilterBank=0;
    CAN_FilterType.FilterIdHigh=0x0000;
    CAN_FilterType.FilterIdLow=0x0000;
    CAN_FilterType.FilterMaskIdHigh=0x0000;
    CAN_FilterType.FilterMaskIdLow=0x0000;
    CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
    CAN_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation=ENABLE;
    CAN_FilterType.SlaveStartFilterBank=14;
    if(HAL_CAN_ConfigFilter(&SERVO_CAN,&CAN_FilterType)!=HAL_OK)
    {
        Error_Handler();
    }
if(HAL_CAN_ActivateNotification(&SERVO_CAN,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_Start(&SERVO_CAN)!=HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t Can_Send_Msg(uint32_t id,uint8_t len,uint8_t *data) {
    uint32_t i=0;
    static  uint32_t  TxMailbox;
    CAN_TxHeaderTypeDef  CAN_TxHeader;
    HAL_StatusTypeDef  HAL_RetVal;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.StdId = id;
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    while(HAL_CAN_GetTxMailboxesFreeLevel(&SERVO_CAN) == 0)
    {
       i++;
       if(i>0xffffe)
            return 1;
    }
    HAL_RetVal = HAL_CAN_AddTxMessage(&SERVO_CAN,&CAN_TxHeader,data,&TxMailbox);

    if(HAL_RetVal != HAL_OK)
        return  1;
    return  0;
}
void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *SERVO_CAN)
{
    CAN_RxHeaderTypeDef  hCAN1_RxHeader;

    if(HAL_CAN_GetRxMessage(SERVO_CAN, CAN_RX_FIFO0, &hCAN1_RxHeader, rx_buffer) == HAL_OK)
    {
    can_id = hCAN1_RxHeader.StdId;
    READ_FLAG = 1;
    }
}
/* USER CODE END 1 */
