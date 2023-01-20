/**
  ******************************************************************************
  * @file    1888bc048_uart.c
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    22.11.2019
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the UART peripheral:
  *           + Initialization and Configuration functions
  *           + UART Read and Write functions
  *
  *  @verbatim
  *
    ===========================================================================
                         ##### How to use this driver #####
    ===========================================================================
      [..]
    @endverbatim
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "1888bc048_uart.h"
/**
  * @brief  Reads the specified input port pin.
  * @param  UARTx: where x can be (0 or 1) to select the UART peripheral.
  * @retval The input port pin value.
  */
uint32_t UART_ReadInputData(UART_TypeDef* UARTx){
return *((uint32_t*)&(UARTx->RxD));
}

/**
  * @brief  Fills each UART_InitStruct member with its default value.
  * @param  UART_InitStruct : pointer to a UART_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void UART_StructInit(UART_InitTypeDef* UART_InitStruct){  
/* Reset UART init structure parameters values */
  UART_InitStruct->Div.N_DIV = 0;
  UART_InitStruct->Div.BAUD_DIV  = (uint32_t)(50e6)/115200;
  UART_InitStruct->Ctrl = (UART_CTRL_TypeDef){0};
  
  UART_InitStruct->Ctrl.Enable = 1;
  UART_InitStruct->Ctrl.APB_FIFO = 1;
  UART_InitStruct->Ctrl.POL = 1;
  UART_InitStruct->Ctrl.StopBits = UART_StopBits_1;
  UART_InitStruct->Ctrl.Length = UART_Length_8;
}

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct.
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that
  *         contains the configuration information for the specified UART peripheral.
  * @retval None
  */
void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct){
  UARTx->BDIV = UART_InitStruct->Div;
  UARTx->CTRL = UART_InitStruct->Ctrl;
}

  /**
  * @brief  Writes data to the specified UART data port.
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
void UART_Write(UART_TypeDef* UARTx, uint8_t PortVal){
	UARTx->TxD = PortVal;
}

/**
  * @brief  Writes data to the specified UART data port with FIFO.
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  data: specifies the data vector to be written to the port output data register.
  * @param  num: specifies the number of bytes to be written.
  * @retval None
  */
void UART_SendData(UART_TypeDef *port, uint8_t *data, uint16_t num){
  while(num--){
    port->TxD = *data++;
  }
}

/**
  * @brief     .
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @retval   .
  */
uint32_t UART_GetRxNum(UART_TypeDef* UARTx){return UARTx->FIFO_STATE.RxNum;}

/**
  * @brief       FIFO  .
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  data:    
  * @retval   .
  */
uint32_t UART_ReadData(UART_TypeDef* UARTx, uint8_t *data, uint16_t limit){
uint32_t out = UARTx->FIFO_STATE.RxNum, j;  
  for(j=0; (j<out) && (j<limit);j++) data[j] = UARTx->RxD.Data;
return out;  
}

/**
  * @}
  */
