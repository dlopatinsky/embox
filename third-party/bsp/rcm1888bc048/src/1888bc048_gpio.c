/**
  ******************************************************************************
  * @file    1888bc048_gpio.c
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    07.11.2019
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the GPIO peripheral:
  *           + Initialization and Configuration functions
  *           + GPIO Read and Write functions
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
#include "1888bc048_gpio.h"
/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (0, 1, or 2) to select the GPIO peripheral.
  * @retval The input port pin value.
  */
uint8_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx){
	  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
return (uint8_t)(GPIOx->READ);
}

/**
  * @brief      .
  * @param  GPIOx: where x can be (0, 1, or 2) to select the GPIO peripheral.
  * @retval None
  */
void GPIO_Deinit(GPIO_TypeDef* GPIOx){
	  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	GPIOx->DIR = 0;
	GPIOx->WRITE = 0;
	GPIOx->SET1 = 0;
	GPIOx->SET0 = 0;
	GPIOx->MASK = 0;
	GPIOx->SWITCH_SOURCE = 0;
	GPIOx->ISTATUS;
}

/**
  * @brief  Fills each GPIO_InitStruct member with its default value.
  * @param  GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct){  
/* Reset GPIO init structure parameters values */
  GPIO_InitStruct->GPIO_Pin  = GPIO_Pin_All;
  GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct->GPIO_Switch = GPIO_Switch_IP;    
  GPIO_InitStruct->GPIO_Mask = GPIO_Mask_None;  
}

/**
  * @brief  Initializes the GPIOx peripheral according to the specified
  *         parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (0..1) to select the GPIO peripheral.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that
  *         contains the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct){
uint8_t dir = GPIOx->DIR, sswitch = GPIOx->SWITCH_SOURCE, j;
uint32_t mask = GPIOx->MASK;
	for(j=0;j<8;j++){
		if(GPIO_InitStruct->GPIO_Pin & (1<<j)){
			dir &= ~(1<<j);
			sswitch &= ~(1<<j);
			mask &= ~(0xF<<(j*4));
			dir |= (GPIO_InitStruct->GPIO_Mode<<j);
			sswitch |= (GPIO_InitStruct->GPIO_Switch<<j);
			mask |= (GPIO_InitStruct->GPIO_Mask<<(4*j));
		}
	}
	GPIOx->DIR = dir;
	GPIOx->SWITCH_SOURCE = sswitch;
	GPIOx->MASK = mask;
}

/**
  * @brief      .
  * @param  GPIOx: where x can be (0..1) to select the GPIO peripheral.
  * @retval None
  */
uint16_t GPIO_ReadIstatus(GPIO_TypeDef* GPIOx){
	  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
return (uint16_t)(GPIOx->ISTATUS);
}

  /**
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (0..2) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
void GPIO_Write(GPIO_TypeDef* GPIOx, uint8_t PortVal){
	GPIOx->WRITE = PortVal;
}

  /**
  * @brief    GPIO data port.
  * @param  GPIOx: where x can be (0..2) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the pattern to be written to the port output data register.
  * @retval None
  */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint8_t GPIO_Pin){
	GPIOx->SET0 = GPIO_Pin;
	GPIOx->SET1 = GPIO_Pin;	
}

  /**
  * @brief    GPIO data port.
  * @param  GPIOx: where x can be (0..2) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the pattern to be written to the port output data register.
  * @retval None
  */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint8_t GPIO_Pin){
	GPIOx->SET1 = ~GPIO_Pin;
	GPIOx->SET0 = ~GPIO_Pin;	
}

/**
  * @}
  */
