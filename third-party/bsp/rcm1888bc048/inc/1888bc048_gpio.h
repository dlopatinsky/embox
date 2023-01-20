/**
  ******************************************************************************
  * @file    1888bc048_gpio.h
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    12.11.2019
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_GPIO_H
#define __1888BC048_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
  
/** 
  * @brief General Purpose IO
  */
typedef struct{//    -
  __I uint32_t  ID; 		//  GPIO
  __I uint32_t  VERSION; 	// 
  __IO uint32_t DIR; 		// /
  __IO uint32_t WRITE; 	// 
  __IO uint32_t READ;		// 
  __IO uint32_t ISTATUS; 	//  
  __IO uint32_t MASK; 	//  
  __IO uint32_t SET1; 	//  
  __IO uint32_t SET0;		//  
  __IO uint32_t SWITCH_SOURCE;// 
}GPIO_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */
  
#define GPIO0               ((GPIO_TypeDef *) GPIO0_BASE)
#define GPIO1               ((GPIO_TypeDef *) GPIO1_BASE)
#define GPIO2               ((GPIO_TypeDef *) GPIO2_BASE)

/** @defgroup GPIO_pins_define 
  * @{
  */
#define GPIO_Pin_0                 ((uint8_t)0x01)  /*!< Pin 0 selected    */
#define GPIO_Pin_1                 ((uint8_t)0x02)  /*!< Pin 1 selected    */
#define GPIO_Pin_2                 ((uint8_t)0x04)  /*!< Pin 2 selected    */
#define GPIO_Pin_3                 ((uint8_t)0x08)  /*!< Pin 3 selected    */
#define GPIO_Pin_4                 ((uint8_t)0x10)  /*!< Pin 4 selected    */
#define GPIO_Pin_5                 ((uint8_t)0x20)  /*!< Pin 5 selected    */
#define GPIO_Pin_6                 ((uint8_t)0x40)  /*!< Pin 6 selected    */
#define GPIO_Pin_7                 ((uint8_t)0x80)  /*!< Pin 7 selected    */
#define GPIO_Pin_All               ((uint8_t)0xFF)  /*!< All pins selected */

/** @defgroup GPIO_Exported_Types
  * @{
  */

  #define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIO0) || \
                                    ((PERIPH) == GPIO1) || \
                                    ((PERIPH) == GPIO2))
                                     
/** @defgroup Configuration_Mode_enumeration 
  * @{
  */
typedef enum
{
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode              */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode             */
}GPIOMode_TypeDef;

/** @defgroup Configuration_Mask_enumeration 
  * @{
  */
typedef enum
{
  GPIO_Mask_None  = 0x00, /*!<         */
  GPIO_Mask_Pos   = 0x01, /*!<       */
  GPIO_Mask_Neg  =  0x02,  /*!<        */
  GPIO_Mask_Both  = 0x03,  /*!<      */
}GPIOMask_TypeDef;

/** @defgroup Configuration_Switch_enumeration 
  * @{
  */

typedef enum
{
  GPIO_Switch_IP   = 0x00, /*!<         */
  GPIO_Switch_GPIO  = 0x01, /*!<   GPIO   */
}GPIOSwitch_TypeDef;

/**
  * @brief  GPIO Init structure definition  
  */
typedef struct
{
  uint8_t GPIO_Pin;              	/*!< Specifies the GPIO pins to be configured.
                                       This parameter can be any value of @ref GPIO_pins_define */
                                       
  GPIOMode_TypeDef GPIO_Mode;     	/*!< Specifies the operating mode for the selected pins.
                                       This parameter can be a value of @ref GPIOMode_TypeDef   */

  GPIOSwitch_TypeDef GPIO_Switch;   /*!<    for the selected pins.
                                       This parameter can be a value of @ref GPIOSwitch_TypeDef  */
									   
  GPIOMask_TypeDef GPIO_Mask;		/*!<      for the selected pins.
                                       This parameter can be a value of @ref GPIOMask_TypeDef  */
}GPIO_InitTypeDef;

/* Initialization and Configuration functions *********************************/
void GPIO_Deinit(GPIO_TypeDef* GPIOx);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

/* GPIO Read and Write functions **********************************************/
uint16_t GPIO_ReadIstatus(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint8_t PortVal);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint8_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint8_t GPIO_Pin);
//
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_GPIO_H */
/**
  * @}
  */
  
