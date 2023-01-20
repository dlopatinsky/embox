/**
  ******************************************************************************
  * @file    1888bc048_tim.h
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    24.01.2020
  * @brief   This file contains all the functions prototypes for the TIM 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_TIM_H
#define __1888BC048_TIM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
  
/** @defgroup TIM_STATE 
  * @{
  */
typedef enum
{
  TIM_State_Off           = 0, /*!<  */
  TIM_State_Single        = 2, /*!<   */
  TIM_State_Period        = 3, /*!<   */
  TIM_State_FRC_Start     = 4, /*!<     Free Run Counter */
  TIM_State_GC_Start      = 5, /*!<     Global Counter */  
}TIMState_TypeDef;

/** 
  * @brief   gp_global_timers
  */
typedef struct{
//   
  __I uint32_t  GT_ID;            /*!<   */
  struct{
    __I uint32_t subversion:  8;  /*!<   */
    __I uint32_t version:     8;  /*!<   */
    __I uint32_t frequency:   16; /*!<   */    
  }GT_VER;       // 
//    
  __I uint32_t  FREE_RUN_L;       /*!<  Free Run Counter [31:0] */
  __I uint32_t  FREE_RUN_H;       /*!<  Free Run Counter [63:32] */
  __I uint32_t  GLOBAL_NS;        /*!<    .  999999990 */ 
  __I uint32_t  GLOBAL_S_L;       /*!<    System Counter [31:0] */
  __I uint32_t  GLOBAL_S_H;       /*!<    System Counter [48:32] */
// 
  __IO uint32_t SW_RST;           /*!<   -   */
  __IO uint32_t FIX_CMD;          /*!<     -   */
  struct{
    __IO uint32_t x10ps:      8;  /*!<     */
    __IO uint32_t ns:         5;  /*!<    */    
    __IO uint32_t adj_10ps:   8;  /*!<       */
    __IO uint32_t adj_ns:     8;  /*!<    */
  }INC_NS;                        /*!<    Global Counter   .?10 */
  __IO uint32_t GLOBAL_ADJUST;    /*!<     */
  __IO uint32_t MOVE_NS;          /*!<     */ 
  __IO uint32_t MOVE_S;           /*!<     */ 
  __IO uint32_t SET_GLOBAL_NS;    /*!<     */ 
  __IO uint32_t SET_GLOBAL_S_L;   /*!<     Global Counter [31:0] */
  __IO uint32_t SET_GLOBAL_S_H;   /*!<     Global Counter [48:32] */
  __IO uint32_t Enable;           /*!<    gp_global_timers -   */
}GTIM_TypeDef;

/** 
  * @brief  
  */
typedef struct{
  struct{
    __IO uint32_t mode:       3;  /*!< : . TIMState_TypeDef */
    __IO uint32_t :           13;
    __IO uint32_t active:     16; /*!< ,     tmr_events_o    1 */
  }STATE; //     0
  __IO uint32_t Limit;            /*!<    0 ( ) */
  __IO uint32_t LIM_CNT_NS;       /*!<  Free Run Counter  Global Counter,       0. */
  __IO uint32_t LIM_CNT_S_L;      /*!<    Free Run Counter    TMR_0_LIM_CNT_NS (  32 )  TMR_0_LIM_CNT_S_L (  32 ). */
  __IO uint32_t LIM_CNT_S_H;      /*!<   TMR_0_LIM_CNT_S_H      */
  __I uint32_t res0[3];
}TIM_TypeDef;  

/** @defgroup TIM_SIZE 
  * @{
  */
typedef enum
{
  TIM_Size_16_bits       = 0,     /*!< 16-  */
  TIM_Size_32_bits       = 1,     /*!< 32-  */
}TIMSize_TypeDef;

/** @defgroup TIM_SHOT 
  * @{
  */
typedef enum
{
  TIM_Shot_Ever        = 0,       /*!<   ( ) */
  TIM_Shot_Single      = 1,       /*!< 16-  */
}TIMShot_TypeDef;

/** @defgroup TIM_DIV 
  * @{
  */
typedef enum
{
  TIM_Div_None        = 0,        /*!<    ( ) */
  TIM_Div_16          = 1,        /*!<    16 */
  TIM_Div_256         = 2,        /*!<    256 */
}TIMDiv_TypeDef;

/** @defgroup TIM_MODE 
  * @{
  */
typedef enum
{
  TIM_Mode_FreeRun    = 0,        /*!< Timer X      ( ) */
  TIM_Mode_Periodic   = 1,        /*!< Timer X     */
}TIMMode_TypeDef;

/** 
  * @brief    
  */
typedef  struct{
  __IO uint32_t OneShot:    1;  /*!<      : 0 =   ( ), 1 =   */
  __IO uint32_t TimSize:    1;  /*!<   : 0 = 16-  ( ), 1 = 32-  */
  __IO uint32_t TimDiv:     2;  /*!<     ( ): 00 =    ( ), 01 =    16, 10 =    256, 11 =   */
  __I  uint32_t:            1;
  __IO uint32_t IntEnable:  1;  /*!<   : 0 =  , 1 =   ( ) */
  __IO uint32_t TimMode:    1;  /*!<  : 0 = Timer X      ( ), 1 = Timer X     */
  __IO uint32_t Active:     1;  /*!<  : 0 = Timer X  ( ), 1 = Timer X  */
  __I  uint32_t:            24;
}TimXControl;    

/** 
  * @brief   DIT
  */
typedef struct{
  __IO uint32_t Tim1Load;     /*!<  . */ 
  __I  uint32_t Tim1Value;    /*!<   . */
  TimXControl Tim1Control;    /*!<  . */ 
  __O  uint32_t Tim1IntClr;   /*!<    -  . */ 
  __I  uint32_t Tim1RIS;      /*!<       -   */
  __I  uint32_t Tim1MIS;      /*!<       -   */
  __I  uint32_t  resX;        /*!<     */
  __IO uint32_t Tim1BGLoad;   /*!<   . */ 
  __IO uint32_t Tim2Load;     /*!<  . */ 
  __I  uint32_t Tim2Value;    /*!<   . */
  TimXControl Tim2Control;    /*!<  . */ 
  __O  uint32_t Tim2IntClr;   /*!<   . */ 
  __I  uint32_t Tim2RIS;      /*!<      . */
  __I  uint32_t Tim2MIS;      /*!<      . */
  __IO uint32_t Tim2BGLoad;   /*!<   . */ 
  __I  uint32_t  res0[945];
  __IO uint32_t TimITCR;      /*!<     -  . */ 
  __O  uint32_t TimITOP;      /*!<    . */ 
  __I  uint32_t res1[54];
//   ,      ...
  __I uint32_t TimPeriphID0;  /*!<   . TimerPeriphID0  [7:0] */
  __I uint32_t TimPeriphID1;  /*!<   . TimerPeriphID1  [15:8] */
  __I uint32_t TimPeriphID2;  /*!<   . TimerPeriphID2  [23:16] */
  __I uint32_t TimPeriphID3;  /*!<   . TimerPeriphID3  [31:24] */
//           ...
  __I uint32_t TimPCellID0;   /*!< PrimeCell . TimerPCellID0  [7:0] */
  __I uint32_t TimPCellID1;   /*!< PrimeCell . TimerPCellID1  [15:8] */
  __I uint32_t TimPCellID2;   /*!< PrimeCell . TimerPCellID2  [23:16] */
  __I uint32_t TimPCellID3;   /*!< PrimeCell . TimerPCellID3  [31:24] */
}DIT_TypeDef;
  
/** @addtogroup Peripheral_declaration
  * @{
  */
#define GTIM        ((GTIM_TypeDef *)GTIMERS_BASE)
#define TIM0        ((TIM_TypeDef *)(GTIMERS_BASE+0x400))
#define TIM1        ((TIM_TypeDef *)(GTIMERS_BASE+0x420))
#define TIM2        ((TIM_TypeDef *)(GTIMERS_BASE+0x440))
#define TIM3        ((TIM_TypeDef *)(GTIMERS_BASE+0x460))
#define TIM4        ((TIM_TypeDef *)(GTIMERS_BASE+0x480))
#define TIM5        ((TIM_TypeDef *)(GTIMERS_BASE+0x4A0))
#define TIM6        ((TIM_TypeDef *)(GTIMERS_BASE+0x4C0))
#define TIM7        ((TIM_TypeDef *)(GTIMERS_BASE+0x4E0))
#define TIM8        ((TIM_TypeDef *)(GTIMERS_BASE+0x500))
#define TIM9        ((TIM_TypeDef *)(GTIMERS_BASE+0x520))
//
#define DIT0        ((DIT_TypeDef *)DIT0_BASE)
#define DIT1        ((DIT_TypeDef *)DIT1_BASE)
#define DIT2        ((DIT_TypeDef *)DIT2_BASE)
#define DIT3        ((DIT_TypeDef *)DIT3_BASE)

/**
  * @brief  TIM structure definition  
  */

/* Initialization and Configuration functions *********************************/

/* GPIO Read and Write functions **********************************************/
//
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_TIM_H */
/**
  * @}
  */
  
