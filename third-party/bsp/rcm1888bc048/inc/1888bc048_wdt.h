/**
  ******************************************************************************
  * @file    1888bc048_wdt.h
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    04.12.2019
  * @brief   This file contains all the functions prototypes for the WDT 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_WDT_H
#define __1888BC048_WDT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */

/** @addtogroup 
  * @{
  */

#define WDT_UNLOCK      (0x1ACCE551) /*!<  WDT */
//     ...
#define WDT_50ms        (uint32_t)(5e6)  
#define WDT_100ms       (uint32_t)(10e6)  
#define WDT_400ms       (uint32_t)(40e6) 
#define WDT_1000ms      (uint32_t)(100e6) 
  
/** @defgroup _enumeration 
  * @{
  */

/**
  * @brief WDT_ structure definition  
  */  

/**
  * @brief WDT structure definition  
  */

typedef struct{
  __IO uint32_t Load;       // 0x000  0xFFFFFFFF	 
  __I uint32_t Value;       // 0x004  0xFFFFFFFF	 
  struct{
    __IO uint32_t IntEn:    1;
    __IO uint32_t ResetEn:  1;
    __IO uint32_t :  30;
  }Control;    // 0x008  0x0	 
  __O uint32_t IntClr;      // 0x00C    
  __I uint32_t RIS;         // 0x010 	0x0	     
  __I uint32_t MIS;         // 0x014 	0x0	     
  __I uint8_t res00[0xC00-0x018];// 
  __IO uint32_t Lock;       // 0xC00  0x0	  
  __I uint8_t res01[0xF00-0xC04];// 
  __IO uint32_t ITCR;       // 0xF00  0x0	   
  __O uint32_t ITOP;        // 0xF04  0x0	   
  __I uint8_t res02[0xFE0-0xF08];// 	
  struct{
    __I uint32_t PartNumber:    12; //[11:0]	    .  WDT 0x805
    __I uint32_t DesignerID:    8;  //[19:12]    .  ARM 0x41 (ASCII A)
    __I uint32_t Revision:      4;  //[23:20]     .     
    __I uint32_t Configuration: 8;  //[31:24]	   .  WDT 0x0
  }PeriphID0,  // 0xFE0 0x05	  
  PeriphID1,   // 0xFE4 0x18	  
  PeriphID2,   // 0xFE8 0x14	  
  PeriphID3;   // 0xFEC 0x00	   
  __I uint32_t PCellID0;    // 0xFF0 	0x0D	 PrimeCell
  __I uint32_t PCellID1;    // 0xFF4 	0xF0	 PrimeCell
  __I uint32_t PCellID2;    // 0xFF8 	0x05	 PrimeCell 
  __I uint32_t PCellID3;    // 0xFFC 	0xB1	 PrimeCell 
}WDT_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  

#define WDT              ((WDT_TypeDef *)WDT_BASE)
  
/* Initialization and Configuration functions *********************************/
extern void WDT_Update(uint32_t x);
/* WDT Read and Write functions **********************************************/

/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_WDT_H */
/**
  * @}
  */
  
