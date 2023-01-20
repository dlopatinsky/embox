/**
  ******************************************************************************
  * @file    1888bc048_wdt.c
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    04.12.2019
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the WDT:
  *           .......
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
#include "1888bc048_wdt.h"

/**
  * @brief    .
  * @param  x:  .
  * @retval None
  */
void WDT_Update(uint32_t x){
  WDT->Lock = WDT_UNLOCK;
  WDT->Load = x;
  WDT->Lock = ~WDT_UNLOCK;  
}

/**
  * @}
  */
