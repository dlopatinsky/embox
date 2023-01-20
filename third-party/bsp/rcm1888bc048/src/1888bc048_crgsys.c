/**
  ******************************************************************************
  * @file    1888bc048_crgsys.c
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    11.11.2019
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the SRGSYS & SRGDDR:
  *           + Initialization and Configuration functions
  *
  * @verbatim
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
#include "1888bc048_crgsys.h"

/**
  * @brief    CRGSYS   .
  * @param  None.
  * @retval None
  */
void CRGSYS_DeInit(void){
  CRGSYS_Init(PLL_PRDIV_INIT, PLL_FBDIV_INIT, PLL_PSDIV_INIT, PLL_CMD_Osc_Use);
}

/**
  * @brief  Initializes the CRGSYS peripheral according to the specified parameters.
  * @param  div_in:    0...31
  * @param  mult:   0...4097
  * @param  div_out:    0...31
  * @retval None
  */  
void CRGSYS_Init(uint8_t div_in, uint16_t mult, uint8_t div_out, CRGSYS_PLL_CMD_TypeDef cmd){
//     :
  CRG_SYS->WR_LOCK = WR_LOCK_DISABLE;
// 1.    
  CRG_SYS->PLL_CTRL.PLL_CMD = PLL_CMD_Osc_Off;
// 2.   
  CRG_SYS->PLL_PRDIV = div_in;
  CRG_SYS->PLL_FBDIV = mult;
  CRG_SYS->PLL_PSDIV = div_out;
// 3.  
  CRG_SYS->PLL_CTRL.PLL_CMD = cmd;
// 4.    
  while(!CRG_SYS->PLL_STAT.PLL_CMD_ACK){}
//    
  CRG_SYS->WR_LOCK = ~WR_LOCK_DISABLE;
}

