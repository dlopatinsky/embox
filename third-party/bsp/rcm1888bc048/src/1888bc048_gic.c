/**
  ******************************************************************************
  * @file    1888bc048_gic.c
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    21.01.2020
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the GIC:
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
#include "1888bc048_gic.h"
#include "1888bc048_gpio.h"
//
void init_dit0_gic(void){

uint8_t temp, cnt = 0;
//     :
  __DI();
  GIC_DIS->Enable = 0;
// -    GICD_ICFGR        :     ;
  GIC_DIS->ICFGR0 = 0x55555555;
//  GIC_DIS->ICFGR1 = 0;
  GICD_ICFGR2 = 0x55555555;
  GICD_ICFGR3 = 0x55555555;
  GICD_ICFGR4 = 0x55555555;
  GICD_ICFGR5 = 0x55555555;
// -    GICD_IGROUPR  0 -                                  0;
  GIC_DIS->IRQ_GROUP0 = 0;
  GIC_DIS->IRQ_GROUP1 = 0;
  GIC_DIS->IRQ_GROUP2 = 0;
// -    GICD_ISENABLER   ,    ;
//  GICD_ISENABLER0 = 0xFFFFFFFF;
//  GICD_ISENABLER1 = 0xFFFFFFFF;
//  GICD_ISENABLER2 = 0xFFFFFFFF;
goto ei;
  GIC_DIS->SET_IRQ.DIT0_TIM1_Int = 1;
// -    GICC_PMR  0xFF -    ;
  GIC_CPU->PMR.Priority = 0xff;
// -    GICC_CTLR  1 -      ;
  GIC_CPU->CTRL.EnableGrp0 = 1;
// -    GICD_CTLR  1 -       ;
ei:
  GIC_DIS->Enable = 1;
// -   IRQ    ARM (   ).
  __EI();
//  GPIO_Write(GPIO0, 0); 
  // while(9){
  // extern int tick;
// //    __DI();
// //    if(GIC_CPU->IAR==42){
      // tick++;
      // GIC_CPU->EOIR = 42;
     
      // cnt++;
      // if((cnt&0x111111)==0x111111)cnt++;
      // temp = (cnt & 0x7) | ((cnt & 0x18)<<3);
      // if(!(cnt&0x20)) temp |= (1<<3);
      // GPIO_Write(GPIO0, temp);

// //    }
// //    __EI();
  // }
}
/**
  * @}
  */
