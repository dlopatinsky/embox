/**
  ******************************************************************************
  * @file    1888bc048_crgsys.h
  * @author  A. Maksimov
  * @version V1.0.1
  * @date    22.11.2019
  * @brief   This file contains all the functions prototypes for the CRGSYS firmware 
  *          library.
  ******************************************************************************
  */

/*!< Define to prevent recursive inclusion -------------------------------------*/
#ifndef __1888BC048_CRGSYS_H
#define __1888BC048_CRGSYS_H

#ifdef __cplusplus
 extern "C" {
#endif

/*!< Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */

/** @addtogroup CRGSYS
  * @{
  */
  
/** 
  * @brief   CRG_SYS
  */
typedef struct{
  struct{
    __I uint32_t PLL_RDY:     1;   //        
    __I uint32_t :         3;
    __I uint32_t PLL_CMD_ACK:   1;   //   ,      ,    PLL_CTRL
    __I uint32_t :         27;
  }PLL_STAT;//    
  struct{
    __IO uint32_t PLL_CMD:     2;   //    
    __I uint32_t:         30;
  }PLL_CTRL; //    
  __IO uint32_t PLL_LOCKDUR;  //      
  __IO uint32_t res0;      //   
  __IO uint32_t PLL_PRDIV;  //      (pre-divider)
  __IO uint32_t PLL_FBDIV;  //       (feedback divider)
  __IO uint32_t PLL_PSDIV;  //      (post-divider)
  __IO uint32_t res1[8];    //   
  __IO uint32_t WR_LOCK;    //       
  struct{
    __IO uint32_t RST_REQ_WDT:     1;   //           :1       ; 0        
    __IO uint32_t RST_REQ_SCTL:   1;  //          :1        0          R/WC  0x0
    __I uint32_t:           6;  
    __IO uint32_t RST_REQ_MODE0:   1;  //     1,         rstreq_i[n]    RSTREQ_MODE[n]=0   RST_CFG2  R/WC  0x0
    __IO uint32_t RST_REQ_MODE1:   1;  //     1,         rstreq_i[n]    RSTREQ_MODE[n]=1   RST_CFG2  R/WC  0x0
    __IO uint32_t RST_SYS_MON:    1;  //     sys_rstn_i: 1  sys_rstn_i    ; 0  sys_rstn_i       R/WC  0x0
    __IO uint32_t RST_POR_MON:     1;  //     por_rstn_i: 1  por_rstn_i    ; 0  por_rstn_i       R/WC  0x1
    __I uint32_t:           20;  
  }RST_MON;    //      
  __IO uint32_t RST_CFG0;    //       aresetn_o/rresetn_o[31:0]
  struct{
    __IO uint32_t SRST_DUR0:    12;  //       sys_rstn_oen_o   ,   (crg_clk/16)  R/W  0x96RST_REQ_WDT:     1;   //           :1       ; 0        
    __I uint32_t:           4;  
    __IO uint32_t SRST_DUR1  :    12;  //       sys_rstn_i      sys_rstn_oen_o,   (crg_clk/16)  R/W  0x32
    __I uint32_t:           4;
  }RST_CFG1;    //       sys_rstn_oen_o  
  struct{
    __IO uint32_t RSTREQ_WDT_MODE:  2;  //           WDT: 0x0:      aresetn_o 0x1:     aresetn_o  sys_rstn_oen_o 0x2:     crg_rstn, aresetn_o 0x3:     crg_rstn, aresetn_o, sys_rstn_oen_o  R/W  0x3
    __IO uint32_t RSTREQ_SCTL_MODE:  2;  //           SCTL (   RSTREQ_WDT_MODE)  R/W  0x3
    __I uint32_t:           12;
    __IO uint32_t RSTREQ_WDT_MASK:  1;  //           WDT: 0    WDT  1    WDT   R/W  0x1
    __IO uint32_t RSTREQ_SCTL_MASK:  1;  //           SCTL: 0    SCTL  1    SCTL   R/W  0x1
    __I uint32_t:           6;
    __IO uint32_t RSTREQ_BYP:    1;  //       1               R/W  0x0
    __I uint32_t:           7;
  }RST_CFG2;    //          SCTL  WDT
  __IO uint32_t res2[4];    //   
  struct{
    __IO uint32_t UPDCKDIV:      1;  //     .     CKDIVMODE[n]                     1,         .       
    __I uint32_t:           3;
    __IO uint32_t UPDCKEN:      1;  //     .    CKEN[n]      1,        .      ,   0
    __I uint32_t:           27;
  }CKUPDATE;    //     
  __IO uint32_t res3[11];    //   
  __IO uint32_t INTMASK;    //     
  __IO uint32_t INTCLR;    //       
  __IO uint32_t res4[26];    //   
  __IO uint32_t CKDIVMODE0;  //      ARM
  __IO uint32_t res5[3];    //   
  __IO uint32_t CKDIVMODE1;  //      AXI_FAST
  __IO uint32_t res6[3];    //   
  __IO uint32_t CKDIVMODE2;  //      AXI_SLOW
  __IO uint32_t res7[3];    //   
  __IO uint32_t CKDIVMODE3;  //      CAN
  __IO uint32_t res8[3];    //   
  __IO uint32_t CKDIVMODE4;  //      TPIU
  __IO uint32_t res9[3];    //   
  __IO uint32_t CKDIVMODE5;  //      MKIO
}CRG_SYS_TypeDef;

/** 
  * @brief   CRG_DDR
  */
typedef struct{
  struct{
    __I uint32_t PLL_RDY:     1;   //        
    __I uint32_t :         3;
    __I uint32_t PLL_CMD_ACK:   1;   //   ,      ,    PLL_CTRL
    __I uint32_t :         27;
  }PLL_STAT;//    
  struct{
    __IO uint32_t PLL_CMD:     2;   //    
    __I uint32_t:         30;
  }PLL_CTRL; //    
  __IO uint32_t PLL_LOCKDUR;  //      
  __IO uint32_t res0;      //   
  __IO uint32_t PLL_PRDIV;  //      (pre-divider)
  __IO uint32_t PLL_FBDIV;  //       (feedback divider)
  __IO uint32_t PLL_PSDIV;  //      (post-divider)
  __IO uint32_t res1[8];    //   
  __IO uint32_t WR_LOCK;    //       
  __IO uint32_t res2[3];    //   
  struct{
    __IO uint32_t RSTREQ_WDT_MODE:  2;  //           WDT: 0x0:      aresetn_o 0x1:     aresetn_o  sys_rstn_oen_o 0x2:     crg_rstn, aresetn_o 0x3:     crg_rstn, aresetn_o, sys_rstn_oen_o  R/W  0x3
    __IO uint32_t RSTREQ_SCTL_MODE:  2;  //           SCTL (   RSTREQ_WDT_MODE)  R/W  0x3
    __I uint32_t:           12;
    __IO uint32_t RSTREQ_WDT_MASK:  1;  //           WDT: 0    WDT  1    WDT   R/W  0x1
    __IO uint32_t RSTREQ_SCTL_MASK:  1;  //           SCTL: 0    SCTL  1    SCTL   R/W  0x1
    __I uint32_t:           6;
    __IO uint32_t RSTREQ_BYP:    1;  //       1               R/W  0x0
    __I uint32_t:           7;
  }RST_CFG2;    //          SCTL  WDT
  __IO uint32_t res3[4];    //   
  struct{
    __IO uint32_t UPDCKDIV:      1;  //     .     CKDIVMODE[n]                     1,         .       
    __I uint32_t:           3;
    __IO uint32_t UPDCKEN:      1;  //     .    CKEN[n]      1,        .      ,   0
    __I uint32_t:           27;
  }CKUPDATE;    //     
  __IO uint32_t res4[11];    //   
  __IO uint32_t INTMASK;    //     
  __IO uint32_t INTCLR;    //       
  __IO uint32_t res5[26];    //   
  __IO uint32_t CKDIVMODE0;  //      DDR0
  __IO uint32_t res6[3];    //   
  __IO uint32_t CKDIVMODE1;  //      DDR1
}CRG_DDR_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */

#define CRG_SYS        ((CRG_SYS_TypeDef *)CRG_SYS_BASE)
#define CRG_DDR        ((CRG_DDR_TypeDef *)CRG_DDR_BASE)

// PLL_CTRL
#define  PLL_CMD_OSC_USE    0  /*     */  
#define  PLL_CMD_OSC_BYP    ((uint32_t)0x00001)/*  ,   */
#define  PLL_CMD_OFF_BYP    ((uint32_t)0x00003)/*  ,   */

#define  PLL_PRDIV_INIT    0
#define  PLL_FBDIV_INIT    0x63
#define  PLL_PSDIV_INIT    0

// CRG_SYS
#define CKDIV_ARM         CKDIVMODE0
#define CKDIV_AXI_FAST    CKDIVMODE1
#define CKDIV_AXI_SLOW    CKDIVMODE2
#define CKDIV_CAN         CKDIVMODE3
#define CKDIV_TPIU        CKDIVMODE4
#define CKDIV_MKIO        CKDIVMODE5
// CRG_DDR
#define CKDIV_DDR0        CKDIVMODE0
#define CKDIV_DDR1        CKDIVMODE1

/** @defgroup Configuration_PLL_CMD_enumeration 
  * @{
  */
typedef enum
{
  PLL_CMD_Osc_Use = 0,/*     */
  PLL_CMD_Osc_Byp = 1,/*  ,   */
  PLL_CMD_Osc_Off = 3/*  ,   */
}CRGSYS_PLL_CMD_TypeDef;

// WR_LOCK       SYS & DDR.
#define  WR_LOCK_DISABLE   ((uint32_t)0x1ACCE551) /*      */
#define SET_CKDIV(sys,ckdiv,num) sys->WR_LOCK = WR_LOCK_DISABLE;\
  sys->ckdiv = num; sys->CKUPDATE.UPDCKDIV = 1; \
  while(sys->CKUPDATE.UPDCKDIV){} sys->WR_LOCK = ~WR_LOCK_DISABLE
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Function used to set the CRGSYS configuration to the default reset state **/
void CRGSYS_DeInit(void);
void CRGSYS_Init(uint8_t div_in, uint16_t mult, uint8_t div_out, CRGSYS_PLL_CMD_TypeDef cmd);
/* CRGSYS configuration functions **********************************************/

#ifdef __cplusplus
}
#endif

#endif /*__1888BC048_CRGSYS_H */

/**
  * @}
  */

/************************ (C) COPYRIGHT STC MODULE *****END OF FILE****/
