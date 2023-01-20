/**
  ******************************************************************************
  * @file    1888bc048_uart.h
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    13.02.2020
  * @brief   This file contains all the functions prototypes for the UART 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_UART_H
#define __1888BC048_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */

/** @defgroup UART_Stop_Bits 
  * @{
  */ 
typedef enum{
  UART_StopBits_1 = 0,
  UART_StopBits_1_5,
  UART_StopBits_2  
}UART_Stop_Bits;

/** @defgroup UART_Length 
  * @{
  */ 
typedef enum{
  UART_Length_5 = 0,
  UART_Length_6,
  UART_Length_7,
  UART_Length_8,
}UART_Length;
    
/** @defgroup UART_Length 
  * @{
  */ 
typedef enum{
  UART_DUM_DataFlags,
  UART_DUM_OnlyData,
}UART_DUM;
  
/**
  * @brief UART_FIFO_STATE structure definition  
  */

typedef struct{
  uint32_t RxNum:   11; //  Receive FIFO state.    FIFO  
  uint32_t :        5;  //
  uint32_t TxNum:   11; //	Transmit FIFO state.    FIFO 
  uint32_t :        5;  //
}UART_FIFO_STATE_TypeDef;
  
/**
  * @brief UART_IRQ_STATE structure definition  
  */
  
typedef struct{
  __IO uint32_t RxLevel:      1;  //	 FIFO     (  0   )
  __IO uint32_t TxLevel:   	  1;  //   FIFO     (  0   )
  __IO uint32_t RxTimeout:    1;  //	 -    (  0   )
  __IO uint32_t StopErr:      1;  //	   (  0   )
  __IO uint32_t ParErr:	      1;  //	   (  0   )
  __IO uint32_t BreakErr:	    1;  //	   (  0   )
  __IO uint32_t RxOver:	      1;  //	  FIFO  (  0   )
  __IO uint32_t TxOver:	      1;  //	  FIFO  (  0   )
  __IO uint32_t TxTimeout:    1;  //	 -    (  0   )
  __IO uint32_t :             23;
}UART_IRQ_STATE_TypeDef;

/**
  * @brief UART_STATUS structure definition  
  */
  
typedef struct{
  uint32_t suspend_done:    1; /*!<    DMA_GP */
  uint32_t cancel_done:     1; /*!<	  DMA_GP (cancel) */
  uint32_t int_desc:        1; /*!<	     Int */
  uint32_t bad_desc:        1; /*!<	    DMA-GP ( ownership  1) */
  uint32_t stop_desc:       1; /*!<	     Stop */
  uint32_t discard_desc:    1; /*!<	     AXI    */
  uint32_t waxi_err:        1; /*!<	     AXI    */
  uint32_t axi_err:         1; /*!<	     AXI  /  */
  uint32_t start_by_event:  1; /*!<	     */
  uint32_t ignore_event:    1; /*!<	,  DMA-GP,      DMA-GP */
  uint32_t :                22; /*!<  */
}UART_STATUS_TypeDef;

/**
  * @brief UART_DMA_STATE structure definition  
  */
  
typedef struct{
  uint32_t wr_fifo_full:      1; /*!<	 FIFO     */
  uint32_t wr_fifo_empty:     1; /*!<	 FIFO     */
  uint32_t rd_fifo_empty:     1; /*!<	 FIFO     */
  uint32_t:                   1; /*!<	 */
  uint32_t ready_for_str:     1; /*!<	    pitch-,     */
  uint32_t ready_for_mod:     1; /*!<	   ,     */
  uint32_t:                   2; /*!<	 */
  uint32_t data_desc_cnt:     2; /*!<	  ,       DMA */
  uint32_t:                   2; /*!<	 */
  uint32_t desc_cnt:          3; /*!<	 ,       DMA */
  uint32_t:                   1; /*!<  */
  uint32_t discard_desc:      1; /*!<	      . */
  uint32_t bad_flag:          1; /*!<	 ,   DMA */
  uint32_t stop_flag:         1; /*!<	    stop */
  uint32_t stop:              1; /*!<	    */
  uint32_t data_send_permit:  1; /*!<	        */
  uint32_t cancel:            1; /*!<	  cancel */ 
  uint32_t suspend:           1; /*!<	  suspend */
  uint32_t enable:            1; /*!<	  DMA */
  uint32_t desc_is_writing:   1; /*!<	    */
  uint32_t desc_is_mod:       1; /*!<	    */ 
  uint32_t wr_state:          1; /*!<	  : 1      */
  uint32_t rd_state:          1; /*!<	  : 1      */
  uint32_t state:             2; /*!<	 DMA: 0   ; 1   ; 2   ; 3        ,     */
  uint32_t:                   2; /*!<	 */
}UART_DMA_STATE_TypeDef;
 
/**
  * @brief UART_SETTINGS  structure definition  
  */
  
typedef struct{
  uint32_t desc_type:       2; /*!<     10  long- (128 ) */
  uint32_t:                 2; /*!<  */
  uint32_t add_info:        1; /*!<   1 */
  uint32_t:                 11; /*!<	 */
  uint32_t desc_gap:        16; /*!< 	   */
}UART_SETTINGS_TypeDef;
 
/**
  * @brief UART_DMA_SETTINGS structure definition  
  */
typedef struct{
  __IO uint32_t	Enable;   /*!< 00	/  0x100 */
  __IO uint32_t Suspend;  /*!< 00	   */
  __IO uint32_t	Cancel;   /*!< 00	       0x108 */
  __I  uint32_t res0;     /*!<  */
  __IO UART_SETTINGS_TypeDef Settings; /*!< 00	  0x110 */
  __IO UART_STATUS_TypeDef IrqMask; /*!< 0x0	   0x114 */  
  __I  UART_STATUS_TypeDef Status;  /*!< 0x0	    0x118 */
  __I  uint32_t res1;        /*!<  */
  __IO uint32_t DescAddr;    /*!< 00	   */
  __I  uint32_t res2;        /*!<  */
  __I  uint32_t CurDescAddr; /*!< 0x0	   */
  __I  uint32_t CurAddr;     /*!< 0x0	   */
  __I  UART_DMA_STATE_TypeDef	DMA_State;  /*!< 0x0	   0x130 */
  __I  uint32_t res3[3];     /*!<  */
  __IO uint32_t	DESC_AXLEN;  /*!< 0x3	 ARLEN  AWLEN    AXI      (      [3:0]) */
  __IO uint32_t	DESC_ACACHE; /*!< 0x3	 ARCACHE  AWCACHE    AXI      (      [3:0]) */
  __IO uint32_t	DESC_APROT;  /*!< 0x2	 ARPROT  AWPROT    AXI      (      [2:0]) */
  __IO uint32_t	DESC_ALOCK;  /*!< 0x0	 ARLOCK  AWLOCK    AXI      (      [1:0]) */
  __I  uint32_t	DESC_RRESP;  /*!< 00	 RRESP    AXI    (      [1:0]) */
  __I  uint32_t	DESC_RAXI_ERR_ADDR;  /*!<   00	 AXI-  ,     AXI */
  __I  uint32_t	DESC_BRESP;  /*!< 00	 BRESP    AXI    (      [1:0]) */
  __I  uint32_t	DESC_WAXI_ERR_ADDR;  /*!<   00	 AXI-    ,     AXI */
  __IO uint32_t	DESC_PERMUT; /*!< 0x76543210	         AXI */
  __I  uint32_t res4[7];     /*!<  */
  __IO uint32_t	MaxTransNum; /*!< MAX_TRANS_NUM	    (  15) */
  __IO uint32_t	ARLEN;       /*!< 0F	 AWLEN    AXI (      [3:0]) */
  __IO uint32_t	ARCACHE;     /*!< 0x3	 AWCACHE    AXI (      [3:0]) */
  __IO uint32_t	ARPROT;      /*!< 0x2	 AWPROT    AXI (      [2:0]) */
  __IO uint32_t	ARLOCK;      /*!< 0x0	 AWLOCK    AXI (      [1:0]) */
  __I  uint32_t	RRESP;       /*!< 00	 BRESP    AXI (      [1:0]) */
  __I  uint32_t	RAXI_ERR_ADDR;  /*!<  00	 AXI- ,     AXI */
  __I  uint32_t res5;        /*!< 0x19C	-	-	-	 */
  __I  uint32_t	RAXI_STATE;  /*!< 0x0	   AXI */
  __I  uint32_t	AVAILABLE_SPACE; /*!<  0x50	      */
  __IO uint32_t	PERMUTATION; /*!<  0x76543210	       AXI */
  __I  uint32_t res6[21];    /*!<  */
}UART_DMA_SETTINGS_TypeDef;

/**
  * @brief UART_CTRL structure definition  
  */
typedef struct{
  uint32_t Enable:     1;  //	UART enable.  UART: 0 = UART ; 1 =  UART 
  uint32_t Loop:       1;  //	  .     1,  TXD     RXD,   RTS    CTS
  uint32_t APB_FIFO:   1;  //	   APB: 0 =   FIFO  APB  ; 1 =   FIFO  APB  
  uint32_t RtsMode:    1;  //      RTS: 0 =  RS-232; 1 =  RS-485
  uint32_t RtsEn:      1;  //    1,    RTS
  uint32_t CtsEn:      1;  //    1,    CTS
  uint32_t POL:        1;  //	   RTS/CTS: 0 =    0 (active low); 1 =    1 (active high)
  uint32_t PAR:        1;  //	 : 0 =  ; 1 =  
  uint32_t EPS:        1;  //	 : 0 = ; 1 = 
  uint32_t SPS:        1;  //	Stick Parity Select.   : 0 =    ; 1 =   : EPS = 1,        1; EPS = 0,        0
  uint32_t StopBits:   2;  //     : b00 = 1 ; b01 = 1,5 ; b10 = 2 
  uint32_t Length:     3;  //   : b100  b111 = ; b011 = 8 ; b010 = 7 ; b001 = 6 ; b000 = 5 
  uint32_t DUM:	       1;  //  Data unloading mode.        DMA: 0   data and flags; 1  data only
  uint32_t :           16;
}UART_CTRL_TypeDef;

/**
  * @brief UART_DIV structure definition  
  */  
typedef struct{
  uint32_t BAUD_DIV:   24; //	Baud rate divisor.    
  uint32_t N_DIV:      1;  //	0   N  8; 1   N  10
  uint32_t :            7;
}UART_DIV_TipeDef;  
/**
  * @brief UART structure definition  
  */

typedef struct{
//    UART  DMA  
  __I  uint32_t ID;  /*!< 0x55415254	  */
  struct{
    __I  uint32_t REF_FREQ:   9;    //	  
    __I  uint32_t SUBVERSION: 4;    //	      
    __I  uint32_t NUMBER:     4;    //	  
    __I  uint32_t  :          15;   //  
  }VERSION;	/*!< 0x10190	     */
//  UART
  __IO uint32_t Reset;	        /*!< 0x0	1    .      */
  __I  uint32_t res0;           /*!< 0x00C	-	-	-	 */
  struct{
    __I  uint32_t UART_IRQ:   1;  //   UART
    __I  uint32_t DMA_RD_IRQ: 1;  //     DMA
    __I  uint32_t DMA_WR_IRQ: 1;  //     DMA
    __I  uint32_t :          29; // 
  }GEN_STATUS;	    /*!< 0x0	     :  UART,    DMA     DMA */
  __I UART_FIFO_STATE_TypeDef FIFO_STATE;	    /*!< 0x0	   FIFO , FIFO       */
  __I UART_IRQ_STATE_TypeDef STATUS;	        /*!< 0x0	        0x018 */
  __I uint32_t res1;          /*!< 0x01C	-	 */
  __IO uint32_t TxD;	                      /*!< 0x0  	*/
  __I  uint32_t res2;         /*!< 0x024	-	 */
  struct{
    __I  uint32_t Data: 8; //	 
    __I  uint32_t :     4; // 
    //   ...
    __I  uint32_t FE:   1; //	Frame error.   .   1,         (    1)   
    __I  uint32_t PE:   1; //	Parity error.  .   1,        ,    EPS  SPS  UART_CTRL   
    __I  uint32_t BE:   1; //	Break error.   .   1,    ,  ,     0       ( , ,  ,  )   
    __I  uint32_t OE:   1; //	Overflow error.  .   1,   ,  FIFO   .    FIFO   ,     0   
    __I  uint32_t :     16; // 
  }RxD;	                      /*!<   */
  __I  uint32_t res3;         /*!< 0x02C -  */  
  __IO UART_DIV_TipeDef BDIV;                      /*!< 0x4	   . */
  __I  uint32_t res4[3];      /*!< 0x0340x03C -  */
  __IO UART_FIFO_STATE_TypeDef FIFOWM;    /*!< 0x2000200	      FIFO    */
  __IO UART_CTRL_TypeDef CTRL;      /*!< 0x0	          0x044 */
  __IO UART_IRQ_STATE_TypeDef MASK; /*!< 0x0	    . 1 -  ; 0 -   0x048 */
  __IO uint32_t RxTimeout;  /*!< 0x0	 - ,      */
  __I  uint32_t res5;       /*!< 0x050	-	 */
  __IO uint32_t TxTimeout;  /*!< 0x0	 -    RS-232,      */
  __I  uint32_t res6[42];   /*!< 0x0580x0FF	-	 */
  UART_DMA_SETTINGS_TypeDef TxDmaSet; /*!<   DMA     0100 */
  UART_DMA_SETTINGS_TypeDef RxDmaSet; /*!<   DMA     0200 */
}UART_TypeDef;

/**
  * @brief UART_Init structure definition  
  */
  
typedef struct{
  UART_DIV_TipeDef    Div;
  UART_CTRL_TypeDef   Ctrl;
}UART_InitTypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  

#define UART0                 ((UART_TypeDef*)MUART0_BASE)
#define UART1                 ((UART_TypeDef*)MUART1_BASE)

/** @defgroup ***
  * @{
  */
  
/* Initialization and Configuration functions *********************************/
void UART_StructInit(UART_InitTypeDef* UART_InitStruct);
void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct);
/* GPIO Read and Write functions **********************************************/
void UART_Write(UART_TypeDef* UARTx, uint8_t PortVal);
void UART_SendData(UART_TypeDef *port, uint8_t *data, uint16_t num);
uint32_t UART_ReadInputData(UART_TypeDef* UARTx);
uint32_t UART_ReadData(UART_TypeDef* UARTx, uint8_t *data, uint16_t limit);
uint32_t UART_GetRxNum(UART_TypeDef* UARTx);
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_UART_H */
/**
  * @}
  */
