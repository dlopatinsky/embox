/**
  ******************************************************************************
  * @file    1888bc048_spi.h
  * @author  A. Maksimov, V. Syrtsov 
  * @version V1.0.1
  * @date    11.12.2019
  * @brief   This file contains all the functions prototypes for the SPI 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_SPI_H
#define __1888BC048_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
  
/** @addtogroup ...
  * @{
  */ 

#define SPI_DMAWCNTRL_MAGIC     (0x1FFFFFE0)  /*!>       */
#define SPI_DMA_READ_DESC       (SPI_DMAWCNTRL_MAGIC | (1UL<<30)) /*!>   DMA  */
#define SPI_DMA_WRITE_DESC      (SPI_DMAWCNTRL_MAGIC | (1UL<<31)) /*!>   DMA  */

#define SPI_AXIR_BUFTYPE_LOOP   (0UL) /*!>   */
#define SPI_AXIR_BUFTYPE_BASIC  (1UL) /*!>   ( ) */

#define SPI_WORDOP_8_Bits       (0UL) /*!>    8  */
#define SPI_WORDOP_16_Bits      (1UL) /*!>    16  */


/** @defgroup SPI_DSS_enumeration -    :
  * @{
  */
typedef enum{
  DSS_4_Bits = 3, DSS_5_Bits, DSS_6_Bits, DSS_7_Bits, DSS_8_Bits, DSS_9_Bits, DSS_10_Bits, 
  DSS_11_Bits, DSS_12_Bits, DSS_13_Bits, DSS_14_Bits, DSS_15_Bits, DSS_16_Bits,
}SPI_DSS_Typedef;

/** @defgroup SPI_FRF_enumeration -  :
  * @{
  */
typedef enum{
  FRF_SPI_Motorola = 0,
  FRF_SSI_TI,
  FRF_Microwire_NS  
}SPI_FRF_Typedef;

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
typedef enum{
  SPH_1Edge = 0,
  SPH_2Edge = 1
}SPI_SPH_Typedef;

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
typedef enum{
  SPO_Low = 0,
  SPO_High = 1
}SPI_SPO_Typedef;

/**
  * @brief ... union definition  
  */  

/**
  * @brief SPI_MASK structure definition  
  */
typedef struct{
  uint32_t ROR:          1;  /*!>    RORINTR */
  uint32_t RT:           1;  /*!>    RTINTR */
  uint32_t RX:           1;  /*!>    RXINTR */
  uint32_t TX:           1;  /*!>    TXINTR */
  uint32_t : 28;             /*!>  */  
}SPI_MASK_TypeDef; 

/**
  * @brief SPI_STATUS structure definition  
  */
typedef struct{
    uint32_t Spi:        1; /*!>   SPI  */
    uint32_t Gspi:       1; /*!>   GSPI */
    uint32_t WrBufEnd:   1; /*!>    DMA    AXI */
    uint32_t RdBuffEnd:  1; /*!>    DMA    AXI */
    uint32_t RdAxiErr:   1; /*!>      AXI,   */
    uint32_t WrAxiErr:   1; /*!>      AXI,   */
    uint32_t WrBufOver:  1; /*!>       AXI */
    uint32_t WrBufEmpty: 1; /*!>      AXI */
    uint32_t :  24;         /*!>  */
}SPI_STATUS_TypeDef;

/**
  * @brief SPI_IRQ structure definition  
  */
//  :
typedef struct{
    uint32_t Spi:        1; /*!>  SPI  */
    uint32_t Gspi:       1; /*!>  GSPI */
    uint32_t WrBufEnd:   1; /*!>   DMA    AXI */
    uint32_t RdBuffEnd:  1; /*!>   DMA    AXI */
    uint32_t RdAxiErr:   1; /*!>     AXI,   */
    uint32_t WrAxiErr:   1; /*!>     AXI,   */
    uint32_t WrBufOver:  1; /*!>      AXI */
    uint32_t :  25;         /*!>  */
}SPI_IRQ_TypeDef; 

typedef struct{
//                       
  struct{
    __IO uint32_t DSS:      4; /*!>    , . SPI_DSS_enumeration */
    __IO uint32_t FRF:      2; /*!>  , . SPI_FRF_enumeration */
    __IO uint32_t SckPol:   1; /*!>   SCK (    FRF_SPI_Motorola) */
    __IO uint32_t SckPhase: 1; /*!>   SCK (    FRF_SPI_Motorola) */
    __IO uint32_t SCR:      8; /*!>  = Fspi_clk/(CPSR.DIV*(1+SCR)) */
    __IO uint32_t :    16; 
  }CR0;                         // 0x00	 0x0	    0
  struct{
    __IO uint32_t Test:   1;    /*!>    */
    __IO uint32_t Enable: 1;    /*!>   */
    __IO uint32_t :    30; 
  }CR1;                         // 0x04	 0x0	    1
  __IO uint32_t DR;             // 0x08	 0xX	   
  __I  uint32_t SR;             // 0x0C	 0x3	                            
  struct{
    __IO uint32_t DIV: 8;         /*!>     (  2  254) */
    uint32_t : 24;                /*!>  */
  }CPSR;                        // 0x10	 0x0	     
  struct{
    __IO uint32_t RxFIFO_Over:  1;  /*!> 0      FIFO  */
    __IO uint32_t RxFIFO_Wait:  1;  /*!> 0     ,             */
    __IO uint32_t RxFIFO_Half:  1;  /*!> 0     50%       */
    __IO uint32_t TxFIFO_Half:  1;  /*!> 0     50%       */    
    uint32_t : 28;                  /*!>  */
  }IMSC;                        // 0x14	 0x0	       
  __I  SPI_MASK_TypeDef RIS;    // 0x18	 0x8	      
  __I  SPI_MASK_TypeDef MIS;    // 0x1C	 0x0	       
  struct{
    __O  uint32_t ROR:  1;          /*!>   ROR */
    __O  uint32_t RT:   1;          /*!>   RT */
    uint32_t : 30;                  /*!>  */
  }ICR;                         // 0x20	 0x0	    
  struct{
    __IO  uint32_t Rx:    1;        /*!>     */
    __IO  uint32_t Tx:    1;        /*!>     */
    uint32_t : 30;                  /*!>  */
  }DMACR;                       // 0x24  0x0	       
  __I  uint8_t r0[0x90-0x28];   // 0x28..   	  
  __IO uint32_t DMAWSTART;      // 0x90	 0x0	     DMA
  __IO uint32_t DMAWEND;        // 0x94	 0x0	     DMA
  __IO uint32_t DMAWCNTRL;      // 0x98	 0x0	    DMA : (SPI_DMA_READ_DESC  SPI_DMA_WRITE_DESC)
  __I  uint32_t DMAWCUR;        // 0x9C	 0x0	     DMA
  __I  uint32_t AXIW_BRESP;     // 0xA0	 0x0	   BRESP     AXI -  
  __IO uint32_t DMARSTART;      // 0xA4	 0x0	     DMA
  __IO uint32_t DMAREND;        // 0xA8	 0x0	     DMA
  __IO uint32_t DMARCNTRL;      // 0xAC	 0x0	    DMA : (SPI_DMA_READ_DESC  SPI_DMA_WRITE_DESC)
  __I  uint32_t DMARCUR;        // 0xB0	 0x0	    DMA  
  __IO uint32_t AXIR_BUFTYPE;   // 0xB4	 0x0	    DMA-,   AXI
  __IO uint32_t AXIR_BUFENA;    // 0xB8	 0x0	    ,   AXI
  __I  uint32_t r1;             // 0xBC         
  __I  uint32_t AXIR_ERRADDR;   // 0xC0	 0x0	    ,       AXI-  ( SLVERR)
  struct{
    __IO uint32_t WrLen:    4;          /*!>      AXI,   AXI */
    __IO uint32_t WrEndian: 1;          /*!>    32-  ,   AXI: 0  LSB, 1  MSB */
    __IO uint32_t RdLen:    4;          /*!>      AXI,   AXI */
    __IO uint32_t RdEndian: 1;          /*!>    32-  ,   AXI: 0  LSB, 1  MSB */ 
    uint32_t :22;                       /*!>  */
  }AXI_PARAMS;                    // 0xC4	 0x0	     AXI 
  __IO uint32_t WORDOP;           // 0xC8	 0x0	      SPI  16 
  __IO uint32_t SOFTSS;           // 0xCC	 0x0	     Slave Select
  __IO uint32_t SOFTRST;          // 0xD0	 0x0	     SPI 
  __I  SPI_STATUS_TypeDef Status; // 0xD4	 0x0	    
  __IO SPI_IRQ_TypeDef IrqMasks;  // 0xD8	 0x0	    
  __I  uint8_t r2[0xFE0-0xDC];    // 0xDC..       
//------	    ------
  __I  uint32_t PeriphID0;        // 0xFE0 0x22
  __I  uint32_t PeriphID1;        // 0xFE4 0x10	
  __I  uint32_t PeriphID2;        // 0xFE8 0x24	
  __I  uint32_t PeriphID3;        // 0xFEC 0x00	
//------------   SPI  ---------------
  __I  uint32_t CellID0;          // 0xFF0 0x0D
  __I  uint32_t CellID1;          // 0xFF4 0xF0	
  __I  uint32_t CellID2;          // 0xFF8 0x05	
  __I  uint32_t CellID3;          // 0xFFC 0xB1	
}SPI_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  

#define SPI0              ((SPI_TypeDef *)GSPI0_BASE)
#define SPI1              ((SPI_TypeDef *)GSPI1_BASE)

#define IS_SPI_ALL_PERIPH(PERIPH)          (((PERIPH) == SPI0) || \
                                            ((PERIPH) == SPI1))


#define IS_SPI_SPEED_FACTOR(DIVIDER)       ((DIVIDER) <= 0xFF)

#define IS_SPI_SPEED_DIVIDER(DIVIDER)      (((DIVIDER) <= 0xFE) && \
                                            ((DIVIDER) >= 0x02) && \
                                            (((DIVIDER) & 0x01) == 0))

/** @defgroup SPI_Flags SPI Flags
  * @{
  */

#define SPI_FLAG_BSY                        ((uint16_t)0x0010)
#define SPI_FLAG_RFF                        ((uint16_t)0x0008)
#define SPI_FLAG_RNE                        ((uint16_t)0x0004)
#define SPI_FLAG_TNF                        ((uint16_t)0x0002)
#define SPI_FLAG_TFE                        ((uint16_t)0x0001)

#define IS_SPI_FLAG(FLAG)                   (((FLAG) == SPI_FLAG_BSY) || \
                                             ((FLAG) == SPI_FLAG_RFF) || \
                                             ((FLAG) == SPI_FLAG_RNE) || \
                                             ((FLAG) == SPI_FLAG_TNF) || \
                                             ((FLAG) == SPI_FLAG_TFE))


/** @defgroup SSP_SOFTSS SSP SOFTSS
  * @{
  */
#define SPI_SOFTSS_LEVEL_Pos                0
#define SPI_SOFTSS_EN_Pos                   1

#define SPI_SOFTSS_LEVEL_Msk                0x00000001UL
#define SPI_SOFTSS_EN_Msk                   0x00000002UL

/**
  * @brief  SPI Init Structure definition
  */

typedef struct
{
  uint16_t SPI_SCR;                        /*!< This member configures the SPI communication speed.
                                                This parameter is number from 0 to 255.
                                                The information rate is computed using the following formula:
                                                F_SPICLK / ( CPSDVR * (1 + SCR) ) */
  uint16_t SPI_CPSDVSR;                    /*!< This member configures the SPI clock divider.
                                                This parameter is an even number from 2 to 254 */
  SPI_DSS_Typedef SPI_WordLength;                 /*!< Specifies the number of data bits transmitted or received in a frame.
                                                This parameter is one of @ref SPI_Word_Length values. */
  SPI_SPH_Typedef SPI_SPH;                        /*!< Specifies the number of stop bits transmitted.
                                                This parameter is one of @ref SPI_Clock_Phase values. */
  SPI_SPO_Typedef SPI_SPO;                        /*!< Specifies the number of stop bits transmitted.
                                                This parameter is one of @ref SPI_Clock_Polarity values. */
  SPI_FRF_Typedef SPI_FRF;                        /*!< Specifies the number of stop bits transmitted.
                                                This parameter is one of @ref SPI_Frame_Format values. */
}SPI_InitTypeDef;
  
/* Initialization and Configuration functions *********************************/

void SPI_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, const SPI_InitTypeDef* SPI_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_SoftSS(SPI_TypeDef* SPIx, BitStatus LevelValue);

FlagStatus SPI_GetFlagStatus(SPI_TypeDef* SPIx, uint32_t SPI_FLAG);

/* SPI Read and Write functions **********************************************/

void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx);

/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_SPI_H */

/**
  * @}
  */
  
