/**
  ******************************************************************************
  * @file    1888bc048_i2c.h
  * @author  V. Syrtsov
  * @version 
  * @date    
  * @brief   This file contains all the functions prototypes for the I2C 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_I2C_H
#define __1888BC048_I2C_H

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

#define ISR_INT_DONE                    0x00000001  // Byte operation completed
#define ISR_INT_AL                      0x00000002  // A lost arbitration occurred
#define ISR_INT_TRN_EMPTY               0x00000004  // Transmit buffer empty
#define ISR_INT_RCV_FULL                0x00000008  // Received buffer full
#define ISR_INT_TRN_ALMOST_EMPTY        0x00000010  // When sending the next byte, the transmitting buffer reached the fullness level specified in the FIFOFIL register
#define ISR_INT_RCV_ALMOST_FULL         0x00000020  // When the next byte was received, the receive buffer reached the fullness level indicated in the FIFOFIL register
#define ISR_INT_NACK                    0x00000040  // No confirmation received (ACK)
#define ISR_INT_RCV_FINISH              0x00000080  // NUMBR bytes received in the current transaction
#define ISR_INT_BUSY_END                0x00000100  // The controller ends the operation and releases the I2C interface

#define IER_EN_INT_DONE                 0x00000001  // Interrupt enable for each of the bits of the ISR register: 0 - interrupt disabled, 1 - interrupt enabled
#define IER_EN_INT_AL                   0x00000002
#define IER_EN_INT_TRN_EMPTY            0x00000004
#define IER_EN_INT_RCV_FULL             0x00000008
#define IER_EN_INT_TRN_EMPTY_ALM        0x00000010
#define IER_EN_INT_RCV_FULL_ALM         0x00000020
#define IER_EN_INT_NACK                 0x00000040
#define IER_EN_INT_RCV_FINISH           0x00000080
#define IER_EN_INT_BUSY_END             0x00000100

#define CR_REG_I2C_EN                   0x00000001  // Enabling the I2C controller
#define CR_REG_START                    0x00000002  // Command begin with a start character (S).
#define CR_REG_TX_FIFO_RST              0x00000004  // Reset TX FIFO.
#define CR_REG_RD                       0x00000008  // Reading command
#define CR_REG_WR                       0x00000010  // Writing command
#define CR_REG_REPEAT                   0x00000020  // Command begin with a repeat start character  (Sr).
#define CR_REG_STOP                     0x00000040  // Command ends with a stop character (P).

#define SR_REG_IBUSY                    0x00000001  // I2C bus is busy (was symbol S, but P don't present).
#define SR_REG_AL                       0x00000002  // A lost arbitration occurred
#define SR_REG_TRN_ALMOST_EMPTY         0x00000004  // When sending the next byte, the transmitting buffer reached the fullness level specified in the FIFOFIL register
#define SR_REG_RCV_ALMOST_FULL          0x00000008  // When the next byte was received, the receive buffer reached the fullness level indicated in the FIFOFIL register
#define SR_REG_TRN_EMPTY                0x00000010  // Transmit buffer empty
#define SR_REG_RCV_FULL                 0x00000020  // Received buffer full
#define SR_REG_TRN_FULL                 0x00000040  // Transmit buffer full
#define SR_REG_RCV_EMPTY                0x00000080  // Received buffer empty
#define SR_REG_RES                      0x00000100  // Reserved
#define SR_REG_NACK                     0x00000200  // NACK was received
#define SR_REG_DONE                     0x00000400  // Byte operation completed
#define SR_REG_STATE_CMD                0x00007800  // State of the send / receive state mashine
// 0b0000  void,0b0001  start sending (S),0b0010  stop sending (P),0b0100  transmiting, 0b1000  receiving
#define SR_REG_CNT                      0x007F8000  // The number of bytes received in the current transaction (counts from 0 to NUMBR)
#define SR_REG_SCL_IN                   0x00800000  // The current value of the SCL signal at the external pin
#define SR_REG_SCL_OEN                  0x01000000  // 0 - the controller outputs 0 to the SCL bus, 1 - does not
#define SR_REG_SDA_IN                   0x02000000  // The current value of the SDA signal at the external pin
#define SR_REG_SDA_OEN                  0x04000000  // 0 - the controller outputs 0 to the SDA bus, 1 - does not

typedef struct{
//                       
  __O  uint32_t ID;
  __O  uint32_t ISR;
  __IO uint32_t IER;
  __O  uint32_t SOFTR;
  __IO uint32_t CR;
  __I  uint8_t r0[0x18-0x14];
  __I  uint32_t SR;
  __IO uint32_t NUMBR;
  __IO uint32_t STAT_RST;
  __IO uint32_t CLKPR;
  __IO uint32_t FIFOFIL;
  __I  uint32_t FILTR;
  __O  uint32_t TX_FIFO;
  __I  uint8_t r1[0x430-0x34];
  __I  uint32_t RX_FIFO;
}I2C_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  

#define I2C0              ((I2C_TypeDef *)I2C0_BASE)
#define I2C1              ((I2C_TypeDef *)I2C1_BASE)

#define IS_I2C_ALL_PERIPH(PERIPH)				(((PERIPH) == I2C0) ||\
												 ((PERIPH) == I2C1))

/** @defgroup I2C_CLK I2C CLK
  * @{
  */

#define I2C_CLK_100_KHz		((uint32_t)0x000000C7)
#define I2C_CLK_400_KHz		((uint32_t)0x00000031)
#define I2C_CLK_1_MHz		((uint32_t)0x00000013)

#define IS_I2C_CLK(CLK)				(((CLK) == I2C_CLK_100_KHz) ||\
									 ((CLK) == I2C_CLK_400_KHz) ||\
								     ((CLK) == I2C_CLK_1_MHz))

// Stop. If the previous operation completed in a busy bus state, this command causes a completion character (P) to be sent
#define CMD_STOP (CR_REG_I2C_EN | CR_REG_STOP)
#define CMD_START (CR_REG_I2C_EN | CR_REG_START)
// The transmit buffer is empty
#define CMD_RST_BUF (CR_REG_I2C_EN | CR_REG_TX_FIFO_RST)
// Complete write transaction. With rpt = 0 it starts with the start symbol (S), with rpt = 1 - with the restart symbol (Sr). Ends with a termination character (P)
#define CMD_WRITE_FULL (CR_REG_I2C_EN | CR_REG_WR | CR_REG_START | CR_REG_STOP)
#define CMD_WRITE_FULL_REP (CMD_WRITE_FULL | CR_REG_REPEAT)
// Start of a write transaction. With rpt = 0 it starts with the start symbol (S), with rpt = 1 - with the restart symbol (Sr). After performing this operation, the bus remains busy (SCL = 0)
#define CMD_WRITE_BEGIN (CR_REG_I2C_EN | CR_REG_WR | CR_REG_START)
#define CMD_WRITE_BEGIN_REP (CMD_WRITE_BEGIN | CR_REG_REPEAT)
// Continue a previously started write operation. After performing this operation, the bus remains busy (SCL = 0)
#define CMD_WRITE_CONTINUE (CR_REG_I2C_EN | CR_REG_WR)
// Completion of a previously started write operation. Ends with a termination character (P)
#define CMD_WRITE_FINISH (CR_REG_I2C_EN | CR_REG_WR | CR_REG_STOP)
// Complete read transaction. With rpt = 0 it starts with the start symbol (S), with rpt = 1 - with the restart symbol (Sr). Ends with a completion character (P)
#define CMD_READ_FULL (CR_REG_I2C_EN | CR_REG_RD | CR_REG_START | CR_REG_STOP)
#define CMD_READ_FULL_REP (CMD_READ_FULL | CR_REG_REPEAT)
// Start of a read transaction. With rpt = 0 it starts with the start symbol (S), with rpt = 1 - with the restart symbol (Sr). After performing this operation, the bus remains busy (SCL = 0)
#define CMD_READ_BEGIN (CR_REG_I2C_EN | CR_REG_RD | CR_REG_START)
#define CMD_READ_BEGIN_REP (CMD_READ_BEGIN | CR_REG_REPEAT)
// Continuation of a previously started read operation. After performing this Read Data (from 1 to 255 bytes) operation, the bus remains busy (SCL = 0)
#define CMD_READ_CONTINUE (CR_REG_I2C_EN | CR_REG_RD)
// Completion of a previously started read operation. Ends with character Read data (1 to 255 bytes) completion (P)
#define CMD_READ_FINISH (CR_REG_I2C_EN | CR_REG_RD | CR_REG_STOP)
  
/* Initialization and Configuration functions *********************************/



/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_I2C_H */

/**
  * @}
  */
  
