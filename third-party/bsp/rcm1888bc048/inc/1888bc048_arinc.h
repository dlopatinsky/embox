//--------------------------------------------------------------
// @author  V. Syrtsov
//--------------------------------------------------------------

#ifndef __1888BC048_ARINC_H
#define __1888BC048_ARINC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
  
/******************************************************************************/
/*                         ARINC429 registers                                   */
/******************************************************************************/

/*--  CHANNEL_EN: channel enable register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t CH_EN_RX                   	:16;                         /*!< X */
  uint32_t CH_EN_TX                    	:16;                         /*!< X */
} _ARINC429_CHANNEL_EN_bits;

/* Bit field positions: */
#define ARINC429_CHANNEL_EN_RX_Pos            0              		 /*!< X */
#define ARINC429_CHANNEL_EN_TX_Pos            16              		 /*!< X */

/* Bit field masks: */
#define ARINC429_CHANNEL_EN_RX_Msk            0x0000FFFFUL           /*!< X */
#define ARINC429_CHANNEL_EN_TX_Msk            0xFFFF0000UL           /*!< X */

/*--  CHANNEL_DIS: channel disable register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t CH_DIS_RX                   	:16;                         /*!< X */
  uint32_t CH_DIS_TX                    :16;                         /*!< X */
} _ARINC429_CHANNEL_DIS_bits;

/* Bit field positions: */
#define ARINC429_CHANNEL_DIS_RX_Pos           0              		 /*!< X */
#define ARINC429_CHANNEL_DIS_TX_Pos           16              		 /*!< X */

/* Bit field masks: */
#define ARINC429_CHANNEL_DIS_RX_Msk           0x0000FFFFUL           /*!< X */
#define ARINC429_CHANNEL_DIS_TX_Msk           0xFFFF0000UL           /*!< X */

/*--  CHANNEL_RST: channel reset register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t CH_RST_RX                   	:16;                         /*!< X */
  uint32_t CH_RST_TX                    :16;                         /*!< X */
} _ARINC429_CHANNEL_RST_bits;

/* Bit field positions: */
#define ARINC429_CHANNEL_RST_RX_Pos           0              		 /*!< X */
#define ARINC429_CHANNEL_RST_TX_Pos           16              		 /*!< X */

/* Bit field masks: */
#define ARINC429_CHANNEL_RST_RX_Msk           0x0000FFFFUL           /*!< X */
#define ARINC429_CHANNEL_RST_TX_Msk           0xFFFF0000UL           /*!< X */

/*-- PARITY_BIT: channel parity bit control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t PARITY_BIT_RX                :16;                         /*!< X */
  uint32_t PARITY_BIT_TX                :16;                         /*!< X */
} _ARINC429_PARITY_BIT_bits;

/* Bit field positions: */
#define ARINC429_PARITY_BIT_RX_Pos            0              		 /*!< X */
#define ARINC429_PARITY_BIT_TX_Pos            16              		 /*!< X */

/* Bit field masks: */
#define ARINC429_PARITY_BIT_RX_Msk            0x0000FFFFUL           /*!< X */
#define ARINC429_PARITY_BIT_TX_Msk            0xFFFF0000UL           /*!< X */

/*-- PARITY_ODD: channel parity odd control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t PARITY_ODD_RX                :16;                         /*!< X */
  uint32_t PARITY_ODD_TX                :16;                         /*!< X */
} _ARINC429_PARITY_ODD_bits;

/* Bit field positions: */
#define ARINC429_PARITY_ODD_RX_Pos            0              		 /*!< X */
#define ARINC429_PARITY_ODD_TX_Pos            16              		 /*!< X */

/* Bit field masks: */
#define ARINC429_PARITY_ODD_RX_Msk            0x0000FFFFUL           /*!< X */
#define ARINC429_PARITY_ODD_TX_Msk            0xFFFF0000UL           /*!< X */

/*-- TEST_EN: channel parity odd control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t NUMBER_TX_TEST               :4;                         /*!< X */
  uint32_t TX_TEST_EN                	:1;                         /*!< X */
  uint32_t 								:27;
} _ARINC429_TEST_EN_bits;

/* Bit field positions: */
#define ARINC429_TEST_EN_NUMBER_TX_TEST_Pos        0              		 /*!< X */
#define ARINC429_TEST_EN_TX_TEST_EN_Pos            4              		 /*!< X */

/* Bit field masks: */
#define ARINC429_TEST_EN_NUMBER_TX_TEST_Msk        0x0000000FUL           /*!< X */
#define ARINC429_TEST_EN_TX_TEST_EN_Msk            0x00000010UL           /*!< X */

/*-- WAIT_TMR_TX: channel wait signal or timer event control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t WAIT_TX               		:16;                         /*!< X */
  uint32_t TIMER_TX                		:16;                         /*!< X */
} _ARINC429_WAIT_TMR_TX_bits;

/* Bit field positions: */
#define ARINC429_WAIT_TMR_WAIT_TX_Pos         0              		 /*!< X */
#define ARINC429_WAIT_TMR_TIMER_TX_Pos        16              		 /*!< X */

/* Bit field masks: */
#define ARINC429_WAIT_TMR_WAIT_TX_Msk         0x0000FFFFUL           /*!< X */
#define ARINC429_WAIT_TMR_TIMER_TX_Msk        0xFFFF0000UL           /*!< X */

/*-- AXI_CTRL: AXI control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t ARLOCK               		:2;                         /*!< X */
  uint32_t ARPROT                 		:3;                         /*!< X */
  uint32_t ARCACHE                  	:4;                         /*!< X */
  uint32_t AWLOCK                		:2;                         /*!< X */
  uint32_t AWPROT                  		:3;                         /*!< X */
  uint32_t AWCACHE                   	:4;                         /*!< X */  
  uint32_t 								:14;
} _ARINC429_AXI_CTRL_bits;

/* Bit field positions: */
#define ARINC429_AXI_CTRL_ARLOCK_Pos          0              		 /*!< X */
#define ARINC429_AXI_CTRL_ARPROT_Pos          2              		 /*!< X */
#define ARINC429_AXI_CTRL_ARCACHE_Pos         5              		 /*!< X */
#define ARINC429_AXI_CTRL_AWLOCK_Pos          9              		 /*!< X */
#define ARINC429_AXI_CTRL_AWPROT_Pos          11              		 /*!< X */
#define ARINC429_AXI_CTRL_AWCACHE_Pos         14              		 /*!< X */

/* Bit field masks: */
#define ARINC429_AXI_CTRL_ARLOCK_Msk          0x00000003UL           /*!< X */
#define ARINC429_AXI_CTRL_ARPROT_Msk          0x0000001CUL   		 /*!< X */
#define ARINC429_AXI_CTRL_ARCACHE_Msk         0x000001E0UL   		 /*!< X */
#define ARINC429_AXI_CTRL_AWLOCK_Msk          0x00000600UL   		 /*!< X */
#define ARINC429_AXI_CTRL_AWPROT_Msk          0x00003800UL    		 /*!< X */
#define ARINC429_AXI_CTRL_AWCACHE_Msk         0x0003C000UL    		 /*!< X */

/*-- LAST_WD_TMR: last timestamp capture control register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t LAST_WD_TMR             		:16;                         /*!< X */
  uint32_t 								:16;
} _ARINC429_LAST_WD_TMR_bits;

/* Bit field positions: */
#define ARINC429_LAST_WD_TMR_Pos          0              		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_LAST_WD_TMR_Msk         0x0000FFFFUL           	 /*!< X */

/*-- TX_DMA_DONE: TX DMA transfer is complete register ------------------------------------------------------------------------------*/
typedef struct {
	uint32_t TX_DMA_ALT_DONE			:16;
	uint32_t TX_DMA_PRM_DONE			:16;
} _ARINC429_TX_DMA_DONE_bits;

/* Bit field positions: */
#define ARINC429_TX_DMA_ALT_DONE_Pos        	0              		 	/*!< X */
#define ARINC429_TX_DMA_PRM_DONE_Pos         	16             		 	/*!< X */

/* Bit field masks: */
#define ARINC429_TX_DMA_ALT_DONE_Msk         	0x0000FFFFUL           /*!< X */
#define ARINC429_TX_DMA_PRM_DONE_Msk        	0xFFFF0000UL           /*!< X */

/*-- IR_CLR: clear interrapt register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t CLR             				:1;                         /*!< X */
  uint32_t 								:31;
} _ARINC429_IR_CLR_bits;

/* Bit field positions: */
#define ARINC429_IR_CLR_Pos          	0              		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_IR_CLR_Msk        	 	0x00000001UL           	 /*!< X */

/*-- IR_CTRL: enable long interrapt level register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t IRQn_LONG_EN            		:1;                         /*!< X */
  uint32_t 								:31;
} _ARINC429_IR_CTRL_bits;

/* Bit field positions: */
#define ARINC429_IR_CTRL_IRQn_LONG_EN_Pos          	0              		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_IR_CTRL_IRQn_LONG_EN_Msk        	0x00000001UL           	 /*!< X */

/*-- FREQ_TX: enable long interrapt level register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t FREQ            				:13;             /*!< X */
  uint32_t GAP            				:6;              /*!< X */
  uint32_t WORD_BITS_SIZE				:6;
  uint32_t 								:1;
  uint32_t AXI_PARITY_BIT_DIS			:1;
  uint32_t 								:2;
  uint32_t FREQ_UNLOCK					:1;
  uint32_t GAP_UNLOCK					:1;
  uint32_t WBS_AND_APBS_UNLOCK 			:1;				/*!< WORD_BITS_SIZE and AXI_PARITY_BIT_DIS unlock bit */
} _ARINC429_FREQ_TX_bits;

/* Bit field positions: */
#define ARINC429_FREQ_TX_FREQ_Pos          			0              		 	 /*!< X */
#define ARINC429_FREQ_TX_GAP_Pos          			13             		 	 /*!< X */
#define ARINC429_FREQ_TX_WORD_BITS_SIZE_Pos         19             		 	 /*!< X */
#define ARINC429_FREQ_TX_AXI_PARITY_BIT_DIS_Pos     26             		 	 /*!< X */
#define ARINC429_FREQ_TX_FREQ_UNLOCK_Pos         	29             		 	 /*!< X */
#define ARINC429_FREQ_TX_GAP_UNLOCK_Pos         	30             		 	 /*!< X */
#define ARINC429_FREQ_TX_WBS_AND_APBS_UNLOCK_Pos    31             		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_FREQ_TX_FREQ_Msk        			0x00001FFFUL           	 /*!< X */
#define ARINC429_FREQ_TX_GAP_Msk          			0x0007E000UL           	 /*!< X */
#define ARINC429_FREQ_TX_WORD_BITS_SIZE_Msk			0x01F80000UL           	 /*!< X */
#define ARINC429_FREQ_TX_AXI_PARITY_BIT_DIS_Msk     0x04000000UL           	 /*!< X */
#define ARINC429_FREQ_TX_FREQ_UNLOCK_Msk         	0x20000000UL           	 /*!< X */
#define ARINC429_FREQ_TX_GAP_UNLOCK_Msk         	0x40000000UL           	 /*!< X */
#define ARINC429_FREQ_TX_WBS_AND_APBS_UNLOCK_Msk    0x80000000UL           	 /*!< X */

/*-- FREQ_RX: enable long interrapt level register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t FREQ            				:13;             /*!< X */
  uint32_t OFFSET            			:13;             /*!< X */
  uint32_t PTIMER						:3;
  uint32_t FREQ_UNLOCK					:1;
  uint32_t OFFSET_UNLOCK				:1;
  uint32_t PTIMER_UNLOCK 				:1;				/*!< X */
} _ARINC429_FREQ_RX_bits;

/* Bit field positions: */
#define ARINC429_FREQ_RX_FREQ_Pos          			0              		 	 /*!< X */
#define ARINC429_FREQ_RX_OFFSET_Pos          		13             		 	 /*!< X */
#define ARINC429_FREQ_RX_PTIMER_Pos         		26             		 	 /*!< X */
#define ARINC429_FREQ_RX_FREQ_UNLOCK_Pos         	29             		 	 /*!< X */
#define ARINC429_FREQ_RX_OFFSET_UNLOCK_Pos         	30             		 	 /*!< X */
#define ARINC429_FREQ_RX_PTIMER_UNLOCK_Pos    		31             		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_FREQ_RX_FREQ_Msk        			0x00001FFFUL           	 /*!< X */
#define ARINC429_FREQ_RX_OFFSET_Msk          		0x03FFE000UL           	 /*!< X */
#define ARINC429_FREQ_RX_PTIMER_Msk					0x1C000000UL           	 /*!< X */
#define ARINC429_FREQ_RX_FREQ_UNLOCK_Msk         	0x20000000UL           	 /*!< X */
#define ARINC429_FREQ_RX_OFFSET_UNLOCK_Msk         	0x40000000UL           	 /*!< X */
#define ARINC429_FREQ_RX_PTIMER_UNLOCK_Msk    		0x80000000UL           	 /*!< X */

/*-- SELF_RX: enable long interrapt level register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t NUM_TX_CH       				:4;             /*!< X */
  uint32_t SELF_TEST_EN        			:1;             /*!< X */
  uint32_t DISABLED_DAT					:1;
  uint32_t 				 				:26;			/*!< X */
} _ARINC429_SELF_RX_bits;

/* Bit field positions: */
#define ARINC429_SELF_RX_NUM_TX_CH_Pos          	0              		 	 /*!< X */
#define ARINC429_SELF_RX_SELF_TEST_EN_Pos          	4             		 	 /*!< X */
#define ARINC429_SELF_RX_DISABLED_DAT_Pos         	5             		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_SELF_RX_NUM_TX_CH_Msk        		0x0000000FUL           	 /*!< X */
#define ARINC429_SELF_RX_SELF_TEST_EN_Msk          	0x00000010UL           	 /*!< X */
#define ARINC429_SELF_RX_DISABLED_DAT_Msk			0x00000020UL           	 /*!< X */

/*-- RX_FIFO_LABEL: receiver FIFO label register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t LABEL_0       				:8;             /*!< X */
  uint32_t LABEL_1       				:8;             /*!< X */
  uint32_t LABEL_2       				:8;             /*!< X */
  uint32_t LABEL_3       				:8;             /*!< X */
} _ARINC429_RX_FIFO_LABEL_bits;

/* Bit field positions: */
#define ARINC429_RX_FIFO_LABEL_0_Pos          		0              		 	 /*!< X */
#define ARINC429_RX_FIFO_LABEL_1_Pos          		8              		 	 /*!< X */
#define ARINC429_RX_FIFO_LABEL_2_Pos          		16              		 /*!< X */
#define ARINC429_RX_FIFO_LABEL_3_Pos          		24             		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_RX_FIFO_LABEL_0_Msk        		0x000000FFUL           	 /*!< X */
#define ARINC429_RX_FIFO_LABEL_1_Msk          		0x0000FF00UL           	 /*!< X */
#define ARINC429_RX_FIFO_LABEL_2_Msk				0x00FF0000UL           	 /*!< X */
#define ARINC429_RX_FIFO_LABEL_3_Msk				0xFF000000UL           	 /*!< X */

/*-- RX_CHANNEL_ASSIGN_FIFO: assign FIFO to channel register ------------------------------------------------------------------------------*/
typedef struct {
  uint32_t NUM_RX_CH       				:4;             /*!< X */
  uint32_t V_NUMBER        				:1;             /*!< X */
  uint32_t LABEL_QUANTITY				:4;
  uint32_t 				 				:23;			/*!< X */
} _ARINC429_RX_CHANNEL_ASSIGN_FIFO_bits;

/* Bit field positions: */
#define ARINC429_RX_CHANNEL_ASSIGN_FIFO_NUM_RX_CH_Pos   		0      		 	 /*!< X */
#define ARINC429_RX_CHANNEL_ASSIGN_FIFO_V_NUMBER_Pos          	4              		 	 /*!< X */
#define ARINC429_RX_CHANNEL_ASSIGN_FIFO_LABEL_QUANTITY_Pos      5              		 	 /*!< X */

/* Bit field masks: */
#define ARINC429_RX_CHANNEL_ASSIGN_FIFO_NUM_RX_CH_Msk        	0x0000000FUL           	 /*!< X */
#define ARINC429_RX_CHANNEL_ASSIGN_FIFO_V_NUMBER_Msk          	0x00000010UL           	 /*!< X */
#define ARINC429_RX_CHANNEL_ASSIGN_FIFO_LABEL_QUANTITY_Msk		0x000001E0UL           	 /*!< X */

typedef struct {
    __I uint32_t Reserved00;
  union {                                                               		/*!< channel enable register */
    __IO uint32_t CHANNEL_EN;                                          	 	/*!< CHANNEL_EN     : type used for word access */
    __IO _ARINC429_CHANNEL_EN_bits  CHANNEL_EN_bit;                     	/*!< CHANNEL_EN_bit : structure used for bit access */
  };
  union {                                                               		/*!< channel disable register */
    __O uint32_t CHANNEL_DIS;                                           	/*!< CHANNEL_DIS      : type used for word access */
    __O _ARINC429_CHANNEL_DIS_bits  CHANNEL_DIS_bit;                     	/*!< CHANNEL_DIS_bit : structure used for bit access */
  };
  union {                                                               		/*!< channel reset register */
    __O uint32_t CHANNEL_RST;                                           	/*!< CHANNEL_RST      : type used for word access */
    __O _ARINC429_CHANNEL_RST_bits  CHANNEL_RST_bit;                     	/*!< CHANNEL_RST_bit : structure used for bit access */
  };
  union {																		/*!< channel parity bit control register */
	__IO uint32_t PARITY_BIT;												/*!< PARITY_BIT      : type used for word access */
	__IO _ARINC429_PARITY_BIT_bits PARITY_BIT_bit;							/*!< PARITY_BIT_bit : structure used for bit access */
  };
  union {																		/*!< channel parity odd control register */
	__IO uint32_t PARITY_ODD;												/*!< PARITY_ODD      : type used for word access */
	__IO _ARINC429_PARITY_ODD_bits PARITY_ODD_bit;							/*!< PARITY_ODD_bit : structure used for bit access */
  };
  union {																		/*!< channel enable test mode register */
	__IO uint32_t TEST_EN;													/*!< TEST_EN      : type used for word access */
	__IO _ARINC429_TEST_EN_bits TEST_EN_bit;								/*!< TEST_EN_bit : structure used for bit access */
  }; 
  __IO uint32_t WAIT_SIG_RX;
  __IO uint32_t TMR_MASK_RX;
  __O uint32_t SW_SIG_RX;
  union {
	__IO uint32_t WAIT_TMR_TX;
	__IO _ARINC429_WAIT_TMR_TX_bits WAIT_TMR_TX_bit;						/*!< WAIT_TMR_TX_bit : structure used for bit access */
  };	
  __O uint32_t SW_SIG_TX;
  __IO uint32_t LABEL_EN_RX;
  union {
	__IO uint32_t AXI_CTRL;
	__IO _ARINC429_AXI_CTRL_bits AXI_CTRL_bit;								/*!< AXI_CTRL_bit : structure used for bit access */
  }; 
  union {
	__O uint32_t FIFO_DIS_RX;
	__I uint32_t FIFO_EN_RX;
  };
  union {
	__IO uint32_t LAST_WD_TMR;
	__IO _ARINC429_LAST_WD_TMR_bits LAST_WD_TMR_bit;								/*!< LAST_WD_TMR_bit : structure used for bit access */
  };  
  __IO uint32_t IENR_TX_DMA_DONE;
  __IO uint32_t IENR_RX_DMA_DONE;  
  __I uint32_t RX_DMA_PRM_DONE;
  __I uint32_t RX_DMA_ALT_DONE;
  union {
	  __I uint32_t TX_DMA_DONE;
	  __I _ARINC429_TX_DMA_DONE_bits TX_DMA_DONE_bit;	  
  };
  union {
	__O uint32_t IR_CLR;
	__O _ARINC429_IR_CLR_bits IR_CLR_bit;
  };
  union {
	__O uint32_t IR_CTRL;
	__O _ARINC429_IR_CTRL_bits IR_CTRL_bit;
  };
  __O uint32_t STUS_SW_TX;
  __O uint32_t STUS_SW_RX;
  __O uint32_t TEST_FRC_TS;
  __O uint32_t IR_EVENT_CTRL;  
	__I uint8_t Reserved01[0x0440-0x006C]; 
  union {
	__IO uint32_t FREQ_TX[16];
	__IO _ARINC429_FREQ_TX_bits FREQ_TX_bit[16];								/*!< FREQ_TX_bit : structure used for bit access */
  };  
	__I uint8_t Reserved02[0x0C40-0x0480]; 
  union {
	__IO uint32_t FREQ_RX[16];
	__IO _ARINC429_FREQ_RX_bits FREQ_RX_bit[16];								/*!< FREQ_RX_bit : structure used for bit access */
  };
  union {
	__IO uint32_t SELF_RX[16];
	__IO _ARINC429_SELF_RX_bits SELF_RX_bit[16];								/*!< SELF_RX_bit : structure used for bit access */
  };
	__I uint8_t Reserved03[0x1080-0x0CC0];	
  __O uint32_t TX_DMA_PRM_SRC[16];
	__I uint8_t Reserved04[0x1100-0x10C0]; 
  __O uint32_t TX_DMA_ALT_SRC[16];
	__I uint8_t Reserved05[0x1180-0x1140]; 
  __O uint32_t TX_DMA_PRM_SRC_SIZE[16];
	__I uint8_t Reserved06[0x1200-0x11C0];   
  __O uint32_t TX_DMA_ALT_SRC_SIZE[16];
	__I uint8_t Reserved07[0x1280-0x1240];
  __I  uint32_t TX_DMA_PRM_STATUS[16];
	__I uint8_t Reserved08[0x1300-0x12C0];
  __I  uint32_t TX_DMA_ALT_STATUS[16];
	__I uint8_t Reserved09[0x1380-0x1340];
  __I uint32_t TIMESTAMP_TX_DMA_PRM_FIRST_WD_LOW[16];
	__I uint8_t Reserved10[0x1400-0x13C0];
  __I uint32_t TIMESTAMP_TX_DMA_PRM_FIRST_WD_HI[16];	
	__I uint8_t Reserved11[0x1480-0x1440];
  __I uint32_t TIMESTAMP_TX_DMA_PRM_FAST_WD_LOW[16];
	__I uint8_t Reserved12[0x1500-0x14C0];
  __I uint32_t TIMESTAMP_TX_DMA_ALT_FIRST_WD_LOW[16];
	__I uint8_t Reserved13[0x1580-0x1540];
  __I uint32_t TIMESTAMP_TX_DMA_ALT_FIRST_WD_HI[16];
	__I uint8_t Reserved14[0x1600-0x15C0];  
  __I uint32_t TIMESTAMP_TX_DMA_ALT_FAST_WD_LOW[16];
	__I uint8_t Reserved15[0x1880-0x1640];
  __O uint32_t RX_DMA_PRM_SRC[32];
  __O uint32_t RX_DMA_ALT_SRC[32];
  __O uint32_t RX_DMA_PRM_SRC_SIZE[32];
  __O uint32_t RX_DMA_ALT_SRC_SIZE[32];
  __I  uint32_t RX_DMA_PRM_STATUS[32];
  __I  uint32_t RX_DMA_ALT_STATUS[32];
  __I uint32_t TIMESTAMP_RX_DMA_PRM_FIRST_WD_LOW[32];
  __I uint32_t TIMESTAMP_RX_DMA_PRM_FIRST_WD_HI[32];	
  __I uint32_t TIMESTAMP_RX_DMA_PRM_FAST_WD_LOW[32];
  __I uint32_t TIMESTAMP_RX_DMA_ALT_FIRST_WD_LOW[32];
  __I uint32_t TIMESTAMP_RX_DMA_ALT_FIRST_WD_HI[32];
  __I uint32_t TIMESTAMP_RX_DMA_ALT_FAST_WD_LOW[32];
  union {	
	__O uint32_t RX_FIFO_LABEL[32];
	__O _ARINC429_RX_FIFO_LABEL_bits RX_FIFO_LABEL_bit[32];
  };
  union {	
	__IO uint32_t RX_CHANNEL_ASSIGN_FIFO[32];
	__IO _ARINC429_RX_CHANNEL_ASSIGN_FIFO_bits RX_CHANNEL_ASSIGN_FIFO_bit[32];
  };
} ARINC429_TypeDef;



/** @addtogroup Peripheral_declaration
  * @{
  */

#define ARINC429             ((ARINC429_TypeDef *) ARINC429_BASE)

/**
  * @brief	ARINC429 Init Channel Structure definition
  */
  
typedef struct {
	uint32_t 		ARINC429_CLK;
	uint32_t		ARINC429_GAP;
	FunctionalState ARINC429_EN_PAR;
	uint32_t 		ARINC429_ODD;	
} ARINC429_InitChannelTypeDef;	

/** @defgroup ARINC429_CLK ARINC429 CLK
  * @{
  */

#define ARINC429_CLK_12_5_KHz		((uint32_t)0x00000F9E)
#define ARINC429_CLK_50_KHz			((uint32_t)0x000003E6)
#define ARINC429_CLK_100_KHz		((uint32_t)0x000001F2)

#define IS_ARINC429_CLK(CLK)		(((CLK) == ARINC429_CLK_12_5_KHz) ||\
									 ((CLK) == ARINC429_CLK_50_KHz) ||\
								     ((CLK) == ARINC429_CLK_100_KHz))
									 
/** @} */ /* End of group ARINC429_CLK */	

/** @defgroup ARINC429_PARITY ARINC429 Parity
  * @{
  */
  
#define ARINC429_PARITY_ENABLE		((uint32_t)0x00000001)
#define ARINC429_PARITY_DISABLE		((uint32_t)0x00000000)

#define IS_ARINC429_PARITY(PARITY)			(((PARITY) == ARINC429_PARITY_ENABLE) || \
											 ((PARITY) == ARINC429_PARITY_DISABLE))

/** @} */ /* End of group ARINC429_PARITY */  

/** @defgroup ARINC429_ODD ARINC429 ODD
  * @{
  */
#define ARINC429_ODD_ADD_TO_ODD				((uint32_t)0x00000001)
#define ARINC429_ODD_ADD_TO_PARITY			((uint32_t)0x00000000)

#define IS_ARINC429_ODD(ODD)				(((ODD) == ARINC429_ODD_ADD_TO_ODD) || \
											 ((ODD) == ARINC429_ODD_ADD_TO_PARITY))
/** @} */ /* End of group ARINC429_ODD */

/** @defgroup ARINC429_DMA_Switch ARINC429 DMA Switch
  * @{
  */

#define ARINC429_DMA_Switch_Immediately		((uint32_t)0x00000000)
#define ARINC429_DMA_Switch_On_SIG_Event	((uint32_t)0x00000001)
#define ARINC429_DMA_Switch_On_TMR_Event	((uint32_t)0x00000002)

#define IS_ARINC429_DMA_SWITCH(SWITCH)		(((SWITCH) == ARINC429_DMA_Switch_Immediately) ||\
											 ((SWITCH) == ARINC429_DMA_Switch_On_SIG_Event) ||\
											 ((SWITCH) == ARINC429_DMA_Switch_On_TMR_Event))
									 
/** @} */ /* End of group ARINC429_DMA_Switch */

/** @defgroup ARINC429_RX_GAP ARINC429 RX GAP
  * @{
  */
  
#define ARINC429_RX_GAP_32T					((uint32_t)0x00000000)
#define ARINC429_RX_GAP_40T					((uint32_t)0x00000001)
#define ARINC429_RX_GAP_48T					((uint32_t)0x00000002)
#define ARINC429_RX_GAP_56T					((uint32_t)0x00000003)
#define ARINC429_RX_GAP_64T					((uint32_t)0x00000004)
#define ARINC429_RX_GAP_72T					((uint32_t)0x00000005)
#define ARINC429_RX_GAP_80T					((uint32_t)0x00000006)
#define ARINC429_RX_GAP_ANY					((uint32_t)0x00000007)

#define IS_ARINC429_RX_GAP(GAP)			(((GAP) >= ARINC429_RX_GAP_32T ) &&\
								         ((GAP) <= ARINC429_RX_GAP_ANY ))

/** @} */ /* End of group ARINC429_RX_GAP */	

/** @defgroup ARINC429_TX_GAP ARINC429 TX GAP
  * @{
  */

#define IS_ARINC429_TX_GAP(GAP)			(((GAP) >= 4 ) &&\
								         ((GAP) <= 63 ))

/** @} */ /* End of group ARINC429_TX_GAP */

/** @defgroup ARINC429_TX_FIFO ARINC429 TX FIFO
  * @{
  */
#define ARINC429_TX_FIFO1			((uint32_t)0x00000000)
#define ARINC429_TX_FIFO2			((uint32_t)0x00000001)
#define ARINC429_TX_FIFO3			((uint32_t)0x00000002)
#define ARINC429_TX_FIFO4			((uint32_t)0x00000003)
#define ARINC429_TX_FIFO5			((uint32_t)0x00000004)
#define ARINC429_TX_FIFO6			((uint32_t)0x00000005)
#define ARINC429_TX_FIFO7			((uint32_t)0x00000006)
#define ARINC429_TX_FIFO8			((uint32_t)0x00000007)
#define ARINC429_TX_FIFO9			((uint32_t)0x00000008)
#define ARINC429_TX_FIFO10			((uint32_t)0x00000009)
#define ARINC429_TX_FIFO11			((uint32_t)0x0000000A)
#define ARINC429_TX_FIFO12			((uint32_t)0x0000000B)
#define ARINC429_TX_FIFO13			((uint32_t)0x0000000C)
#define ARINC429_TX_FIFO14			((uint32_t)0x0000000D)
#define ARINC429_TX_FIFO15			((uint32_t)0x0000000E)
#define ARINC429_TX_FIFO16			((uint32_t)0x0000000F)

#define IS_ARINC429_TX_FIFO(FIFO)		(((FIFO) >= ARINC429_TX_FIFO1 ) &&\
								         ((FIFO) <= ARINC429_TX_FIFO16 ))

/** @} */ /* End of group ARINC429_TX_FIFO */

/** @defgroup ARINC429_RX_FIFO ARINC429 RX FIFO
  * @{
  */
#define ARINC429_RX_FIFO1			((uint32_t)0x00000000)
#define ARINC429_RX_FIFO2			((uint32_t)0x00000001)
#define ARINC429_RX_FIFO3			((uint32_t)0x00000002)
#define ARINC429_RX_FIFO4			((uint32_t)0x00000003)
#define ARINC429_RX_FIFO5			((uint32_t)0x00000004)
#define ARINC429_RX_FIFO6			((uint32_t)0x00000005)
#define ARINC429_RX_FIFO7			((uint32_t)0x00000006)
#define ARINC429_RX_FIFO8			((uint32_t)0x00000007)
#define ARINC429_RX_FIFO9			((uint32_t)0x00000008)
#define ARINC429_RX_FIFO10			((uint32_t)0x00000009)
#define ARINC429_RX_FIFO11			((uint32_t)0x0000000A)
#define ARINC429_RX_FIFO12			((uint32_t)0x0000000B)
#define ARINC429_RX_FIFO13			((uint32_t)0x0000000C)
#define ARINC429_RX_FIFO14			((uint32_t)0x0000000D)
#define ARINC429_RX_FIFO15			((uint32_t)0x0000000E)
#define ARINC429_RX_FIFO16			((uint32_t)0x0000000F)
#define ARINC429_RX_FIFO17			((uint32_t)0x00000010)
#define ARINC429_RX_FIFO18			((uint32_t)0x00000011)
#define ARINC429_RX_FIFO19			((uint32_t)0x00000012)
#define ARINC429_RX_FIFO20			((uint32_t)0x00000013)
#define ARINC429_RX_FIFO21			((uint32_t)0x00000014)
#define ARINC429_RX_FIFO22			((uint32_t)0x00000015)
#define ARINC429_RX_FIFO23			((uint32_t)0x00000016)
#define ARINC429_RX_FIFO24			((uint32_t)0x00000017)
#define ARINC429_RX_FIFO25			((uint32_t)0x00000018)
#define ARINC429_RX_FIFO26			((uint32_t)0x00000019)
#define ARINC429_RX_FIFO27			((uint32_t)0x0000001A)
#define ARINC429_RX_FIFO28			((uint32_t)0x0000001B)
#define ARINC429_RX_FIFO29			((uint32_t)0x0000001C)
#define ARINC429_RX_FIFO30			((uint32_t)0x0000001D)
#define ARINC429_RX_FIFO31			((uint32_t)0x0000001E)
#define ARINC429_RX_FIFO32			((uint32_t)0x0000001F)

#define IS_ARINC429_RX_FIFO(FIFO)		(((FIFO) >= ARINC429_RX_FIFO1 ) &&\
								         ((FIFO) <= ARINC429_RX_FIFO32 ))

/** @} */ /* End of group ARINC429_RX_FIFO */

/**
  * @brief	ARINC429 FIFO DMA Status Structure definition
  */

typedef struct
{
	uint32_t ACTUAL_QUANTITY		:26;
	uint32_t RD_WR_COMPLT			:1;
	uint32_t ERROR_DATA			:1;
	uint32_t WAIT_SIG_EVENT			:1;
	uint32_t SIG_EVENT			:1;
	uint32_t BUSY				:1;
	uint32_t TRANSFER_SUCCESS		:1;
} ARINC429_FIFO_DMA_StatusStruct_TypeDef;

/** @} */ /* End of group ARINC429 FIFO DMA Status */

/**
  * @brief	ARINC429 FIFO DMA Structure definition
  */

typedef struct
{
	__IO uint32_t * SRC;
	__IO uint32_t SRC_WORD_SIZE;
} ARINC429_FIFO_DMA_ConfigStruct_TypeDef;

/** @} */ /* End of group ARINC429 FIFO DMA */

/**
  * @brief	ARINC429 FIFO DMA Data Structure definition
  */

typedef struct
{
    ARINC429_FIFO_DMA_ConfigStruct_TypeDef PRM_DATA;  /*!< Основная управляющая структура */
    ARINC429_FIFO_DMA_ConfigStruct_TypeDef ALT_DATA;  /*!< Альтернативная управляющая структура */
} ARINC429_FIFO_DMA_ConfigData_TypeDef;

/** @defgroup ARINC429_DMA_definition ARINC429 DMA definition
  * @{
  */

#define ARINC429_DMA_PRM				((uint32_t)0x00000000)
#define ARINC429_DMA_ALT				((uint32_t)0x00000001)

#define IS_ARINC429_DMA(DMA)			(((DMA) == ARINC429_DMA_PRM) || \
										 ((DMA) == ARINC429_DMA_ALT))

/** @} */ /* End of group ARINC429_DMA_definition */

/** @defgroup ARINC429_DMA_flags_definition ARINC429 DMA flags definition
  * @{
  */

#define ARINC429_DMA_FLAG_PRM_DONE		((uint32_t)0x00000000)
#define ARINC429_DMA_FLAG_ALT_DONE		((uint32_t)0x00000001)

#define IS_ARINC429_DMA_FLAG(FLAG)		(((FLAG) == ARINC429_DMA_FLAG_PRM_DONE) || \
										 ((FLAG) == ARINC429_DMA_FLAG_ALT_DONE))

/** @} */ /* End of group ARINC429_DMA_flags_definition */

/** @defgroup ARINC429_Channel ARINC429 Channel
  * @{
  */
#define ARINC429_CHANNEL1			((uint32_t)0x00000000)
#define ARINC429_CHANNEL2			((uint32_t)0x00000001)
#define ARINC429_CHANNEL3			((uint32_t)0x00000002)
#define ARINC429_CHANNEL4			((uint32_t)0x00000003)
#define ARINC429_CHANNEL5			((uint32_t)0x00000004)
#define ARINC429_CHANNEL6			((uint32_t)0x00000005)
#define ARINC429_CHANNEL7			((uint32_t)0x00000006)
#define ARINC429_CHANNEL8			((uint32_t)0x00000007)
#define ARINC429_CHANNEL9			((uint32_t)0x00000008)
#define ARINC429_CHANNEL10			((uint32_t)0x00000009)
#define ARINC429_CHANNEL11			((uint32_t)0x0000000A)
#define ARINC429_CHANNEL12			((uint32_t)0x0000000B)
#define ARINC429_CHANNEL13			((uint32_t)0x0000000C)
#define ARINC429_CHANNEL14			((uint32_t)0x0000000D)
#define ARINC429_CHANNEL15			((uint32_t)0x0000000E)
#define ARINC429_CHANNEL16			((uint32_t)0x0000000F)

#define IS_ARINC429_CHANNEL(CHANNEL)		(((CHANNEL) >= ARINC429_CHANNEL1 ) &&\
									         ((CHANNEL) <= ARINC429_CHANNEL16 ))

/** @} */ /* End of group ARINC429_Channel */

/** @defgroup ARINC429TX_Exported_Funstions ARINC429TX Exported Funstions
  * @{
  */

void ARINC429TX_ChannelDeInit(uint32_t ARINC429_CHANNELx);
void ARINC429TX_ChannelInit(uint32_t ARINC429_CHANNELx, ARINC429_InitChannelTypeDef * ARINC429_InitChanelStruct);
void ARINC429TX_ChannelCmd(uint32_t ARINC429_CHANNELx, FunctionalState NewState);
void ARINC429TX_FIFOForceSwitchDMA(uint32_t ARINC429_FIFOx);
void ARINC429TX_DMAInit(uint32_t ARINC429_FIFOx, ARINC429_FIFO_DMA_ConfigData_TypeDef* DMADscr);
void ARINC429TX_DMASwitchModeConfig(uint32_t ARINC429_FIFOx, uint32_t Mode);
FlagStatus ARINC429TX_DMA_GetFlagStatus(uint32_t ARINC429_FIFOx, uint32_t DMA_Flag);
ARINC429_FIFO_DMA_StatusStruct_TypeDef * ARINC429TX_DMA_GetStatus(uint32_t ARINC429_FIFOx, uint32_t DMA);
void ARINC429TX_ITConfig(uint32_t ARINC429_FIFOx, FunctionalState NewState);

/** @} */ /* End of group ARINC429TX_Exported_Funstions */

/** @defgroup ARINC429RX_Exported_Funstions ARINC429RX Exported Funstions
  * @{
  */

void ARINC429RX_ChannelDeInit(uint32_t ARINC429_CHANNELx);
void ARINC429RX_ChannelInit(uint32_t ARINC429_CHANNELx, ARINC429_InitChannelTypeDef * ARINC429_InitChanelStruct);
void ARINC429RX_ChannelCmd(uint32_t ARINC429_CHANNELx, FunctionalState NewState);
void ARINC429RX_FIFOAssignChannel(uint32_t ARINC429_FIFOx, uint32_t ARINC429_CHANNELx);
void ARINC429RX_FIFOLabelCmd(uint32_t ARINC429_FIFOx, FunctionalState NewState);
void ARINC429RX_FIFOLoadLabel(uint32_t ARINC429_FIFOx, uint8_t *Label, uint32_t Size);
void ARINC429RX_FIFOCmd(uint32_t ARINC429_FIFOx, FunctionalState NewState);
void ARINC429RX_FIFOForceSwitchDMA(uint32_t ARINC429_FIFOx);
void ARINC429RX_DMAInit(uint32_t ARINC429_FIFOx, ARINC429_FIFO_DMA_ConfigData_TypeDef* DMADdst);
void ARINC429RX_DMASwitchModeConfig(uint32_t ARINC429_FIFOx, uint32_t Mode);
FlagStatus ARINC429RX_DMA_GetFlagStatus(uint32_t ARINC429_FIFOx, uint32_t DMA_Flag);
ARINC429_FIFO_DMA_StatusStruct_TypeDef * ARINC429RX_DMA_GetStatus(uint32_t ARINC429_FIFOx, uint32_t DMA);
void ARINC429RX_ITConfig(uint32_t ARINC429_FIFOx, FunctionalState NewState);;

/** @} */ /* End of group ARINC429RX_Exported_Funstions */


/** @defgroup ARINC429_Exported_Funstions ARINC429 Exported Funstions
  * @{
  */

void ARINC429_SetLoopback(uint32_t ARINC429TX_CHANNELx, uint32_t ARINC429RX_CHANNELx);
void ARINC429_ResetLoopback(uint32_t ARINC429TX_CHANNELx, uint32_t ARINC429RX_CHANNELx);
void ARINC429_ITPendClear(void);

/** @} */ /* End of group ARINC429_Exported_Funstions */

//
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_ARINC_H */
/**
  * @}
  */
  
