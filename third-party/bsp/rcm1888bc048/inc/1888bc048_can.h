/**
  ******************************************************************************
  * @file    1888bc048_can.h
  * @author  A. Maksimov, V. Syrtsov
  * @version V1.0.1
  * @date    05.12.2019
  * @brief   This file contains all the functions prototypes for the CAN 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_CAN_H
#define __1888BC048_CAN_H

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
//                              FDIV  SJW TS1 TS2  
#define CAN_TRT_1M    (CAN_TRT_TypeDef){ 3,    0,  12, 1}  
#define CAN_TRT_800K  (CAN_TRT_TypeDef){ 4,    0,  12, 1}  
#define CAN_TRT_500K  (CAN_TRT_TypeDef){ 7,    0,  12, 1}  
#define CAN_TRT_250K  (CAN_TRT_TypeDef){15,    0,  12, 1}  
#define CAN_TRT_125K  (CAN_TRT_TypeDef){31,    0,  12, 1}  
#define CAN_TRT_100K  (CAN_TRT_TypeDef){39,    0,  12, 1} 
#define CAN_TRT_50K   (CAN_TRT_TypeDef){63,    0,  15, 2}
  
/** @addtogroup   
  * @{
  */
//
#define CAN_TX_REPORT_ID  0x5AD912C3 /*!<     */
#define CAN_BUS_STATUS_ID 0x7E39B6E8 /*!<   */
#define CAN_RX_MSG_ID     0x92DC79A5 /*!<   */ 
//  CAN
#define CAN_STATUS_OK     1
#define CAN_STATUS_ERR    0
#define CAN_DATA_BUF_LEN  8
#define CAN_TQ_BUF_NUM    4
#define CAN_FILTER_IDE    (1UL<<29)

/** @defgroup Init_Mode_Change_Request_enumeration 
  * @{
  */
typedef enum{
  IMCR_NoRequest,   // 0          :
  IMCR_InitIn,      // 1       ; 
  IMCR_InitOut,     // 2       
}IMCR_TypeDef;

/** @defgroup BEC_enumeration 
  * @{
  */
typedef enum{
  BEC_NoError,      // 0   
  BEC_6_bits,       // 1       ,   ,     
  BEC_FormatErr,    // 2     . //   2: //          End of frame,    
  BEC_NoAck,        // 3 -        ACK SLOT
  BEC_RecessErr,    // 4 - ,   ,     
  BEC_DominantErr,  // 5 - ,   ,       //   4-5: // -      ,    ,     (ARBITRATION FIELD)  ACK SLOT,    ; // -   ,  Passive error flag,   ,    
  BEC_CRC_Err,      // 6  CRC,  ,    
}BEC_TypeDef;

/**
  * @brief CAN_FILTER structure definition  
  */  
typedef struct{
  __IO uint32_t Extend:     18; // 0  ID     x
  __IO uint32_t Standard:   11; // 0  ID     x
  __IO uint32_t Format:     1;  // 0  IDE ( )   x
  uint32_t : 2;// 
}CAN_Filter_TypeDef;

/**
  * @brief CAN_TRT structure definition  
  */  
typedef struct{
  uint32_t FDIV:       6; // 1	,         Tq.      
  uint32_t SJW:        2; // 0	      (ISO 11898-1, 12.4.1.1)
  uint32_t TS1:        4; // (Timing Segment 1)	3	      CAN, sample point
  uint32_t TS2:        2; // (Timing Segment 2)	2	      CAN,            sample point
  uint32_t :           17;// 
}CAN_TRT_TypeDef;

/**
  * @brief CAN_FILTER_MASK structure definition  
  */  
typedef struct{
  __IO CAN_Filter_TypeDef Filter;
  __IO CAN_Filter_TypeDef Mask;
}CAN_FILTER_MASK_TypeDef;

/**
  * @brief CAN_IDENTIFIER union definition  
  */  
typedef union{
  struct{
    uint32_t  :           1;  
    uint32_t  RTR:        1; //
    uint32_t  :           18;
    uint32_t  bID:        11; //  
    uint32_t  IDE:        1;  //   
  }Std;
  struct{
    uint32_t  :           1;  
    uint32_t  RTR:        1; //
    uint32_t  ID:        29; //  
    uint32_t  IDE:        1;  //   
  }Extd;
  uint32_t  Num;
}CAN_IDENTIFIER_TypeDef;

/**
  * @brief CAN_TX_REPORT_structure definition  
  */  
  
typedef struct{
  uint32_t TxID;            //    
//  
  uint32_t TxResult:    4;  //    CAN_TRSL_TypeDef
  uint32_t :           28;
//  
  uint32_t :            1;
  uint32_t RTR:         1; //???
  uint32_t ID:         29; //  
  uint32_t IDE:         1; //   
//  
  uint32_t PID;             //  
  uint32_t TS[2];           //  .
  uint32_t  :     32;
  uint32_t  :     32;
}CAN_TX_REPORT_TypeDef;

/** @defgroup CAN_TRSL_enumeration 
  * @{
  */
typedef enum{
  TRSL_OK = 1,    // 1	 
  TRSL_TTimeOut,  // 2	    
  TRSL_Break,     // 3	 ;      ,            
  TRSL_Clean,     // 4	 ;      ,              
  TRSL_Init,      // 5	 ;      ,              
  TRSL_BusOff,    // 6	 ;      ,         Bus Off
  TRSL_SingleShot,// 7	    Single Shot
  TRSL_Format,    // 8	    
}CAN_TRSL_TypeDef;

/**
  * @brief CAN_DATA_TX structure definition  
  */  
typedef struct{
  uint32_t  TTime[2];       //      
  CAN_IDENTIFIER_TypeDef MsgId;
  uint32_t  :           25;
  uint32_t  TxReport:   1;  //      
  uint32_t  RxOwnFrame: 1;  //         
  uint32_t  SingleShot: 1;  //    
  uint32_t  DLC:        4;  //   
//
  uint32_t Data[2];         //  
  uint32_t PID;             //  
  uint32_t  :     32;
}CAN_DATA_TX_TypeDef;

/**
  * @brief CAN_DATA_RX structure definition  
  */  
typedef struct{
  uint32_t RxID; //   
  uint32_t PID; //  
  CAN_IDENTIFIER_TypeDef MsgId;
  uint32_t  :           27;
  uint32_t  OwnFrame:   1;  //        
  uint32_t  DLC:        4;  //   
  uint32_t  Data[2];        //  
  uint32_t  TTime[2];       // ,       SOF
}CAN_DATA_RX_TypeDef;

/** @defgroup TX_TEST_enumeration 
  * @{
  */
typedef enum{//    Tx: 
  TX_TEST_Self = 0,     // 0    ; 
  TX_TEST_SamplePoint,  // 1  Sample Point   Tx;
  TX_TEST_Dominant,     // 2  Tx    (Tx=0); 
  TX_TEST_Recessive,    // 3  Tx    (Tx=1)
}CAN_TX_TEST_TypeDef;

/** @defgroup RX_TEST_enumeration 
  * @{
  */
typedef enum{//     Rx:
  RX_TEST_Dominant = 0,
  RX_TEST_Recessive = 1,
}CAN_RX_TEST_TypeDef;

/** @defgroup SOFT_RESET_enumeration 
  * @{
  */
typedef enum{//     Rx:
  SOFT_RESET_On = 5,
  SOFT_RESET_Off = 0,
}CAN_SOFT_RESET;

/** @defgroup SOFT_RESET_enumeration 
  * @{
  */
typedef enum{LARB, TER, RER, EWRN, EAEP, EPEA, BO, BOR, BI}CAN_BSE_TypeDef;

/**
  * @brief CAN structure definition  
  */

typedef struct{
  struct{     
    __I uint32_t InitMode:        1;  // 0 RO	1	  // 1   // 0  
    __I uint32_t :        	      5;  // 
    __IO uint32_t TRCE:           1;  // 0   ,        // 1   // 0   //     
    __IO uint32_t TME:            1;  // 0    // 1   // 0   //     
    uint32_t :                    24; // 
  }Mode;
  __IO uint32_t  IMCR;    /*!<    Init Mode */  
  struct{
    __IO uint32_t BusError:       1;  //      ,     Status Register
    __IO uint32_t :               2;  // 
    __IO uint32_t TxOk:           1;  //      
    __IO uint32_t RxOk:           1;  //      
    __IO uint32_t ErrorPassive:   1;  //      Error Passive
    __IO uint32_t WarningLevel:   1;  //         (TEC  REC)  
    __IO uint32_t BusOff:         1;  //      Bus Off
    __IO uint32_t BusIdle:        1;  //           
    __IO uint32_t Global:         1;  //   
    uint32_t :                    22;	// 
  }IrqEnable;             /*!<    */
  struct{    
    uint32_t :                  3;	// 
    __IO uint32_t Silence:      1;  // 0	   //         
    __IO uint32_t LoopBack:     1;  // 0	  //         
    __I  uint32_t Tx:           2;  // 0	   Tx (. CAN_TX_TEST_TypeDef)
    __I  uint32_t Rx:           1;  // 0	    Rx (. CAN_RX_TEST_TypeDef)
    uint32_t :                  24;	// 
  }Test;                  /*!< Test Register(0x000C) */
  struct{
  __IO uint32_t ArbitrErr:    1; //       
  __IO uint32_t TxErr:        1; //         
  __IO uint32_t RxErr:        1; //         
  __IO uint32_t WarningLevel: 1; //          
  __IO uint32_t ErrorPassive: 1; //         Error active   Error passive
  __IO uint32_t ErrorActive:  1; //         Error passive   Error active
  __IO uint32_t BusOff:       1; //          -    Bus Off
  __IO uint32_t BusRecovery:  1; //          Bus Off
  __IO uint32_t BusIdle:      1; //               
  uint32_t :                  23;// 
  }BSE;                       /*!< 0x0010	Bus Status Enable Register	       */
  __IO CAN_TRT_TypeDef TRT;   /*!< 0x0014	TR Timing Register	        (sample point).     CAN Mode Register[IM] = 1    CAN Mode Register[TRCE] = 1 */
  __IO uint32_t DIVE;         /*!< 0x0018	DIV Extension Register	   DIV    TR Timing Register[DIV].       1024*/
  __IO uint32_t SoftReset;    /*!< 0x001C	Soft Reset Register	  .   5        .       . Soft Reset      */
  struct{
    __IO uint32_t EID:        1; // (Extension ID) 0- ; 1- 
    __IO uint32_t DLCC:       1; // (Data Length Code Check)	0- ; 1- 
    uint32_t : 30;
  }TQF;                         /*!< 0x0020	TQ Format Enable	       */
  __IO uint32_t TxErrCnt;       /*!< 0x0024	Overall Transmit Error Counter	    */
  __IO uint32_t RxErrCnt;       /*!< 0x0028	Overall Receive Error Counter	    */
  __IO uint32_t WarningLevelCnt;/*!< 0x002C	TR Warning Level Counter	      */
  __IO uint32_t ErrorPassiveCnt;/*!< 0x0030	Error Passive Counter	    Error Passive  */
  __IO uint32_t BusOffCnt;      /*!< 0x0034	Bus Off Counter	    Bus Off */
  __IO uint32_t TxCnt;          /*!< 0x0038	Successful Transmit Counter	    */
  __IO uint32_t TimeoutCnt;     /*!< 0x003C	Timeout Frames Counter	      */
  __IO uint32_t SSFErrCnt;      /*!< 0x0040	Single Shot Frames Error Counter	    Single Shot */
  __IO uint32_t RxCnt;          /*!< 0x0044	Successful Receive Counter	    */
  __IO uint32_t EQ_RxCnt;       /*!< 0x0048	Overflow EQ RM Counter	         */
  struct{
    __IO uint32_t BusError:         3; //      CAN (. BEC_TypeDef)
    __IO uint32_t TxOk:             1; //   : 1        ; 0  
    __IO uint32_t RxOK:             1; //   : 1        
    __I  uint32_t ErrorPassive:     1; // 1   CAN   Error Passive, 0   CAN   Error Active
    __I  uint32_t WarningLevel:     1; //        
    __I  uint32_t BusOff:           1; // C Bus Off.
    __IO uint32_t BusIdle:          1; //          .
    uint32_t :                      23;// 
  }Status;                /*!< Status Register(0x004C) */
  struct{
    __I  uint32_t BusError:         3; //      CAN (. BEC_TypeDef)
    __I  uint32_t TxOk:             1; //   : 1        ; 0  
    __I  uint32_t RxOK:             1; //   : 1        
    __I  uint32_t ErrorPassive:     1; // 1   CAN   Error Passive, 0   CAN   Error Active
    __I  uint32_t WarningLevel:     1; //        
    __I  uint32_t BusOff:           1; // C Bus Off.
    __I  uint32_t BusIdle:          1; //          .
    uint32_t :                 23;// 31:9			
  }IrqFlag;                /*!< Interrupt Flag Register(0x0050) */
  struct{
    uint32_t :                      5; // 
    __IO uint32_t TxMode:           2; // 0	     Tx: 0  Tx   ; // 1  Sample Point   Tx; 2  Tx    (Tx=0); 3  Tx    (Tx=1)
    __IO uint32_t TxRequest:        1; // 0	       Tx. // 1       .  Tx      Tx Test Request[Tx]      // 0            
    uint32_t :                      24;// 
  }TxTestRequest;                 /*!< Tx Test Request(0x0054) */
  __IO uint32_t EQ_TxCnt;         /*!< 0x0058	Overflow EQ TR Counter	        .     */
  __IO uint32_t EQ_StateCnt;      /*!< 0x005C	          .     */
  __IO uint32_t GTC;              /*!< Global Transmit Cancel(0x0060) */  
  __IO uint32_t GTCCnt;           /*!< Global Transmit Cancel Counter(0x0064) */
  __I  uint32_t res0[2]; //  0x0068-0x006
  struct{
    __I  uint32_t TxErrCnt:         8; //   
    __I  uint32_t RxErrCnt:         7; //   
    __I  uint32_t ErrorPassive:     1; //        Error Passive.
    uint32_t :                 16; // 
  }StdErrCnt;                     /*!< Standard Error Counters Register(0x0070) */
  __IO uint32_t TQFErrCnt;        /*!< TQ Format Error Counter(0x0074) */
  __IO uint32_t TCIMCnt;          /*!< Transmit Cancel Init Mode Counter(0x0078) */
  __IO uint32_t TCBOCnt;          /*!< Transmit Cancel Bus Off Counter(0x007C) */
  struct{//  
    __IO uint32_t Size;     // 
    __IO uint32_t Control;  // 
    __I  uint32_t EFCnt;    //   
    __I  uint32_t WaterMark;
  }TQ[CAN_TQ_BUF_NUM];        //  0x0080 - 0x00BC 
  __I  uint32_t res1[16];     //  0x00C0-0x0FC
  __IO uint32_t SizeEQ;       // 0x0100
  __I  uint32_t FrameCntEQ;   // 0x0104
  __I  uint32_t WaterMarkEQ;  // 0x0108
  __I  uint32_t res2[61];     //  0x010-0x01F
  struct{
    uint32_t Filter, Mask;
  }FilterMask[16];            // 0x0200
  uint32_t res3[864];         // 
  __O uint32_t DataTQ[CAN_TQ_BUF_NUM][1024]; //  : 0x1000-0x1FFF //0x2000-0x2FFF //0x3000-0x3FFF //0x4000-0x4FFF
  union {
  __IO uint32_t DataEQ[1024]; //  : 0x5000-0x5FFF
  __IO CAN_DATA_RX_TypeDef RX[128]; // Канал чтения данных из очереди событий 0 : 0x5000-0x5FFF
  };
}CAN_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  

#define CAN0              ((CAN_TypeDef *)CAN0_BASE)
#define CAN1              ((CAN_TypeDef *)CAN1_BASE)
  
/* Initialization and Configuration functions *********************************/
extern uint32_t *CAN_GetTRT(CAN_TypeDef *CANx);
extern void CAN_SetTRT(CAN_TypeDef *CANx, CAN_TRT_TypeDef trt);
/* CAN Read and Write functions **********************************************/
extern uint32_t *CAN_ReadRxBuf(CAN_TypeDef *CANx);
extern uint32_t CAN_WriteTxBuf(CAN_TypeDef *CANx, uint32_t num, uint32_t *buf);
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_CAN_H */
/**
  * @}
  */
  
