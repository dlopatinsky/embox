/**
  ******************************************************************************
  * @file    1888bc048_sctl.h
  * @author  A. Maksimov 
  * @version V1.0.1
  * @date    09.12.2019
  * @brief   This file contains all the functions prototypes for the SCTL 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_SCTL_H
#define __1888BC048_SCTL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
    
/** @defgroup 
  * @{
  */
#define SCTL_RST_UNLOCK     (0x526D) /*!>     SYS_RST. */

/**
  * @brief PUPD enumerator structure definition  
  */
  
typedef enum{
  PUPD_HighZ = 0, //  
  PUPD_PullUp,    //   
  PUPD_PullDown,  //   
}PUPD_Typedef;

typedef enum{//   /   
  SR_Slow = 0,  SR_Fast,
}SR_Typedef;

typedef enum{//       
  SM_Off = 0,  SM_On,
}SM_Typedef;

/**
  * @brief      SCTL_CFG...
  */

#define BM(x)   uint32_t:x      /*    */
#define BM_SM   uint32_t sm:1
#define BM_SR   uint32_t sr:1
#define BM_PUPD uint32_t pupd:2
//     \...
// sr - ; sm -  ; pupd -  
//            |  --   |  sr  |  sm  | --  |  pupd   |  --   |
typedef struct{BM( 1); BM_SR;                        BM(30);} SCTL_SR0;
typedef struct{BM( 9); BM_SR;                        BM(22);} SCTL_SR1;
typedef struct{BM( 2);        BM_SM;                 BM(29);} SCTL_SM0;
typedef struct{BM(10);        BM_SM;                 BM(21);} SCTL_SM1;
typedef struct{BM( 4);                      BM_PUPD; BM(26);} SCTL_PUPD0;
typedef struct{BM(12);                      BM_PUPD; BM(18);} SCTL_PUPD1;
typedef struct{BM( 2);        BM_SM; BM(1); BM_PUPD; BM(26);} SCTL_PUPD_SM0;
typedef struct{BM(10);        BM_SM; BM(1); BM_PUPD; BM(18);} SCTL_PUPD_SM1;
typedef struct{BM( 1); BM_SR; BM_SM; BM(1); BM_PUPD; BM(26);} SCTL_CFG0;
typedef struct{BM( 9); BM_SR; BM_SM; BM(1); BM_PUPD; BM(18);} SCTL_CFG1;
typedef struct{BM(17); BM_SR; BM_SM; BM(1); BM_PUPD; BM(10);} SCTL_CFG2;
typedef struct{BM(25); BM_SR; BM_SM; BM(1); BM_PUPD; BM( 2);} SCTL_CFG3;
//
typedef  union{SCTL_CFG0 mdc;SCTL_CFG1 mdio;SCTL_CFG2 intn;SCTL_CFG3 rstn;}SCTL_MDIO;
typedef  struct{
  union{SCTL_CFG0 pin0;SCTL_CFG1 pin1;SCTL_CFG2 pin2;SCTL_CFG3 pin3;};
  union{SCTL_CFG0 pin4;SCTL_CFG1 pin5;SCTL_CFG2 pin6;SCTL_CFG3 pin7;};
}SCTL_GPIO;
//
/**
  * @brief SCTL_IOCFG structure definition  
  */

//   IO 
typedef struct{
  union{
    union{
      SCTL_SR0  sck;
      SCTL_CFG0 mosi, miso, irq, dat2, scs;
    }spi0;
    union{
      SCTL_SR1  sck;
      SCTL_CFG1 mosi, miso, irq, dat2, scs;
    }spi1;
  };//IOCFG0
  union{
    union{
      SCTL_SR0   tx;
      SCTL_PUPD0 rx;
    }can0;
    union{
      SCTL_SR1   tx;
      SCTL_PUPD1 rx;
    }can1;    
  };//IOCFG1
  union{
    union{
      SCTL_SR0   txa, txan, txb, txbn, txinha, txinhb;
      SCTL_PUPD0 rxa, rxan, rxb, rxbn;      
    }mk0;
    union{
      SCTL_SR1   txa, txan, txb, txbn;
      SCTL_PUPD1 rxa, rxan, rxb, rxbn;
    }mk1;    
  };//IOCFG2
  union{
    union{
      SCTL_SR0 inha, inhb;
    }mk1_tx;
    union{
      SCTL_SR0 txa, txb, testa, testb;
      SCTL_PUPD0  rxa, rxb;    
    }arnc;
  };//IOCFG3
  union{
    union{
      SCTL_SR0 txd, rts;
      SCTL_PUPD0  rxd, cts;    
    }uart0;
    union{ 
      SCTL_SR1 txd, rts;
      SCTL_PUPD1  rxd, cts;  
    }uart1;
  };//IOCFG4
  SCTL_MDIO mdio0; //IOCFG5
  SCTL_MDIO mdio1; //IOCFG6
  SCTL_MDIO mdio2; //IOCFG7
  SCTL_MDIO mdio3; //IOCFG8
  SCTL_GPIO gpio0; //IOCFG9//IOCFG10
  SCTL_GPIO gpio1; //IOCFG11//IOCFG12
  SCTL_GPIO gpio2; //IOCFG13//IOCFG14
  struct{
    union{
      SCTL_CFG0 _0;
      SCTL_CFG1 _1;
      SCTL_CFG2 _2;
      SCTL_CFG3 _3;
    };
    union{
      SCTL_CFG0 _4;
      SCTL_CFG1 _5;
      SCTL_CFG2 _6;
      SCTL_CFG3 _7;
    };
    union{
      SCTL_CFG0 _8;
      SCTL_CFG1 _9;
      SCTL_CFG2 _10;
      SCTL_CFG3 _11;
    };
    union{
      SCTL_CFG0 _12;
      SCTL_CFG1 _13;
      SCTL_CFG2 _14;
      SCTL_CFG3 _15;
    };
  }arnc_slp;//IOCFG15...IOCFG18
  union{
    union{
      SCTL_SR0      tdo;
      SCTL_CFG0     tms;
      SCTL_PUPD_SM0 tdi, tck, trstn;
    }dbg;
    union{
      SCTL_SR1      tdo;
      SCTL_PUPD_SM1 tms, tdi, tck, trstn;
    }dft;
  };//IOCFG19
  union{
    SCTL_SR0 wdt_irq, ext_irq;
  };//IOCFG20
  union{
    SCTL_SM0 sda0, scl0;
    SCTL_SM1 sda1, scl1;
  }i2c;//IOCFG21
}SCTL_IOCFG_TypeDef;

/**
  * @brief SCTL structure definition  
  */

typedef struct{  
  __I uint32_t BOOTM;               // 0x000     BOOTM[2:0]
  struct{
    __I uint32_t WFI: 1;
    __I uint32_t WFE: 1;
    uint32_t:30;
  }ARM_STANDBY_STAT;    // 0x004	  STANDBY  ARM
  uint8_t  res00[0x014-0x008];  // 0x008	
  struct{
    __IO uint32_t PHY_EN: 1;  // PLL_LOCK
    uint32_t : 3;
    __I uint32_t PHY_RDY: 4;  //    ( )
    __I uint32_t PLL_LOCK: 1; //   PLL  SGMII_PHY
    uint32_t WFE: 23;
  }SGMII_CTRL_STAT;     // 0x014	    SGMII_PHY
  uint8_t  res01[0x020-0x018];  // 0x018	
  struct{
    __IO uint32_t Request: 1;  //       CRG
    uint32_t : 15;
    __IO uint32_t Lock: 16;  //0x0001	    Request.    : 0x0001    0x0000   
  }SYS_RST;             // 0x020	   
  uint8_t  res02[0x030-0x024];  // 0x024	
  __IO uint32_t CAN_SEL_0;           // 0x030	   CAN0 0  CAN    , 0x7  CAN    
  __IO uint32_t CAN_SEL_1;           // 0x034	   CAN1 0  CAN    , 0x7  CAN    
  struct{
  __IO uint32_t START:      1; // 0	 1       
  __IO uint32_t PROG_START: 1; // 0	 1       
  __I uint32_t FINISH:      1; // 0	1           
  __I uint32_t RSLT:        1; // 0	1                  ( )
  __I uint32_t REDOK:       1; // 0	1                     ( )
  uint32_t : 27;    // 
  }BISR_L2C;            // 0x038	  BISR_L2C
  struct{
    __I uint32_t mem_rst_valid:           1;
    __I uint32_t refresh_in_process:      1;
    __I uint32_t q_almost_full:           1;
    __I uint32_t controller_busy:         1;
    __I uint32_t port_busy:               1;
    __I uint32_t cke_status:              1;
    __I uint32_t ecc_dataout_corrected:   1;
    __I uint32_t ecc_dataout_uncorrected: 1;
    uint32_t : 24;              // 
  }BISR_CLK_EN;                 // 0x03C	 L2C   
  struct{
    __IO uint32_t AWCOBUF:    1;
    __IO uint32_t AWALLSTRB:  1;
    __IO uint32_t AWAPCMD:    1;
    __IO uint32_t ARAPCMD:    1;
    uint32_t : 28;              // 
  }
  DDR0_STATUS,                  // 0x040	  DDR0
  DDR1_STATUS;                  // 0x044	  DDR1
  uint32_t DDR0_AXI_CFG;        // 0x048	 AXI-  DDR0
  uint32_t DDR1_AXI_CFG;        // 0x04C	 AXI-  DDR1
  __I uint8_t  res03[0x100-0x050];    // 0x050
  /*!>  IOCFG0IOCFG21    -    */
  __IO SCTL_IOCFG_TypeDef IoConfig;   // 0x100...0x154
  __I  uint8_t res04[0x400-0x158];    // 0x158	
  __IO uint32_t PCIE_RST;             // 0x400	    PCIe
  struct{//  uint32_t PCIE_REG_0;          // 0x404	     PCIe
    __IO uint32_t  pcie_generation_sel:   1; // 1  PCIe // 0: Gen1 (2.5Gbs per lane) // 1: Gen1 (2.5Gbs per lane)  Gen2 (5Gbs per lane)  ,  RC.//        
    __IO uint32_t  mode_select:           1; // 0	   PCIe// 0: Endpoint// 1: Root Complex//        
    __IO uint32_t  lane_count_in:         2; // 2	   // 00:   0 // 01:   0   1 // 10:   0,  1,  2,   3 // 11: reserved //         . //        
    __IO uint32_t  link_training_enable:  1; // 1	    // 0:  // 1: 
    __IO uint32_t  config_enable:         1; // 1	0:    ;   (CRS Completion)   PCIe // 1:    
    __IO uint32_t  req_pm_to_L23:         1; // 0	  Endpoint.      PCIe     L23_READY       L2 .     ,  PCI     D3   ,     PME_Turn_Off   Root Port.        
    __IO uint32_t  client_req_exit_L2:    1; // 0	         L2     .       ,     ,     L2_IDLE  DETECT 
    __IO uint32_t  :                      1; // 
    __IO uint32_t  down_cfg_susp_ln_00:   1; // 0	   0 // 0:  // 1:  //        
    __IO uint32_t  down_cfg_susp_ln_01:   1; // 0	   1 // 0:  // 1:  //        
    __IO uint32_t  down_cfg_susp_ln_02:   1; // 0	   2 // 0:  // 1:  //        
    __IO uint32_t  down_cfg_susp_ln_03:   1; // 0	   3 // 0:  // 1:  //        
    __IO uint32_t  hot_reset_in:          1; // 0	     RC  PCIe .     EP
    __I  uint32_t  hot_reset_out:         1; // 0	   RC  EP    
    __I  uint32_t  link_down_reset_out:   1; // 0	  PCIe      (link-down event  LINK_UP   0)
    __I  uint32_t  link_status:           2; // 0	   PCIe // 0      // 1     // 2   ,      // 3   ,    
    __I  uint32_t  link_power_state:      4; // 0	    // 1  L0 // 2  L0s // 4  L1 // 8  L2
    __IO uint32_t  client_req_exit_L1:    1; // 0	      PCIe   L1   L0.         L1   EndPoint
    __IO uint32_t  mbist_start:           1; // 0	          PCIe.//         mbist_enable
    __IO uint32_t  mbist_error:           1; // 0	        mbist_done. // 0     // 1        PCIe.  PCIe   .    
    __I  uint32_t  mbist_done:            1; // 0	         
    __IO uint32_t  mbist_enable:          1; // 0	   . //    : //  mbist_enable //  mbist_start //   mbist_done //  mbist_error
    __I  uint32_t  :                      5; // 
  }PCIE_Ctrl;
  struct{//  uint32_t PCIE_REG_1;          // 0x408	   
    __I   uint32_t  int200:               1;  // 0	        D1  D3.    ,    .    PCIe         
    __IO  uint32_t  ack200:               1;  // 0	 1      .        int200
    __IO  uint32_t  autoack200:           1;  // 0	1         // 0    
    __I   uint32_t  :                     29; // 
  }PCIE_Func;
  struct{//  uint32_t PCIE_REG_2;          // 0x40C	  
    __I   uint32_t  debug_data_out:       1; // 0	    .       PCIe
    __I   uint32_t  ltssm_state:          1; // 0	    .       PCIe
    __I   uint32_t  :                     30;//	
  }PCIE_Info;
}SCTL_TypeDef;  

/** @addtogroup Peripheral_declaration
  * @{
  */  

/* Initialization and Configuration functions *********************************/

/* GPIO Read and Write functions **********************************************/

/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_SCTL_H */
/**
  * @}
  */  
