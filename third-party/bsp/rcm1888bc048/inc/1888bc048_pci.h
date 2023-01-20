/**
  ******************************************************************************
  * @file    1888bc048_pci.h
  * @author  A. Maksimov, V. Syrtsov 
  * @version V1.0.1
  * @date    19.12.2019
  * @brief   This file contains all the functions prototypes for the PCI 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_PCI_H
#define __1888BC048_PCI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */

#define PCIE_RC_APB_ACCESS                    0x00002000


#define PCI_CLASS_BRIDGE                      0x0604

typedef enum{//    PCIe
  PCIE_Endpoint = 0,
  PCIE_RootComplex = 1
}PCIE_MODE_TypeDef;
  
/**
  * @brief PCI_BASE_SET structure definition  
  */
typedef struct{
  __IO uint32_t pcie_generation_sel:      1; // 1	 PCIe: // 0: Gen1 (2.5Gbs per lane)  // 1: Gen1 (2.5Gbs per lane)  Gen2 (5Gbs per lane)  ,  RC.//        
  __IO uint32_t mode_select:              1; // 0	   PCIe: // 0: Endpoint; // 1: Root Complex //        
  __IO uint32_t lane_count_in:            2; // 2	  : // 00:   0 // 01:   0   1 // 10:   0,  1,  2,   3// 11: reserved
//         . //        
  __IO uint32_t link_training_enable_reg: 1; // 1		   : // 0:  // 1: 
  __IO uint32_t config_enable_reg:        1; //	1	     PCIe: 0 - ;   (CRS Completion); 1 - 
  __IO uint32_t req_pm_transition_l23_ready_reg:1; // 0	  Endpoint.      PCIe     L23_READY       L2 .     ,  PCI     D3   ,     PME_Turn_Off   Root Port.        
  __IO uint32_t client_req_exit_l2_reg:   1; // 0          L2     .       ,     ,     L2_IDLE  DETECT 
  __IO uint32_t :                         1; // [8]	-	-	R0	
  __IO uint32_t down_cfg_susp_ln_00:      1; // 0	   0: // 0:  // 1:  //        
  __IO uint32_t down_cfg_susp_ln_01:      1; // 0		   1 // 0:  // 1:  //        
  __IO uint32_t down_cfg_susp_ln_02:      1; // 0		   2 // 0:  // 1:  //        
  __IO uint32_t down_cfg_susp_ln_03:      1; // 0		   3 // 0:  // 1:  //        
  __IO uint32_t hot_reset_in_reg:         1; // 0		     RC  PCIe .     EP
  __I uint32_t hot_reset_out_sync:        1; // 0		   RC  EP    
  __I uint32_t link_down_reset_out_sync:  1; // 0		  PCIe      (link-down event  LINK_UP   0)
  __I uint32_t link_status_sync:          2; // 0		   PCIe // 0      // 1     // 2   ,     
// 3   ,     
  __I uint32_t link_power_state_sync:     4; // 0		    // 1  L0 // 2  L0s // 4  L1 // 8  L2
  __IO uint32_t client_req_exit_l1_reg:   1; // 0		      PCIe   L1   L0.         L1   EndPoint
  __IO uint32_t mbist_start:              1; // 0		          PCIe. //         mbist_enable
  __IO uint32_t mbist_error:              1; // 0	        mbist_done. // 0     // 1        PCIe.  PCIe   .    
  __I uint32_t mbist_done:                1; // 0		         
  __IO uint32_t mbist_enable:             1; // 0	   .
//    : //  mbist_enable //  mbist_start //   mbist_done //  mbist_error
  __I uint32_t :                          5;  // [31:27]	-	-	R0	
}PCI_BASE_SET_TypeDef;
  
/**
  * @brief PCI_CTRL_SET structure definition  
  */
typedef struct{
  __I uint32_t int200:      1;  // 0	        D1  D3.    ,    .    PCIe         
  __IO uint32_t ack200:     1;  // 0  1      .        int200
  __IO uint32_t autoack200: 1;  // 0	1         // 0    
  __I uint32_t :            29; // 
}PCI_CTRL_TypeDef;
  
/**
  * @brief PCI_DBG_INFO structure definition  
  */
typedef struct{
  __I uint32_t debug_data_out_sync: 16; // 0	    .       PCIe
  __I uint32_t ltssm_state_sync:    6;  // 0	    .       PCIe
  __I uint32_t :                    10; // 
}PCI_DBG_INFO_TypeDef;

/**
  * @brief PCI PHY structure definition  
  */
typedef struct{
  __IO uint32_t Reset; /*!<  1     :  0    ; 
     0     ;  1    .
       1    0.      ( PLL)               */
  PCI_BASE_SET_TypeDef BaseSettings; /*!<   */
  PCI_CTRL_TypeDef Control; /*!<    */
  PCI_DBG_INFO_TypeDef DbgInfo; /*!<    */
}PCI_PHY_TypeDef;

/**
  * @brief PCI_CorrErr structure definition  
  */
typedef struct{
  __IO  uint32_t ReceiverError:1;           // 0	      0x0	                 :  - 8B10B ;  - ;  -   .    1   APB
  __I   uint32_t :5;                        // 5:1	    0x0	        Reserved
  __IO  uint32_t Bad_TP:1;                  // 6	      0x0	              TLP    :  - LCRC;  -    EDB    LCRC                LCRC.    1   APB
  __IO  uint32_t Bad_DLLP:1;                // 7	      0x0	               DLLP  LCRC     .    1   APB
  __IO  uint32_t ReplayNumberRollover:1;     // 8	      0x0	              .    1   APB
  __I   uint32_t :3;                        // 11:9	    0x0	        Reserved
  __IO  uint32_t ReplayTimerTimeout:1;      // 12	      0x0	             ,      TLP.    1   APB
  __IO  uint32_t AdvisoryNonFatalError:1;   // 13	      0x0	             (Ucorrectable error).                       . 6.2.3.2.4  PCI Express 2.0.    1   APB
  __IO  uint32_t CorrectInternalError:1;    // 14	      0x0	              ( ECC     ).    1   APB
  __IO  uint32_t HeaderLogOverflow:1;       // 15	      0x0	              Header Log Register.    1   APB
  __I   uint32_t :16;                       // 31:16	  0x0	        Reserved    
}PCI_CorrErr_TypeDef;

/**
  * @brief PCI_ID structure definition  
  */
typedef struct{
  __IO uint32_t Vendor:16;
  __IO uint32_t Device:16;
}PCI_ID_TypeDef;

/**
  * @brief PCI_CMD structure definition  
  */
typedef struct{
  __IO uint32_t IO_SpaceEnable: 1;          // 0	      0x0	            IO  .    APB  
  __IO uint32_t MEM_SpaceEnable: 1;          // 1	      0x0	            Memory  .    APB  
  __IO uint32_t BusMasterEnable: 1;         // 2	      0x0	            IO  Memory  .    APB  
  __I  uint32_t : 3;                        // 5:3      0x0	        Reserved
  __IO uint32_t ParErrResponseEnable: 1;    // 6	      0x0	        1    Master Data Parity Error  0   Master Data Parity Error     APB  
  __I  uint32_t : 1;                        // 7	      0x0	        Reserved
  __IO uint32_t SERR_Enable: 1;             // 8	      0x0	        1          - 0       APB  
  __IO uint32_t : 1;                        // 9	      0x0	        Reserved
  __IO uint32_t INTxMessageDisable: 1;      // 10	      0x0	        1  INTx (Legacy)   0  INTx (Legacy)      APB  
  __I  uint32_t : 8;                        // 18:11	  0x0	        Reserved
  __IO uint32_t InterruptStatus: 1;         // 19	      0x0	        ,    INTx (Legacy) . ,   Assert_INTx  ,   Deassert_INTx   
  __IO uint32_t CapabilitiesList: 1;        // 20	      0x1	              (PCI Extended Capabilities registers).   1
  __I  uint32_t : 3;                        // 23:21	  0x0	        Reserved
  __IO uint32_t MasterDataParError: 1;      // 24	      0x0	              (poisoned completion)      (poisoned write request).       1   APB
  __I  uint32_t : 2;                        // 26:25	  0x0	        Reserved
  __IO uint32_t SignaledTargetAbort: 1;     // 27	      0x0	        ,        Completer Abort.       1   APB
  __IO uint32_t ReceivedTargetAbort: 1;     // 28	      0x0	        ,        Completer Abort.       1   APB
  __IO uint32_t ReceivedMasterAbort: 1;     // 29	      0x0	        ,        Unsupported Request.       1   APB
  __IO uint32_t SignaledSystemError: 1;     // 30	      0x0	            -       .       1   APB
  __IO uint32_t DetectedParError: 1;        // 31	      0x0	             TLP  .       1   APB
}PCI_CMD_TypeDef;

/**
  * @brief PCI_RevClass structure definition  
  */
typedef struct{
  __IO uint32_t RevisionID:8;               // 7:0     	0	            ,   .    APB  
  __IO uint32_t ProgramInterfaceByte:8;	    // 15:8     0	             .    APB  
  __IO uint32_t SubClassCode:8;             // 23:16    0	             .    APB  
  __IO uint32_t ClassCode:8;                // 24:31	  0              .    APB  
}PCI_RevClass_TypeDef;

/**
  * @brief PCI_BistHeader structure definition  
  */
typedef struct{
  __IO uint32_t CacheLineSize:8;            // 7:0	    0	          . PCI Specifications 3.0.    PCIe  APB     ,     
  __IO uint32_t LatencyTimer:8;             // 15:8	    0	           .  0
  __IO uint32_t HeaderType:7;               // 22:16	  0	           .   0
  __IO uint32_t DeviceType:1;               // 23	R	    0	            0
  __IO uint32_t BIST_Register:8;            // 31:24	  0	            BIST.    APB    
}PCI_BistHeader_TypeDef;

/**
  * @brief PCI_PWR_MGMNT structure definition  
  */
typedef struct{
  __IO uint32_t CapabilityID:8;             // 7:0	    0x01	      ,     .    APB  
  __IO uint32_t CapabilitiesPointer:8;      // 15:8	    0x90	            (PCI Capability Structure).    APB  
  __IO uint32_t VersionID:3;                // 18:16	  0x3	            ,   .    APB  
  __IO uint32_t PME_Clock:1;                // 19	      0x0	          0
  __I  uint32_t :1;                         // 20	      0x0	        Reserved
  __IO uint32_t DeviceSpecInitBit:1;        // 21	      0x0	          0
  __IO uint32_t MaxCurAuxPwrSupply:3;       // 24:22	  0x0	          0
  __IO uint32_t D1_Support:1;               // 25	      0x1	           PCI   D1.    APB  
  __IO uint32_t D2_Support:1;               // 26	      0x0	           PCI   D2,   0
  __IO uint32_t PME_SupportD0_State:1;      // 27	      0x01	      ,    PCI        (PME messages),    D0.    APB  
  __IO uint32_t PME_SupportD1_State:1;      // 28	      0x1	        ,    PCI        (PME messages),    D1.    APB  
  __IO uint32_t PME_SupportD2_State:1;      // 29	    	0x0	          0 (D2   )
  __IO uint32_t PME_SupportD3hotState:1;    // 30	    	0x01	      ,    PCI        (PME messages),    D3(hot).    APB  
  __IO uint32_t PME_SupportD3coldState:1;   // 31	      0x0	          0
}PCI_PWR_MGMNT_TypeDef;

/**
  * @brief PCI_PWR_CTRL structure definition  
  */
typedef struct{
  __IO uint32_t PowerState:2;               // 1:0	    0x0	                     00  D0 01  D1 10  reserved 11  D3(hot)    APB  
  __I  uint32_t :1;                         // 2	      0x0	        Reserved
  __IO uint32_t NoSoftReset:1;              // 3	      0x01	        PCI      D3(hot) .    APB  
  __I  uint32_t :4;                         // 7:4	    0x0	        Reserved 
  __IO uint32_t PMEEnable:1;                // 8	      0x0	              (PME messages).    APB  
  __I  uint32_t :6;                         // 14:9	    0x0	        Reserved
  __IO uint32_t PMEStatus:1;                // 15	      0x0	           PME Enable  1   APB   PME- (   End Point). RC            1 (Config Write).  //    APB  
  __I  uint32_t :8;                         // 23:16	  0x0	        Reserved
  __IO uint32_t Data:8;                     // 31:24	  0x0	          0
}PCI_PWR_CTRL_TypeDef;

/**
  * @brief PCI_MSI_CTRL structure definition  
  */
typedef struct{
  __IO uint32_t CapabilityID:8;             // 7:0	    0x05	      ,     MSI .   0x05
  __IO uint32_t CapabilitiesPointer:8;      // 15:8	    0xB0	           .    APB  
  __IO uint32_t MsiEnable:1;                // 16	      0x0	         MSI .    APB  
  __IO uint32_t MultipleMsgCapable:3;       // 19:17	  0x0	          MSI ,        (..    ) 000 = 1 001 = 2 010 = 4 011 = 8 100 = 16 101 = 32     32  ,          ,   .    APB  
  __IO uint32_t MultipleMsgEnable:3;        // 22:20	  0x0	          MSI ,       (..    ) 000 = 1 001 = 2 010 = 4 011 = 8 100 = 16 101 = 32         ,   .    APB  
  __IO uint32_t AddressCapable:1;           // 23	      0x1	          1 ,    64-   MSI .    APB  
  __IO uint32_t MsiMaskCapable:1;           // 24	      0x1	           APB  
  __I  uint32_t :7;                         // 31:25	  0x0	        Reserved
  }PCI_MSI_CTRL_TypeDef;

/**
  * @brief PCI_MSI_MSG_LA structure definition  
  */
typedef struct{
  __I  uint32_t :2;                         // 1:0	    0x0	        Reserved  
  __IO uint32_t Address:30;                 // 31:2     0x0	           MSI .    APB  
}PCI_MSI_MSG_LA_TypeDef;

/**
  * @brief PCI_MSIX_CTRL structure definition  
  */
typedef struct{
  __IO uint32_t CapabilityID:8;             // 7:0	    0x11	      ,     MSI-X.    APB  
  __IO uint32_t CapabilitiesPointer:8;      // 15:8	    0xc0	           .    APB  
  __IO uint32_t TableSize:11;               // 26:16	  0x3f	        MSI-X , ..    . ,  ,     .    APB  
  __I  uint32_t :3;                         // 29:27	  0x0	        Reserved
  __IO uint32_t FunctionMask:1;             // 30	      0x0	          MSI-X .    APB  
  __IO uint32_t Enable:1;                   // 31	      0x0	          MSI-X .    APB  
}PCI_MSIX_CTRL_TypeDef;

/**
  * @brief PCI_MSIX_TBLOFFSET & PCI_MSIX_PENDI structure definition  
  */
typedef struct{
  __IO uint32_t BarIndicator:3;             // 2:0	    0	            BAR,   ,     MSI-X. 000  BAR 0 001  BAR 1 010  BAR 2 011  BAR 3 100  BAR 4 101  BAR 5        BAR 0      .    APB   
  __IO uint32_t Offset:29;                  // 31:3	                     MSI-X     BAR.    ,                                  32- .           v0_Message_Address   ,     .               .    APB  
}PCI_MSIX_TBLOFFSET_TypeDef, PCI_MSIX_PENDI_TypeDef;

/**
  * @brief PCI_CAPLIST structure definition  
  */
typedef struct{
  __IO uint32_t CapabilityID:8;             // 7:0	    0x010	      ,     Express .   0x010
  __IO uint32_t NextCapabilityPtr:8;        // 15:8	    0x0	             .   ,    .   0
  __IO uint32_t CapabilityVersion:4;        // 19:16	  0x2	            .       PCI Express Base Specification Revision 3.0.    APB  
  __IO uint32_t DeviceType:4;               // 23:20	  0x0	           0x0     End Point (EP) 0x4  -  Root Port (RP)   0   
  __IO uint32_t SlotStatus:1;               // 24	      0x0	          1,     .   0
  __IO uint32_t InterruptMsgNumber:5;       // 29:25	  0x0	          MSI  MSI-X   ,          Slot Status Register, Root Status Register    .          MSI  MSI-X.   0 	  	  	
  __IO uint32_t TCS_RoutingSupport:1;       // 30	      0x0	               (Trusted Configuration Requests). 1   .      .   0
  __I  uint32_t :1;                         // 31	      0x0	        Reserved
}PCI_CAPLIST_TypeDef;

/**
  * @brief PCI_DEV_CAP structure definition  
  */
typedef     struct{
  __IO uint32_t MaxPayloadSize:3;           // 2:0      b011	           .    APB  
  __IO uint32_t PhantomFuncSupport:2;       // 4:3	    0x0	              .     .   0
  __IO uint32_t ExtendTagFieldSupport:1;    // 5	      0x0	          tag  .   0
  __IO uint32_t AcceptableL0s_Latency:3;    // 8:6	    0x4              L0s  L0 .    APB  
  __IO uint32_t AcceptableL1_Latency:3;     // 11:9	    0x0	             L1  L0 .    APB  
  __I  uint32_t :3;                         // 14:12	  0x0	        Reserved
  __IO uint32_t RoleBasedErrorReport:1;     // 15	      0x01	        (role-based)   .    APB  
  __I  uint32_t :2;                         // 17:16	  0x0	        Reserved
  __IO uint32_t CaptSlotPwrLimValue:8;      // 25:18	  0x0	          ,  .    APB  
  __IO uint32_t CaptPwrLimScale:2;          // 27:26	  0x0	           Captured Slot Power Limit Value  .    APB  
  __IO uint32_t FuncLevelResetCapability:1; // 28	      0x0	          0
  __I  uint32_t :3;                         // 31:29	  0x0	        Reserved    
}PCI_DEV_CAP_TypeDef;

/**
  * @brief PCI_CTRL_STAT structure definition  
  */
typedef struct{
  __IO uint32_t CorrErrorReportEn:1;        // 0	      0x0	          ERR_COR     .    APB  
  __IO uint32_t NonFatalErrorReportEn:1;    // 1	      0x0	          ERR_NONFATAL     .    APB  
  __IO uint32_t FatalErrorReportEn:1;       // 2	      0x0	          ERR_FATAL     .    APB  
  __IO uint32_t UnsupportReqReportEn:1;     // 3	      0x0	              .    APB  
  __IO uint32_t RelaxOrderingEn:1;          // 4	      0x01	         Relaxed Ordering   .    APB  
  __IO uint32_t MaxPayloadSize:3;           // 7:5	    0x0	             .                  .       Device Capabilities Register    .    APB  
  __IO uint32_t ExtendedTagFieldEn:1;       // 8	      0x0	          tag  5  8 .   0
  __IO uint32_t PhantomFuncEn:1;            // 9	      0x0	          0
  __IO uint32_t AuxPowerEn:1;               // 10	      0x0	          0
  __IO uint32_t NoSnoopEn:1;                // 11	      0x1	          1    No Snoop   ,    .    APB  
  __IO uint32_t MaxReadReqSize:3;           // 14:12	  0x02	            ,   .    APB  
  __IO uint32_t FunctionLevelReset:1;       // 15	      0x0	          0
  __IO uint32_t CorrErrorDetect:1;          // 16	      0x0	         ,     .        .    1   APB
  __IO uint32_t NonFatalErrorDetect:1;      // 17	      0x0	         ,     .        .    1   APB
  __IO uint32_t FatalErrorDetect:1;         // 18	      0x0	         ,     .        .    1   APB
  __IO uint32_t UnsupportReqDetect:1;       // 19	      0x0	         ,     .      .    1   APB
  __IO uint32_t AuxPowerDetect:1;           // 20	      0x0	         .   0
  __IO uint32_t TransPending:1;             // 21	      0x0	        ,    ,   (Non-Posted).      APB
  __I  uint32_t :10;                        // 31:22	  0x0	        Reserved 
}PCI_CTRL_STAT_TypeDef;

/**
  * @brief PCI_LINC_CAP structure definition  
  */
typedef struct{
  __IO uint32_t MaxLinkSpeed:4;             // 3:0	    0x2	           .  ,    .      APB
  __IO uint32_t MaxLinkWidth:6;             // 9:4	    0x4	           .  ,    .      APB
  __IO uint32_t ASPM:2;                     // 11:10	  0x3	           ASPM.    APB  
  __IO uint32_t L0s_ExitLatency:3;          // 14:12	  0x2	         ,       L0S  L0.     Phy.    APB  
  __IO uint32_t L1_ExitLatency:3;           // 17:15	  0x3	         ,       L1.     Phy.    APB  
  __IO uint32_t ClockPwrManagement:1;       // 18	      0x0	        ,       .    APB  
  __IO uint32_t SurpriseDownErrorReport:1;  // 19	      0x0	                (Surprize Down error).    .   0
  __IO uint32_t DataLinkLayerActiveReport:1;// 20	      0x0	                   DL_Active.     .   0
  __IO uint32_t LinkBandwidthNotification:1;// 21	      0x0	               .     Endpoint.   0
  __IO uint32_t ASPM_OptionCompliance:1;    // 22	      0x1	            ASPM .    APB  
  __I  uint32_t :1;                         // 23	      0x0	        Reserved
  __IO uint32_t PortNumber:8;               // 31:24	  0x0	          ,  PCIe ,   .    APB  
}PCI_LINC_CAP_TypeDef;

/**
  * @brief PCI_LINC_CTRL_STAT structure definition  
  */
typedef struct{
  __IO uint32_t ASPM_Control:2;             // 1:0	    0x0	         ASPM : 00  ASPM  01  L0s  , L1  10  L0s  , L1  11  L0s  L1             Active State Power Management  PCIe_EP_i_link_cap.    APB  
  __I  uint32_t :1;                         // 2	      0x0	        Reserved
  __IO uint32_t ReadCompletionBoundary:1;   // 3	      0x0	             -,     . 0  64  1  128     APB      [13]  
  __IO uint32_t LinkDisable:1;              // 4	      0x0	           Endpoint 
  __IO uint32_t RetrainLink:1;              // 5	      0x0	           Endpoint 
  __IO uint32_t CommonClockConfig:1;        // 6	      0x0	              EP    0   1      APB  
  __IO uint32_t ExtendedSync:1;             // 7	      0x0	                (Ordered Sets)    L0s  L0 .    APB  
  __IO uint32_t ClockPwrManagementEn:1;     // 8	      0x0	             1,    CLKREQ#  PCIe     .        Clock Power Management   Link Capability Register.    APB  
  __IO uint32_t HardwareAutoWidthDis:1;     // 9	      0x0	        ,    ,               .    APB  
  __IO uint32_t LinkBandManagementIrqEn:1;  // 10	      0x0	           Endpoint 
  __IO uint32_t LinkAutoBandIrqEn:1;        // 11	      0x0	           Endpoint 
  __I  uint32_t :4;                         // 15:12	  0x0	        Reserved
  __IO uint32_t NegotiatedLinkSpeed:4;      // 19:16	  0x2	            0001  2,5 /   0010  5  /      APB   
  __IO uint32_t NegotiatedLinkWidth:6;      // 25:20	  0x4	          .     .    APB   
  __I  uint32_t :1;                         // 26	      0x0	        Reserved
  __IO uint32_t LinkTrainingStatus:1;       // 27	      0x0	         ,  LTSSM   Configuration  Recovery     .   0   Endpoint.    APB   
  __IO uint32_t SlotClockConfig:1;          // 28	      0x0	        ,      .            ,       0.    APB  
  __IO uint32_t DataLinkLayerActive:1;      // 29	      0x0	           .   0
  __IO uint32_t LinkBandManagementStatus:1; // 30	      0x0	                ,    .   0,  Link Bandwidth Notification Capability  0.   0   Endpoint.    1   APB
  __IO uint32_t LinkAutoBandwidthStatus:1;  // 31	      0x0	                ,     .   0,  Link Bandwidth Notification Capability  0.   0   Endpoint.    1   APB
}PCI_LINC_CTRL_STAT_TypeDef;

/**
  * @brief PCI_EndPoint structure definition  
  */
typedef struct{
//                          		       
  PCI_ID_TypeDef ID;                        // 0x0000		0x010017CD	   (Vendor ID  Device ID)
  PCI_CMD_TypeDef Command;                  // 0x0004		0x00100000	 
  PCI_RevClass_TypeDef RevisionClass;       // 0x0008		0x0	          (Revision ID  Class Code)
  PCI_BistHeader_TypeDef BistHeader;        // 0x000C		0x0	           
  union{
  struct{  
  __IO uint32_t Type: 1;                    // 0	      0x0	         ,  BAR 0  0  memory 1  IO      0
  __I  uint32_t : 1;                        // 1	      0x0	        Reserved
  __IO uint32_t Size: 1;                    // 2	      0x0	          (  memory ) 0  32  1  64 ,    BAR 1  BAR 0 (  BAR   )      0
  __IO uint32_t Prefetch: 1;                // 3	      0x0	           (  memory ).      0
  __I  uint32_t :4;                         // 7:4	    0x0	        Reserved
  __IO uint32_t AddrRd:10;                  // 17:8	    0x0	           .         0.    APB  
  __IO uint32_t AddrWr:14;                  // 31:18	  0x0	           .         0.    APB  
  }BAR0; //  
  struct{    
  __IO uint32_t Type: 1;                    // 0	      0x0	         ,  BAR 1  0  memory 1  IO      0
  __I  uint32_t : 1;                        // 1	      0x0	        Reserved
  __IO uint32_t Size: 1;                    // 2	      0x0	         32-  (  BAR   )      0
  __IO uint32_t Prefetch: 1;                // 3	      0x0	           (  memory ).      0
  __I  uint32_t :4;                         // 7:4	    0x0	        Reserved
  __IO uint32_t AddrRO:12;                  // 19:8	    0x0	           .         0.    APB  
  __IO uint32_t AddrRW:12;                  // 31:20	  0x0	           .         0.    APB    
  }BAR1; //  
  }             BaseAddr[6];                // 0x0010		0x0	           
  __I  uint32_t res00;                      // 0x0028               
  struct{
  __IO uint32_t Vendor:16;                  // 15:0	    0x17cd	    ID  ,   PCI SIG.      PCIe_LocMgmt_i_vendor_id_reg
  __IO uint32_t Device:16;                  // 31:16	  0	          ID ,    .    APB  
  }             SubsystemID;                // 0x002C	  0x000017CD	   (Subsystem Vendor ID  Subsystem ID)
  __I  uint32_t res01;                      // 0x0030               
  struct{
  __IO uint32_t Pointer:8;                  // 7:0	    0x80	            (PCI Capability Structure).       (Power Management Capability Structure).    APB  
  __I  uint32_t :24;                        // 31:8	    0x0	        Reserved
  }             CapabilityStruct;           // 0x0034	  0x80	           (PCI Capability Structure)
  __I  uint32_t res02;                      // 0x0038               
  struct{
  __IO uint32_t Line:8;                     // 7:0	    0xff	      Interrupt Line Register: ,     - ( )    .         Assert_INTx  Deassert_INTx   PCIe. (00 = IRQ0, ... , 0F = IRQ15, FF = ).    APB  
  __IO uint32_t Pin:3;                      // 10:8	    0x1	        Interrupt Pin Register: ,    Legacy-,    0x1  INTA 0x2  INTB 0x3  INTC 0x4  INTD    APB  .    Legacy-    PCIe    INTA.      Legacy-
  __I  uint32_t  :21;                       // 31:11	  0x0	        Reserved
  }             InterruptLinePin;           // 0x003C	  0x1FF	        Legacy-
  __I  uint8_t  res03[0x80-0x40];           // 0x0040               
  PCI_PWR_MGMNT_TypeDef PowerManagement;    // 0x0080	  0x5A039001	    
  PCI_PWR_CTRL_TypeDef PowerControl;        // 0x0084		0x8	          
  __I  uint32_t res04[2];                   // 0x0088               
  PCI_MSI_CTRL_TypeDef       MsiControl;    // 0x0090	  0x0180B005	  MSI-
  PCI_MSI_MSG_LA_TypeDef     MsiMsgLowAddr; // 0x0094	  0x0	          MSI- ( )
  __IO uint32_t MsiMsgHiAddr;               // 0x0098	  0x0	          MSI- ( )
  __IO uint32_t MsiMsgData;                 // 0x009C	  0x0	          MSI-
  __IO uint32_t MsiMask;                    // 0x00A0	  0x0	             MSI-
  __IO uint32_t MsiPendingBits;             // 0x00A4	  0x0	          MSI-
  __I  uint32_t res05[2];                   // 0x00A8                 
  PCI_MSIX_CTRL_TypeDef MsixControl;        // 0x00B0	  0x0000C011	  MSI-X-
  PCI_MSIX_TBLOFFSET_TypeDef MsixTblOffset; // 0x00B4	  0x0	            MSI-X-
  PCI_MSIX_PENDI_TypeDef MsixPendingIrq;    // 0x00B8	  0x00000008	    MSI-X-
  __I  uint32_t res06;                      // 0x00BC               
  PCI_CAPLIST_TypeDef CapList;              // 0x00C0	  0x00020010	   PCIe
  PCI_DEV_CAP_TypeDef  DevCap;              // 0x00C4	  0x00008103	   PCIe
  PCI_CTRL_STAT_TypeDef DevCtrlStatus;      // 0x00C8   0x00002810	     PCIe
  PCI_LINC_CAP_TypeDef LinkCap;             // 0x00CC		0x0041AC42	  
  PCI_LINC_CTRL_STAT_TypeDef LinkCtrlStatus;// 0x00D0		0x00410000	    
  __I  uint8_t  res07[0xE4-0xD4];           // 0x00D4                 
  struct{
  __IO uint32_t CompletTimeoutRanges:4;     // 3:0	    0x02	           .   10-250 .    APB  
  __IO uint32_t CompletTimeoutDisSupport:1; // 4	      0x01	            .    APB  
  __IO uint32_t AriForwardSupport:1;        // 5	      0x0	          0
  __IO uint32_t SupportAtomOP_Routing:1;    // 6	      0x0	           (Atomic OP) .     Endpoint
  __IO uint32_t Support32_BitAtomOp:1;      // 7	      0x0	          0
  __IO uint32_t Support64_BitAtomOp:1;      // 8	      0x0	          0
  __IO uint32_t Support128_BitCasAtomOp:1;  // 9	      0x0	          0
  __I  uint32_t :1;                         // 10	      0x0	        Reserved
  __IO uint32_t LTR_MechanismSupport:1;     // 11	      0x0	          0
  __IO uint32_t TPH_CompletSupport:2;       // 13:12	  0x0	          0
  __I  uint32_t :4;                         // 17:14	  0x0	        Reserved
  __IO uint32_t OBFF_Support:2;             // 19:18	  0x0	        ,    OBFF.   0
  __IO uint32_t ExtendFormatFieldSupport:1; // 20	      0x1	        ,    3-   Fmt  TLP.   1
  __IO uint32_t End_EndTLP_PrefixSupport:1; // 21	      0x0	        ,    End-End TLP Prefix.   0
  __IO uint32_t MaxEnd_EndTLP_Prefixes:2;   // 23:22	  0x0	          End-End TLP Prefixes 01  1 End-End TLP Prefix 10  2 End-End TLP Prefix   0
  __I  uint32_t :8;                         // 31:24	  0x0	        Reserved
  }             PcieDevCap2;                // 0x00E4		0x00100012	   PCIe 2
  struct{
  __IO uint32_t CompletTimeoutValue:4;      // 3:0	    0x0	            . 0101   1 0110   2      Completion Timeout Interval Registers 0  1.    APB  
  __IO uint32_t CompletTimeoutDis:1;        // 4	      0x0	              .    APB  
  __IO uint32_t ARI_ForwardEn:1;            // 5	      0x0	          0
  __IO uint32_t AtomOpRequestEn:1;          // 6	      0x0	           .   0
  __I  uint32_t :1;                         // 7	      0x0	        Reserved
  __IO uint32_t IDO_RequestEn:1;            // 8	      0x0	           ,     ID-based Ordering (IDO) Attribute   .   0
  __IO uint32_t IDO_CompletEn:1;            // 9	      0x0	           ,     ID-based Ordering (IDO) Attribute   .   0
  __IO uint32_t LTR_MechanismEn:1;          // 10	      0x0	          0
  __I  uint32_t :2;                         // 12:11	  0x0	        Reserved
  __I  uint32_t OBFF_En:2;                  // 14:13	  0x0	         OBFF (Optimized Buffer Flush/Fill). 00   01   A 10   B   0
  __I  uint32_t :17;                        // 31:15	  0x0	        Reserved
  }             PcieDevCtrlStatus2;         // 0x00E8	  0x0	             PCIe 2
  struct{
  __I   uint32_t :1;                        // 0	      0	          Reserved
  __IO  uint32_t SupportLinkSpeedVector:2;  // 2:1	    0x3	             0001  2,5 / (Gen 1) 0011  2,5 /  5 / (Gen 2)  ,    .    APB   
  __I   uint32_t :6;                        // 8:3	    0x0	        Reserved
  __IO  uint32_t SKP_OS_GeneratSupport:2;   // 10:9	    0x0	           0, ,      SRIS        SKP OS.    APB   
  __I   uint32_t :5;                        // 15:11	  0x0	        Reserved
  __IO  uint32_t SKP_OS_ReceptSupport:2;    // 17:16	  0x0	           0, ,      SRIS     SKP OS  ,   SRNS     SRIS.    APB   
  __I   uint32_t :14;                       // 31:18	  0x0	        Reserved
  }             LinkCapLink2;               // 0x00EC		0x00000006	   2
  struct{
  __IO  uint32_t TargetLinkSpeed:4;         // 3:0	    2	           Endpoint      .        Compliance,    Enter Compliance.         . 0001  2,5 / 0010  5 /    APB  
  __IO  uint32_t EnterCompliance:1;         // 4	      0x0	         Endpoint    1  Compliance .         (Hot Reset).    APB  
  __IO  uint32_t HardwareAutoSpeedDis:1;    // 5	      0x0	               .    APB  
  __IO  uint32_t SelectDeemphasis:1;        // 6	      0x0	            Endpoint
  __IO  uint32_t TransmitMargin:3;          // 9:7	    0x0	              Compliance .      . 000    001  800-1200        400-700    . 010-111      PCI Express Base Specification 2.0.    0      (LTSSM)   Polling.Configuration.    APB  
  __IO  uint32_t EnterModifiedCompliance:1; // 10	      0x0	              Compliance .       Compliance   Modified Compliance Pattern.        Enter Compliance       (Hot Reset).    APB  
  __IO  uint32_t ComplianceSOS:1;           // 11	      0x0	             Compliance Patterns   SKP  .    APB  
  __IO  uint32_t ComplianceDeEmphasis:4;    // 15:12	  0x0	          De-Emphasis   Compliance 0  -6  1  -3,5           5 / (Gen 2).    APB   
  __IO  uint32_t CurrentDeEmphasisLevel:1;  // 16	      0x1	           De-Emphasis . 0  -6  1  -3,5        5 / (Gen 2).    APB   
  __I   uint32_t :15;                       // 31:17	  0x0	        Reserved
  }             LinkCtrlStatus2;            // 0x00F0		0x00010002	     2
  __I  uint8_t  res08[0x100-0xF4];          // 0x00F4                 
  struct{
  __IO  uint32_t ExtendCapabilityID:16;     // 15:0	    0x01	      ,        (AER).    APB   
  __IO  uint32_t CapabilityVersion:4;       // 19:16	  2	             .    APB  
  __IO  uint32_t NextCapabilityOffset:12;   // 31:20	  0	                 .    APB  
  }             AerEnhancedCapHdr;          // 0x0100	  0x00020001	    
  struct{
  __I   uint32_t :4;                        // 3:0	    0x0	        Reserved
  __IO  uint32_t DataLinkProtoErrStatus:1;  // 4	      0x0	        ,    (DLLP Ack/Nak)      (TLP).    1   APB
  __I   uint32_t :7;                        // 11:5	    0x0	        Reserved
  __IO  uint32_t PoisonedTLP_Status:1;      // 12	      0x0	           TLP     (poisoned bit).   .      Header Log Register.    1   APB
  __IO  uint32_t FlowCtrlProtoErrStatus:1;  // 13	      0x0             .    1   APB
  __IO  uint32_t CompletTimeoutStatus:1;    // 14	      0x0	        ,            .   .    1   APB
  __IO  uint32_t CompletAbortStatus:1;      // 15	      0x0	        ,     Completer Abort    ,   .      Header Log Register.    1   APB
  __IO  uint32_t UnexpectCompletStatus:1;   // 16	      0x0	              (unexpected Completion).    1   APB
  __IO  uint32_t ReceiverOverflowStatus:1;  // 17	      0x0	              (TLP)    .    1   APB
  __IO  uint32_t MalformTLP_Status:1;       // 18	      0x0	           TLP   .   .      Header Log Register.    1   APB
  __IO  uint32_t ECRC_ErrStatus:1;          // 19	      0x0	            ECRC   TLP.      Header Log Register.    1   APB
  __IO  uint32_t UnsupportReqErrStatus:1;   // 20	      0x0	             .   .      Header Log Register.    1   APB
  __I   uint32_t :1;                        // 21	      0x0	        Reserved
  __IO  uint32_t UncorrIntErrStatus:1;      // 22	      0x0	               ( /ECC    ).   .    1   APB
  __I   uint32_t :9;                        // 31:23	  0x0	        Reserved    
  }             UncorrErrStatus;            // 0x0104		0x0	           
  struct{
  __I   uint32_t :4;                        // 3:0	    0x0	        Reserved
  __IO  uint32_t DataLinkProtoErrMask:1;    // 4	      0x0	          Data Link Protocol.    APB  
  __I   uint32_t :7;                        // 11:5	    0x0	        Reserved
  __IO  uint32_t PoisonedTLP_Mask:1;        // 12	      0x0	          Poisoned TLP.    APB  
  __IO  uint32_t FlowCtrlProtoErrMask:1;    // 13	      0x0	           .    APB  .
  __IO  uint32_t CompletTimeoutMask:1;      // 14	      0x0	             .    APB  
  __IO  uint32_t CompletAbortMask:1;        // 15	      0x0	             .    APB  
  __IO  uint32_t UnexpectCompletMask:1;     // 16	      0x0	             .    APB  
  __IO  uint32_t RecOverflowMask:1;         // 17	      0x0	           .    APB  
  __IO  uint32_t MalformedTLP_Mask:1;       // 18	      0x0	            TLP.    APB  
  __IO  uint32_t ECRC_ErrorMask:1;          // 19	      0x0	          ECRC.    APB  
  __IO  uint32_t UnsuppReqErrMask:1;        // 20	      0x0	           .    APB  
  __I   uint32_t :1;                        // 21	      0x0	        Reserved
  __IO  uint32_t UncorrIntErrMask:1;        // 22	      0x1	           .    APB  
  __I   uint32_t :9;                        // 31:23	  0x0	        Reserved  
  }             UncorrErrMask;              // 0x0108		0x00400000	   
  struct{
  __I   uint32_t :4;                        // 3:0	    0x0	        Reserved
  __IO  uint32_t DataLinkProtocolError:1;   // 4	      0x01	         0    1      APB  
  __IO  uint32_t SurpriseDownError:1;       // 5	      0x1	          1 
  __I   uint32_t :6;                        // 11:6	    0x0	        Reserved
  __IO  uint32_t Poisoned_TLP:1;            // 12	      0x0	           0    1      APB  
  __IO  uint32_t FlowControlProtoError:1;   // 13	      0x01	         0    1      APB  
  __IO  uint32_t CompletionTimeout:1;       // 14	      0x0	           0    1      APB  
  __IO  uint32_t CompleterAbort:1;          // 15	      0x0	           0    1      APB  
  __IO  uint32_t UnexpectCompletion:1;      // 16	      0x0	           0   1      APB  
  __IO  uint32_t ReceiverOverflow:1;        // 17	      0x01	         0    1      APB  
  __IO  uint32_t MalformedTLP:1;            // 18	      0x01	         0    1      APB  .
  __IO  uint32_t ECRC_Error:1;              // 19	      0x0	           0    1      APB  
  __IO  uint32_t UnsupportReqError:1;       // 20	      0x0	           0   1      APB  
  __I   uint32_t :1;                        // 21	      0x0	        Reserved
  __IO  uint32_t UncorrIntError:1;          // 22	      0x01	         0    1      APB  
  __I   uint32_t :9;                        // 31:23	  0x0	        Reserved  
  }             UncorrErrSeverity;          // 0x010C	  0x00462030	   
  PCI_CorrErr_TypeDef CorrErrStatus;        // 0x0110		0x0	           
  PCI_CorrErr_TypeDef CorrErrMask;          // 0x0114		0x0000E000	   
  struct{
  __IO  uint32_t FirstErrorPointer:5;       // 4:0	    0x0	          ,    .     ,       .                .    APB   
  __IO  uint32_t ECRC_GenCapability:1;      // 5	      0x1	        ,     ECRC   .    APB  
  __IO  uint32_t ECRC_GenerationEn:1;       // 6	      0x0	           ECRC   .    APB  
  __IO  uint32_t ECRC_CheckCapability:1;    // 7	      0x1	        ,     ECRC   .    APB  
  __IO  uint32_t EnableECRC_Check:1;        // 8	      0x0	           ERCR   .    APB  
  __IO  uint32_t MultipleHeadRecCapable:1;  // 9	      0x0	        ,          Header Log Registers.   0
  __IO  uint32_t MultipleHeadRecEn:1;       // 10	      0x0	               Header Log Registers.   0
  __I   uint32_t :21;                       // 31:11	  0x0	        Reserved    
  }             AdvcdErrCapCtrl;            // 0x0118		0x000000A0	     
  __IO uint32_t HeadLog0;                   // 0x011C		0x0	            0
  __IO uint32_t HeadLog1;                   // 0x0120		0x0	            1
  __IO uint32_t HeadLog2;                   // 0x0124		0x0	            2
  __IO uint32_t HeadLog3;                   // 0x0128		0x0	            3
}PCI_EndPoint_TypeDef;

/**
  * @brief PCI_RootPort structure definition  
  */
typedef struct{
//                          		       
  PCI_ID_TypeDef ID;                        // 0x0000		0x010017CD	   (Vendor ID  Device ID)
  PCI_CMD_TypeDef Command;                  // 0x0004		0x00100000	 
  PCI_RevClass_TypeDef RevisionClass;       // 0x0008		0x0	          (Revision ID  Class Code)
  PCI_BistHeader_TypeDef BistHeader;        // 0x000C	  0x0	           
  struct{
  __IO uint32_t Type:1;                     // 0	      0x0	            BAR 0 0  memory 1  IO     ,                             266
  __I  uint32_t :1;                         // 1	      0x0	        Reserved
  __IO uint32_t Size:1;                     // 2	      0x0	           memory: 0  32- , 1  64- .    IO  Base Address [2].     ,                             266
  __IO uint32_t Prefetch:1;                 // 3	      0x0	           memory     .    IO  Base Address [3].     ,                               266
  __IO uint32_t AddrRO:18;                  // 21:4	    0x0	           .        ,    266.    APB  
  __IO uint32_t AddrRW:10;                  // 31:22	  0x0	           .          266.    APB  
  }             RC_BAR_0,                   // 0x0010		0x0	           0
                RC_BAR_1;                   // 0x0014		0x0	           1
  struct{
  __IO uint32_t PrimaryBusNumber:8;         // 7:0	    0x0	                  APB,     
  __IO uint32_t SecondaryBusNumber:8;       // 15:8	    0x0	                  APB,     
  __IO uint32_t SubordinateBusNumber:8;     // 23:16	  0x0	                  APB,     
  __IO uint32_t SecondaryLatencyTimer:8;    // 31:24	  0x0	        Reserved    
  }             PcieBusNumbers;             // 0x0018		0x0	          
  struct{
  __IO uint32_t IO_BarSize:1;               // 0	      0x0	          .       ,    266  20 .   0   19  ,                                    266
  __I  uint32_t :3;                         // 3:1	    0x0	        Reserved
  __IO uint32_t IO_Base:4;                  // 7:4	    0x0	          .    APB  .   0,  IO BAR 
  __IO uint32_t BarSize:1;                  // 8	      0x0	            [0]
  __I  uint32_t :3;                         // 11:9	    0x0	        Reserved
  __IO uint32_t IO_Limit:4;                 // 15:12	  0x0	          .    APB  .   0,  IO BAR 
  __I  uint32_t :8;                         // 23:16	  0x0	        Reserved
  __IO uint32_t MasterDataParError:1;       // 24	      0x0	            .    1   APB.    APB      [13]  .   0,   Parity Error Response Enable ,    183,    0
  __I  uint32_t :2;                         // 26:25	  0x0	        Reserved
  __IO uint32_t SignaledTargetAbort:1;      // 27	      0x0	            .    1   APB.    APB      [13]  
  __IO uint32_t ReceivedTargetAbort:1;      // 28	      0x0	            .    1   APB.    APB      [13]  
  __IO uint32_t ReceivedMasterAbort:1;      // 29	      0x0	            .    1   APB.    APB      [13]  
  __IO uint32_t ReceivedSystemError:1;      // 30	      0x0	            .    1   APB.    APB      [13]  
  __IO uint32_t DetectedParityError:1;      // 31	      0x0	            .    1   APB.    APB      [13]  
  }             PcieIoBaseLim;              // 0x001C		0x0	           IO
  struct{
  __I  uint32_t :4;                         // 3:0	    0x0	        Reserved
  __IO uint32_t Base:12;                    // 15:4	    0	            .    APB  
  __I  uint32_t :4;                         // 19:16	  0x0	        Reserved
  __IO uint32_t Limit:12;                   // 31:20	  0             .    APB  
  }             PcieMemBaseLim;             // 0x0020		0x0	           Memory
  struct{
  __IO uint32_t Base:16;                    // 15:0	    0	            .    APB  .   0,     Type1 cfg prefetchable mem bar enable  ,    266
  __IO uint32_t Limit:16;                   // 31:16	  0	            .    APB  .   0,     Type1 cfg prefetchable mem bar enable  ,    266
  }             PciePrefetchBaseLim;        // 0x0024		0x0	           Memory  
  __IO uint32_t PciePrefetchBaseUpper;      // 0x0028		0x0	           Memory   (   )
  __IO uint32_t PciePrefetchLimUpper;       // 0x002C	  0x0	           Memory   (  )
  struct{
  __IO uint32_t Base:16;                    // 15:0	    0	            .    APB  .   0,  32bit IO     ,    266
  __IO uint32_t Limit:16;                   // 31:16	  0	            .    APB  .   0,  32bit IO     ,    266
  }             PcieIoBaseLimUpper;         // 0x0030		0x0	           IO ( )
  __IO uint32_t CapabilitiesPointer;        // 0x0034		0x80	           (PCI Capability Structure)
  __I  uint32_t res00;                      // 0x0038                
  struct{
  __IO uint32_t Line:8;                     // 7:0	    0xff	           RP.    APB  
  __IO uint32_t Pin:3;                      // 10:8	    0x01	      ,    Legacy ,    0x1  INTA 0x2  INTB 0x3  INTC 0x4  INTD    APB  
  __I  uint32_t :5;                         // 15:11	  0x0	        Reserved
  __IO uint32_t ParErrorResponseEn:1;       // 16	      0x0	           Master Data Parity Error.    APB  
  __IO uint32_t BridgeControlSERR_En:1;     // 17	      0x0	            .    APB  
  __IO uint32_t ISA_Enable:1;               // 18	      0x0	            .    APB  
  __IO uint32_t VGA_Enable:1;               // 19	      0x0	            .    APB  
  __IO uint32_t VGA_16_Decode:1;            // 20	      0x0	            .    APB  
  __I  uint32_t :1;                         // 21	      0x0	        Reserved
  __IO uint32_t BusResetControl:1;          // 22	      0x0	           .    APB  
  __I  uint32_t :9;                         // 31:23	  0x0	        Reserved
}               IrqLineIrqPin;              // 0x003C		0x1FF	        Legacy-
  __I  uint8_t  res01[0x80-0x40];           // 0x0040               
  PCI_PWR_MGMNT_TypeDef   PowerManagement;  // 0x0080		0x5A039001	    
  PCI_PWR_CTRL_TypeDef    PowerControl;     // 0x0084		0x8	          
  __I  uint32_t res02[2];                   // 0x0088               
  PCI_MSI_CTRL_TypeDef    MsiControl;       // 0x0090	  0x0180B005	  MSI-
  PCI_MSI_MSG_LA_TypeDef  MsiMsgLowAddr;    // 0x0094	  0x0	          MSI- ( )
  __IO uint32_t MsiMsgHiAddr;               // 0x0098	  0x0	          MSI- ( )
  __IO uint32_t MsiMsgData;                 // 0x009C	  0x0	          MSI-
  __IO uint32_t MsiMask;                    // 0x00A0	  0x0	             MSI-
  __IO uint32_t MsiPendingBits;             // 0x00A4	  0x0	          MSI-
  __I  uint32_t res03[2];                   // 0x00A8               
  PCI_MSIX_CTRL_TypeDef MsixControl;        // 0x00B0	  0x0000C011	  MSI-X-
  PCI_MSIX_TBLOFFSET_TypeDef MsixTblOffset; // 0x00B4	  0x0	            MSI-X-
  PCI_MSIX_PENDI_TypeDef MsixPendingIrq;    // 0x00B8	  0x00000008	    MSI-X-
  __I  uint32_t res04;                      // 0x00BC               
  PCI_CAPLIST_TypeDef CapList;              // 0x00C0		0x00420010	   PCIe
  PCI_DEV_CAP_TypeDef DevCap;               // 0x00C4		0x00008003	   PCIe
  PCI_CTRL_STAT_TypeDef DevCtrlStatus;      // 0x00C8   0x00002810	     PCIe
  PCI_LINC_CAP_TypeDef LinkCap;             // 0x00CC		0x0061AC42	  
  PCI_LINC_CTRL_STAT_TypeDef LinkCtrlStatus;// 0x00D0		0x00420000	    
  __I  uint32_t res05;                      // 0x00D4                  
  __IO uint32_t SlotCtrlStatus;             // 0x00D8		0x00400000	    
  __IO uint32_t RootCtrlCap;                // 0x00DC		0x0	          
  __IO uint32_t RootStatus;                 // 0x00E0		0x0	          
  __IO uint32_t PcieCap2;                   // 0x00E4		0x00100012	   PCIe 2
  __IO uint32_t PcieDevCtrlStatus2;         // 0x00E8		0x0	             PCIe 2
  __IO uint32_t LinkCap2;                   // 0x00EC		0x00000006	   2
  __IO uint32_t LinkCtrlStatus2;            // 0x00F0		0x00010002	     2
  __I  uint8_t  res06[0x100-0xF4];          // 0x00F4                   
  __IO uint32_t AerEhncdCap;                // 0x0100		0x00020001	    
  __IO uint32_t UncorrErrStatus;            // 0x0104		0x0	           
  __IO uint32_t UncorrErrMask;              // 0x0108		0x00400000	   
  __IO uint32_t UncorrErrSeverity;          // 0x010C		0x00462030	   
  __IO uint32_t CorrErrStatus;              // 0x0110		0x0	           
  __IO uint32_t CorrErrMask;                // 0x0114		0x0000E000	   
  __IO uint32_t AdvErrCapCtl;               // 0x0118		0x000000A0	     
  __IO uint32_t HeadLog0;                   // 0x011C		0x0	            0
  __IO uint32_t HeadLog1;                   // 0x0120		0x0	            1
  __IO uint32_t HeadLog2;                   // 0x0124		0x0	            2
  __IO uint32_t HeadLog3;                   // 0x0128		0x0	            3
  __IO uint32_t RootErrCmd;                 // 0x012C		0x0	            
  __IO uint32_t RootErrStat;                // 0x0130		0x0	            
  __IO uint32_t ErrSrcId;                   // 0x0134		0x0	            
}PCI_RootPort_TypeDef;

#define LINK_UP_Msk                     0x00000001
#define NEGOTIATED_LANE_COUNT_Msk       0x00000006
#define LINK_WIDTH_X1                   0x00000000
#define LINK_WIDTH_X2                   0x00000002
#define LINK_WIDTH_X4                   0x00000004
#define LINK_WIDTH_X8                   0x00000006 

#define NEGOTIATED_SPEED_Msk            0x00000018
#define NEGOTIATED_SPEED_2_5G           0x00000000
#define NEGOTIATED_SPEED_5G             0x00000008

#define BAR_APERTURE_4B                 0x00000000
#define BAR_APERTURE_8B                 0x00000001
#define BAR_APERTURE_16B                0x00000002
#define BAR_APERTURE_32B                0x00000003
#define BAR_APERTURE_64B                0x00000004
#define BAR_APERTURE_128B               0x00000005
#define BAR_APERTURE_256B               0x00000006
#define BAR_APERTURE_512B               0x00000007
#define BAR_APERTURE_1KB                0x00000008
#define BAR_APERTURE_2KB                0x00000009
#define BAR_APERTURE_4KB                0x0000000A
#define BAR_APERTURE_8KB                0x0000000B
#define BAR_APERTURE_16KB               0x0000000C
#define BAR_APERTURE_32KB               0x0000000D
#define BAR_APERTURE_64KB               0x0000000E
#define BAR_APERTURE_128KB              0x0000000F
#define BAR_APERTURE_256KB              0x00000010
#define BAR_APERTURE_512KB              0x00000011
#define BAR_APERTURE_1MB                0x00000012
#define BAR_APERTURE_2MB                0x00000013
#define BAR_APERTURE_4MB                0x00000014
#define BAR_APERTURE_8MB                0x00000015
#define BAR_APERTURE_16MB               0x00000016
#define BAR_APERTURE_32MB               0x00000017
#define BAR_APERTURE_64MB               0x00000018
#define BAR_APERTURE_128MB              0x00000019
#define BAR_APERTURE_256MB              0x0000001A
#define BAR_APERTURE_512MB              0x0000001B
#define BAR_APERTURE_1GB                0x0000001C
#define BAR_APERTURE_2GB                0x0000001D
#define BAR_APERTURE_4GB                0x0000001E
#define BAR_APERTURE_8GB                0x0000001F
#define BAR_APERTURE_16GB               0x00000020
#define BAR_APERTURE_32GB               0x00000021
#define BAR_APERTURE_64GB               0x00000022
#define BAR_APERTURE_128GB              0x00000023
#define BAR_APERTURE_256GB              0x00000024

#define BAR_CFG_CTRL_DISABLED           0x00000000
#define BAR_CFG_CTRL_IO                 0x00000040
#define BAR_CFG_CTRL_MEM                0x00000100
#define BAR_CFG_CTRL_PREFETCH_MEM	      0x00000140

#define MAX_PAYLOAD_128B                0
#define MAX_PAYLOAD_256B                1
#define MAX_PAYLOAD_512B                2
#define MAX_PAYLOAD_1024B               3
#define MAX_PAYLOAD_2048B               4
#define MAX_PAYLOAD_4096B               5

/**
  * @brief PCI_LocMgmt structure definition  
  */
typedef struct{
//                        		     
  
  __IO uint32_t  PhyConf0;              // 0x1000		0x00000024	    0
  __IO uint32_t  PhyConf1;              // 0x1004		0x40808000	    1
  __IO uint32_t  PackTimConf;           // 0x1008		0x0	            
  __IO uint32_t  RxCredLim0;            // 0x100C		0x20020200	    0
  __IO uint32_t  RxCredLim1;            // 0x1010		0x00000020	    1
  __I  uint32_t  TxCredLim0;            // 0x1014		0x0	            0
  __I  uint32_t  TxCredLim1;            // 0x1018		0x0	            1
  __IO uint32_t  TxCredUpdateConf0;     // 0x101C		0x00040004	    0
  __IO uint32_t  TxCredUpdateConf1;     // 0x1020		0x03AA0004	    1
  __IO uint32_t  LosTimeoutLimit;       // 0x1024		0x00000177	  -   L0s
  __IO uint32_t  TxTlpCount;            // 0x1028		0x0	            TLP
  __IO uint32_t  TxTlpPayloadDwCount;   // 0x102C		0x0	            
  __IO uint32_t  RxTlpCount;            // 0x1030		0x0	            TLP
  __IO uint32_t  RxTlpPayloadDwCount;   // 0x1034		0x0	            
  __IO uint32_t  ComplnTimeoutLim0;     // 0x1038		0x00BEBC20	  -             0
  __IO uint32_t  ComplnTimeoutLim1;     // 0x103C		0x02FAF080	  -            1
  __IO uint32_t  L1_ReentryDelay;       // 0x1040		0x0	              L1 
  __IO uint32_t  VendorId;              // 0x1044		0x17cd17cd	   
  __IO uint32_t  AspmTimeoutEntryL1;    // 0x1048		0x000002EE	  -   L1
  __IO uint32_t  PmeAckDelayOff;        // 0x104C		0x00000064	   
  __IO uint32_t  LinkWidthControl;      // 0x1050		0x0000000F	   
  __I  uint8_t   res00[0x74-0x54];      // 0x1054                   
  __IO uint32_t  SrisControl;           // 0x1074		0x0	          SRIS
  __I  uint8_t   res01[0x100-0x78];     // 0x1078                   
  __IO uint32_t  ShadowHeaderLog0;      // 0x1100		0x0	            0
  __IO uint32_t  ShadowHeaderLog1;      // 0x1104		0x0	            1
  __IO uint32_t  ShadowHeaderLog2;      // 0x1108		0x0	            2
  __IO uint32_t  ShadowHeaderLog3;      // 0x110C		0x0	            3
  __IO uint32_t  ShadowFuncNum;         // 0x1110		0x0	           
  __O  uint32_t  Shadow_UR_Err;         // 0x1114		0x0	            
  __I  uint8_t   res02[0x44-0x18];      // 0x1118                 
  __I  uint32_t  DbgDllpCountGen1;      // 0x1144		0x0	               Gen1
  __I  uint32_t  DbgDllpCountGen2;      // 0x1148		0x0	               Gen2
  __I  uint32_t  DbgDllpCountGen3;      // 0x114C		0x0	               Gen3
  __I  uint8_t   res03[0x200-0x150];    // 0x1150               
  __I  uint32_t  NegotiatedLaneMap;     // 0x1200		0x0	           
  __I  uint32_t  FastTrainingCount;     // 0x1204		0x0	            
  __IO uint32_t  DbgControl;            // 0x1208		0x80000000	  
  __IO uint32_t  LocalErrorStatus;      // 0x120C		0x0	            
  __IO uint32_t  LocalErrorMask;        // 0x1210		0x022C0FFF	   
  __IO uint32_t  LcrcErrCount;          // 0x1214		0x0	           LCRC
  __IO uint32_t  EccCorrErrCount;       // 0x1218		0x0	            ECC
  __I  uint32_t  res04[2];              // 0x121C                 
  __IO uint32_t  PmeServiceTimeout;     // 0x1224		0x000186A0	  -  PM_PME
  __IO uint32_t  RootPortId;            // 0x1228		0x0	          
  __I  uint32_t  BusDeviceNumber;       // 0x122C		0x0	            
  __I  uint32_t  res05;                 // 0x1230                   
  __IO uint32_t  DbgControl2;           // 0x1234		0x0	           2
  __I  uint32_t  res06[2];              // 0x1238                    
  __IO uint32_t  PhyFuncBarConf0;       // 0x1240		0x05058D8B	  BAR   0
  __IO uint32_t  PhyFuncBarConf1;       // 0x1244		0x00000505	  BAR   1
  __I  uint8_t   res07[0xC0-0x48];      // 0x1248                   
  __IO uint32_t  PhyFuncConf;           // 0x12C0		0x00000001	   
  __I  uint8_t   res08[0x100-0xC4];     // 0x12C4                   
  __IO uint32_t  BarConf;               // 0x1300		0x00002914	  BAR 
  __I  uint8_t   res09[0xC80-0x304];    // 0x1304                   
  __IO uint32_t  EccAxiCorrErrCount;    // 0x1C80		0x0	            ECC AXI 
  __I  uint32_t  res10;                 // 0x1C84                 
  __IO uint32_t  LowPowerDbgControl0;   // 0x1C88		0x0	               0
  __IO uint32_t  LowPowerDbgControl1;   // 0x1C8C		0x0	               1
  __I  uint32_t  res11;                 // 0x1C90                 
  __IO uint32_t  TransLevelControl;     // 0x1C94		0x0	           
  __I  uint8_t   res12[0xDA0-0xC98];    // 0x1C98                   
  __IO uint32_t  IndLinkOff;            // 0x1DA0		0x015F5E10	   
  __IO uint32_t  PulseFilterControl;    // 0x1DA4		0x04200000	   
}PCI_LocMgmt_TypeDef;

#define AXI_OBR_DESC0_TYPE_Msk                  0x00000007
#define AXI_OBR_DESC0_TYPE_IO_WRITE		0x6
#define AXI_OBR_DESC0_TYPE_MEM_WRITE		0x2
#define AXI_OBR_DESC0_TYPE_CFG_TYPE0		0xa
#define AXI_OBR_DESC0_TYPE_CFG_TYPE1		0xb
#define AXI_OBR_DESC0_TYPE_NOR_MSG		0xc

#define AXI_OBR_DESC0_HARDCODED_RID             0x01000000

/**
  * @brief PCI_OutboundRegion structure definition  
  */
typedef struct{
  __IO uint32_t  AddrTrans0;           // 0x00                  0x0	         0(Outbound Region 0)
  __IO uint32_t  AddrTrans1;           // 0x04                  0x0	         1(Outbound Region 0)
  __IO uint32_t  HeaderDesc0;          // 0x08                  0x0	        0(Outbound Region 0)
  __IO uint32_t  HeaderDesc1;          // 0x0                  0x0	        1(Outbound Region 0)
  __IO uint32_t  res00[2];             // 0x10-0x14             
  __IO uint32_t  BaseAddr0;            // 0x18                  0x0	         0(Outbound Region 0)
  __IO uint32_t  BaseAddr1;            // 0x1C                  0x0	         0(Outbound Region 0)
}PCI_OutboundRegion_TypeDef, PCI_OBR;


/**
  * @brief PCI_InboundRegion structure definition  
  */
typedef struct{
  __IO uint32_t  AddrTrans0;           // 0x00                  0x0	      Регистр адреса трансляции 0(Inbound Region x)
  __IO uint32_t  AddrTrans1;           // 0x04                  0x0	      Регистр адреса трансляции 1(Inbound Region x) 
}PCI_InboundRegion_TypeDef, PCI_IBR;

/**
  * @brief PCI_AXI_ structure definition  
  */
typedef struct{
//                          	       
  __IO PCI_OBR   OutboundRegion[32];        // 0x4000                  ,   Outbound Region X
  __I  uint8_t   res01[0x800-0x400];        // 0x4400                 
  __IO PCI_IBR   InboundRegion[3];          // 0x4800	  	      Группа регистров, адреса трансляции Inbound Region X	
  __I  uint32_t  res02[3];                  // 0x4818                 
  __IO uint32_t  Axi10;                     // 0x4824	  0x0	
  __IO uint32_t  InRegEP_Bar0_AddrTrans[2]; // 0x4840	  0x0         inregion_ep_bar_0_addr_translation
  __IO uint32_t  InRegEP_Bar1_AddrTrans[2]; // 0x4848	  0x0	      inregion_ep_bar_1_addr_translation	
  __IO uint32_t  InRegEP_Bar2_AddrTrans[2]; // 0x4850	  0x0	      inregion_ep_bar_2_addr_translation	
  __IO uint32_t  InRegEP_Bar3_AddrTrans[2]; // 0x4858	  0x0	      inregion_ep_bar_3_addr_translation	
  __IO uint32_t  InRegEP_Bar4_AddrTrans[2]; // 0x4860	  0x0	      inregion_ep_bar_4_addr_translation	
  __IO uint32_t  InRegEP_Bar5_AddrTrans[2]; // 0x4868	  0x0	      inregion_ep_bar_5_addr_translation	
  __IO uint32_t  InRegEP_Bar6_AddrTrans[2]; // 0x4870	  0x0	      inregion_ep_bar_6_addr_translation
}PCI_AXI_TypeDef;

/**
  * @brief PCI_DMA structure definition  
  */
typedef struct{
//                          		     
  __IO uint32_t Channel0_Control;         // 0x6000		0x0	          0
  __IO uint32_t Channel0_SPL;             // 0x6004		0x0	            0 ( )
  __IO uint32_t Channel0_SPU;             // 0x6008		0x0	            0 ( )
  __IO uint32_t Channel0_AttrL;           // 0x600C		0x0	          0 ( )
  __IO uint32_t Channel0_AttrU;           // 0x6010		0x0	          0 ( )
  __IO uint32_t Channel1_Control;         // 0x6014		0x0	          1
  __IO uint32_t Channel1_SPL;             // 0x6018		0x0	            1 ( )
  __IO uint32_t Channel1_SPU;             // 0x601C		0x0	            1 ( )
  __IO uint32_t Channel1_AttrL;           // 0x6020		0x0	          1 ( )
  __IO uint32_t Channel1_AttrU;           // 0x6024		0x0	          1 ( )
  __I  uint8_t  res00[0xA0-0x28];         // 0x6028               
  __IO uint32_t CommUdmaIrq;              // 0x60A0		0x0	         
  __IO uint32_t CommUdmaIrqEn;            // 0x60A4		0x0000FFFF	 
  __IO uint32_t CommUdmaIrqDis;           // 0x60A8		0x0000FFFF	 
  __I  uint32_t CommUdmaInUncorrErrors;   // 0x60AC		0x0	            
  __I  uint32_t CommUdmaInCorrErrors;     // 0x60B0		0x0	            
  __I  uint32_t CommUdmaOutUncorrErrors;  // 0x60B4		0x0	            
  __I  uint32_t CommUdmaOutCorrErrors;    // 0x60B8		0x0	            
  __I  uint8_t  res01[0xF8-0xBC];         // 0x60BC               
  __I  uint32_t CommUdmaCapVer;           // 0x60F8		0x00000001	    
  __I  uint32_t CommUdmaConfig;           // 0x60FC		0x00004622	  
}PCI_DMA_TypeDef;

/**
  * @brief PCI_DMA structure definition  
  */
typedef struct{
  union {
    PCI_EndPoint_TypeDef EndPoint;
    PCI_RootPort_TypeDef RootPort;
  };
  __I  uint8_t  res00[0x1000-0x0138];     //                
  PCI_LocMgmt_TypeDef LocMgmt;
  __I  uint8_t  res01[0x4000-0x1DA8];     //                
  PCI_AXI_TypeDef AXI;
  __I  uint8_t  res02[0x6000-0x4878];     //                
  PCI_DMA_TypeDef DMA;
}PCI_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  
#define PCIE                   ((PCI_TypeDef *)PCI_CONT_BASE)
#define PCIE_PHY               ((PCI_PHY_TypeDef *)(SCTL_BASE + 0x400))
#define PCIE_WINDOW            (0x40000000)
  
/* Initialization and Configuration functions *********************************/

#define PCIE_BUS(x)			(((x) & 0xff) << 20)
#define PCIE_DEV(x)			(((x) & 0x1f) << 15)
#define PCIE_FUNC(x)			(((x) & 0x7) << 12)
#define PCIE_REG(x)			(((x) & 0xfff) << 0)
#define PCIE_ADDR(bus, dev, func, reg) \
	  (PCIE_BUS(bus) | PCIE_DEV(dev) | \
	   PCIE_FUNC(func) | PCIE_REG(reg))

/* GPIO Read and Write functions **********************************************/
extern void init_dit0_pci(void);
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_PCI_H */
/**
  * @}
  */
  