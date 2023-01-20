/**
  ******************************************************************************
  * @file    1888bc048_gic.h
  * @author  A. Maksimov
  * @version V1.0.1
  * @date    19.11.2019
  * @brief   This file contains all the functions prototypes for the GIC 
  *          firmware library. 
  ******************************************************************************
  */
#ifndef __1888BC048_GIC_H
#define __1888BC048_GIC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "1888bc048.h"

/** @addtogroup __1888BC048_StdPeriph_Driver
  * @{
  */
  
/**
  * @brief IRQ_NUM structure definition  
  */

typedef struct{
  uint32_t  SGI0_Int         :1; // 0,  /*!<   0  GIC390 */
  uint32_t  SGI1_Int         :1; // 1,  /*!<   1  GIC390 */
  uint32_t                   :30; // 32-2
//  SPI  -    
  uint32_t DDC_RX_Int        :1; // 32,  /*!<   (DCC) RX */
  uint32_t DDC_TX_Int        :1; // 33,  /*!<   (DCC) TX */
  uint32_t                   :1; // 34
  uint32_t TERMO_Int         :1; // 35,  /*!<     */
  uint32_t MDIO0_Int         :1; // 36,  /*!<  0  MDIO */
  uint32_t MDIO1_Int         :1; // 37,  /*!<  1  MDIO */
  uint32_t MDIO2_Int         :1; // 38,  /*!<  2  MDIO */
  uint32_t MDIO3_Int         :1; // 39,  /*!<  3  MDIO */
  uint32_t MDIO_GPIO0_Int    :1; // 40,  /*!<  0 GPIO  MDIO */
  uint32_t MDIO_GPIO1_Int    :1; // 41,  /*!<  1 GPIO  MDIO */
  uint32_t DIT0_TIM1_Int     :1; // 42,  /*!<    DIT0    1 */
  uint32_t DIT0_TIM2_Int     :1; // 43,  /*!<    DIT0    2 */
  uint32_t DIT1_TIM1_Int     :1; // 44,  /*!<    DIT1    1 */
  uint32_t DIT1_TIM2_Int     :1; // 45,  /*!<    DIT1    2 */
  uint32_t DIT2_TIM1_Int     :1; // 46,  /*!<    DIT2    1 */
  uint32_t DIT2_TIM2_Int     :1; // 47,  /*!<    DIT2    2 */
  uint32_t DIT3_TIM1_Int     :1; // 48,  /*!<    DIT3    1 */
  uint32_t DIT3_TIM2_Int     :1; // 49,  /*!<    DIT3    2 */
  uint32_t                   :1; // 50
  uint32_t MGETH0_Int        :1; // 51,  /*!<    MGETH0 */
  uint32_t MGETH1_Int        :1; // 52,  /*!<    MGETH1 */
  uint32_t MGETH2_Int        :1; // 53,  /*!<    MGETH2 */
  uint32_t MGETH3_Int        :1; // 54,  /*!<    MGETH3 */
  uint32_t PCIE_DMA_Int      :1; // 55,  /*!< DMA   PCIe */
  uint32_t PCIE_Local_Int    :1; // 56,  /*!< Local error   PCIe */
  uint32_t PCIEa_Int         :1; // 57,  /*!< PCI legacy interrupt a  PCIe */
  uint32_t PCIEb_Int         :1; // 58,  /*!< PCI legacy interrupt b  PCIe */
  uint32_t PCIEc_Int         :1; // 59,  /*!< PCI legacy interrupt c  PCIe */
  uint32_t PCIEd_Int         :1; // 60,  /*!< PCI legacy interrupt d  PCIe */
  uint32_t PCIE_Slv_Int      :1; // 61,  /*!<      AXI   PCIe */
  uint32_t PCIE_Mst_Int      :1; // 62,  /*!<      AXI   PCIe */
  uint32_t WDT_Int           :1; // 63,  /*!<     */
//  
  uint32_t MDMA0_Int         :1; // 64,  /*!<    MDMA0 */
  uint32_t MDMA1_Int         :1; // 65,  /*!<    MDMA1 */
  uint32_t MDMA2_Int         :1; // 66,  /*!<    MDMA2 */
  uint32_t MDMA3_Int         :1; // 67,  /*!<    MDMA3 */
  uint32_t MKIO0_Int         :1; // 68,  /*!<    MKIO0 */
  uint32_t MKIO1_Int         :1; // 69,  /*!<    MKIO1 */
  uint32_t MKIO2_Int         :1; // 70,  /*!<    MKIO2 */
  uint32_t MKIO3_Int         :1; // 71,  /*!<    MKIO3 */
  uint32_t SDIO0_Int         :1; // 72,  /*!<    GSPI_SDIO0 */
  uint32_t SDIO1_Int         :1; // 73,  /*!<    GSPI_SDIO1 */
  uint32_t I2C0_Int          :1; // 74,  /*!<    I2C0 */
  uint32_t I2C1_Int          :1; // 75,  /*!<    I2C1 */
  uint32_t UART0_Int         :1; // 76,  /*!<     UART0 */
  uint32_t UART1_Int         :1; // 77,  /*!<     UART1 */
  uint32_t GPIO0_Int         :1; // 78,  /*!<     GPIO0 */
  uint32_t GPIO1_Int         :1; // 79,  /*!<     GPIO1 */
  uint32_t GPIO2_Int         :1; // 80,  /*!<     GPIO2 */
  uint32_t GPIO3_Int         :1; // 81,  /*!<     GPIO3 */
  uint32_t GPIO4_Int         :1; // 82,  /*!<     GPIO4 */
  uint32_t CAN0_Int          :1; // 83,  /*!<  CAN0 */
  uint32_t CAN1_Int          :1; // 84,  /*!<  CAN1 */
  uint32_t ARINC_Int         :1; // 85,  /*!<   ARINC */
  uint32_t SGMII_Int         :1; // 86,  /*!<   SGMII */
  uint32_t DDR0_Int          :1; // 87,  /*!<   DDR0 */
  uint32_t DDR1_Int          :1; // 88,  /*!<   DDR1 */
  uint32_t GTIM_Int          :1; // 89,  /*!<   gp_global_timers */
  uint32_t CRG_SYS_Int       :1; // 90,  /*!<    PLL  crg_sys */
  uint32_t CRG_DDR_Int       :1; // 91,  /*!<    PLL  crg_ddr */
  uint32_t PMU_Int           :1; // 92,  /*!<   PMU */
  uint32_t L2C_Int           :1; // 93,  /*!<   L2C */
  uint32_t L2C_SlvErr_Int    :1; // 94,  /*!<  Slave Error L2C */
  uint32_t L2C_DecErr_Int    :1; // 95,  /*!<  Decode Error L2C */
}IRQ_NUM_Typedef;

/**
  * @brief GIC_DIS structure definition  
  */
typedef struct{
  __IO uint32_t Enable;         /*!<   GIC,   */
  struct{
    __I  uint32_t ITLinesNumber:  5;  /*!<   GIC   96 ,      2 */
    __I  uint32_t CPUNumber	:     3;  /*!<     ,     000    */
    __I  uint32_t:                2;
    __I  uint32_t SecurityExtn:   1;  /*!<    ,      1 */
    __I  uint32_t:                21;
  }TYPER;                     /*!<   GIC */
  struct{
    __I  uint32_t Implementer:    12; /*!<     JEP106 (0x0000043B    ARM) */
    __I  uint32_t Revision:       4;  /*!<   */
    __I  uint32_t:                16;
  }IIDR;                              /*!<   GIC */           
  __I  uint32_t res0[(0x080-0x008-4)/4]; /*!<  */
  __IO uint32_t IRQ_GROUP0;           /*!<  0    */
  __IO uint32_t IRQ_GROUP1;           /*!<  1    */
  __IO uint32_t IRQ_GROUP2;           /*!<  2    */   
  __I  uint32_t res1[(0x100-0x088-4)/4]; /*!<  */
  __IO IRQ_NUM_Typedef SET_IRQ;       /*!<    */        
  __I  uint32_t res2[(0x180-0x108-4)/4]; /*!<  */
  __IO IRQ_NUM_Typedef CLR_IRQ;       /*!<    */       
  __I  uint32_t res3[(0x200-0x188-4)/4]; /*!<  */
  __IO IRQ_NUM_Typedef SET_PEND;      /*!<      */   
  __I  uint32_t res4[(0x280-0x208-4)/4]; /*!<  */
  __IO IRQ_NUM_Typedef CLR_PEND;      /*!<      */   
  __I  uint32_t res5[(0x300-0x288-4)/4]; /*!<  */
  __I  IRQ_NUM_Typedef IN_ACT;        /*!<  0    */  
  __I  uint32_t res6[(0x400-0x308-4)/4]; /*!<  */
//    0  7...  
  __IO uint32_t IPRIORITY0;   /*!<  0  */
  __IO uint32_t IPRIORITY1;   /*!<  1  */                       
  __I  uint32_t res7[(0x420-0x404-4)/4]; /*!<  */
  struct{//   32  95...
  //  GICD_IPRIORITY8
    __IO uint32_t : 3;  __IO uint32_t DDC_RX_Ip:      5;  /*!<   (DCC) RX */
    __IO uint32_t : 3;  __IO uint32_t DDC_TX_Ip:      5;  /*!<   (DCC) TX */
    __IO uint32_t : 3;  __IO uint32_t IP_34:          5;  /*!<   34 */
    __IO uint32_t : 3;  __IO uint32_t TERMO_Ip:       5;  /*!<     */
  //  GICD_IPRIORITY9
    __IO uint32_t : 3;  __IO uint32_t MDIO0_Ip:       5;  /*!<  0  MDIO */
    __IO uint32_t : 3;  __IO uint32_t MDIO1_Ip:       5;  /*!<  1  MDIO */
    __IO uint32_t : 3;  __IO uint32_t MDIO2_Ip:       5;  /*!<  2  MDIO */
    __IO uint32_t : 3;  __IO uint32_t MDIO3_Ip:       5;  /*!<  3  MDIO */
  //  GICD_IPRIORITY10
    __IO uint32_t : 3;  __IO uint32_t MDIO_GPIO0_Ip:  5;  /*!<  0 GPIO  MDIO */
    __IO uint32_t : 3;  __IO uint32_t MDIO_GPIO1_Ip:  5;  /*!<  1 GPIO  MDIO */
    __IO uint32_t : 3;  __IO uint32_t DIT0_TIM1_Ip:   5;  /*!<    DIT0    1 */
    __IO uint32_t : 3;  __IO uint32_t DIT0_TIM2_Ip:   5;  /*!<    DIT0    2 */
  //  GICD_IPRIORITY11
    __IO uint32_t : 3;  __IO uint32_t DIT1_TIM1_Ip:   5;  /*!<    DIT1    1 */
    __IO uint32_t : 3;  __IO uint32_t DIT1_TIM2_Ip:   5;  /*!<    DIT1    2 */
    __IO uint32_t : 3;  __IO uint32_t DIT2_TIM1_Ip:   5;  /*!<    DIT2    1 */
    __IO uint32_t : 3;  __IO uint32_t DIT2_TIM2_Ip:   5;  /*!<    DIT2    2 */
  //  GICD_IPRIORITY12
    __IO uint32_t : 3;  __IO uint32_t DIT3_TIM1_Ip:   5;  /*!<    DIT3    1 */
    __IO uint32_t : 3;  __IO uint32_t DIT3_TIM2_Ip:   5;  /*!<    DIT3    2 */
    __IO uint32_t : 3;  __IO uint32_t IP_50:          5;  /*!<   50 */
    __IO uint32_t : 3;  __IO uint32_t MGETH0_Ip:      5;  /*!<    MGETH0 */
  //  GICD_IPRIORITY13  
    __IO uint32_t : 3;  __IO uint32_t MGETH1_Ip:      5;  /*!<    MGETH1 */
    __IO uint32_t : 3;  __IO uint32_t MGETH2_Ip:      5;  /*!<    MGETH2 */
    __IO uint32_t : 3;  __IO uint32_t MGETH3_Ip:      5;  /*!<    MGETH3 */
    __IO uint32_t : 3;  __IO uint32_t PCIE_DMA_Ip:    5;  /*!< DMA   PCIe */
    //  GICD_IPRIORITY14
    __IO uint32_t : 3;  __IO uint32_t PCIE_Local_Ip:  5;  /*!< Local error   PCIe */
    __IO uint32_t : 3;  __IO uint32_t PCIEa_Ip:       5;  /*!< PCI legacy interrupt a  PCIe */
    __IO uint32_t : 3;  __IO uint32_t PCIEb_Ip:       5;  /*!< PCI legacy interrupt b  PCIe */
    __IO uint32_t : 3;  __IO uint32_t PCIEc_Ip:       5;  /*!< PCI legacy interrupt c  PCIe */
    //  GICD_IPRIORITY15
    __IO uint32_t : 3;  __IO uint32_t PCIEd_Ip:       5;  /*!< PCI legacy interrupt d  PCIe */
    __IO uint32_t : 3;  __IO uint32_t PCIE_Slv_Ip:    5;  /*!<      AXI   PCIe */
    __IO uint32_t : 3;  __IO uint32_t PCIE_Mst_Ip:    5;  /*!<      AXI   PCIe */
    __IO uint32_t : 3;  __IO uint32_t WDT_Ip:         5;  /*!<     */
    //  GICD_IPRIORITY16
    __IO uint32_t : 3;  __IO uint32_t MDMA0_Ip:       5;  /*!<    MDMA0 */
    __IO uint32_t : 3;  __IO uint32_t MDMA1_Ip:       5;  /*!<    MDMA1 */
    __IO uint32_t : 3;  __IO uint32_t MDMA2_Ip:       5;  /*!<    MDMA2 */
    __IO uint32_t : 3;  __IO uint32_t MDMA3_Ip:       5;  /*!<    MDMA3 */
    //  GICD_IPRIORITY17
    __IO uint32_t : 3;  __IO uint32_t MKIO0_Ip:       5;  /*!<    MKIO0 */
    __IO uint32_t : 3;  __IO uint32_t MKIO1_Ip:       5;  /*!<    MKIO1 */
    __IO uint32_t : 3;  __IO uint32_t MKIO2_Ip:       5;  /*!<    MKIO2 */
    __IO uint32_t : 3;  __IO uint32_t MKIO3_Ip:       5;  /*!<    MKIO3 */
    //  GICD_IPRIORITY18
    __IO uint32_t : 3;  __IO uint32_t SDIO0_Ip:       5;  /*!<    GSPI_SDIO0 */
    __IO uint32_t : 3;  __IO uint32_t SDIO1_Ip:       5;  /*!<    GSPI_SDIO1 */
    __IO uint32_t : 3;  __IO uint32_t I2C0_Ip:        5;  /*!<    I2C0 */
    __IO uint32_t : 3;  __IO uint32_t I2C1_Ip:        5;  /*!<    I2C1 */
    //  GICD_IPRIORITY19
    __IO uint32_t : 3;  __IO uint32_t UART0_Ip:       5;  /*!<     UART0 */
    __IO uint32_t : 3;  __IO uint32_t UART1_Ip:       5;  /*!<     UART1 */
    __IO uint32_t : 3;  __IO uint32_t GPIO0_Ip:       5;  /*!<     GPIO0 */
    __IO uint32_t : 3;  __IO uint32_t GPIO1_Ip:       5;  /*!<     GPIO1 */
    //  GICD_IPRIORITY20
    __IO uint32_t : 3;  __IO uint32_t GPIO2_Ip:       5;  /*!<     GPIO2 */
    __IO uint32_t : 3;  __IO uint32_t GPIO3_Ip:       5;  /*!<     GPIO3 */
    __IO uint32_t : 3;  __IO uint32_t GPIO4_Ip:       5;  /*!<     GPIO4 */
    __IO uint32_t : 3;  __IO uint32_t CAN0_Ip:        5;  /*!<  CAN0 */
    //  GICD_IPRIORITY21
    __IO uint32_t : 3;  __IO uint32_t CAN1_Ip:        5;  /*!<  CAN1 */
    __IO uint32_t : 3;  __IO uint32_t ARINC_Ip:       5;  /*!<   ARINC */
    __IO uint32_t : 3;  __IO uint32_t SGMII_Ip:       5;  /*!<   SGMII */
    __IO uint32_t : 3;  __IO uint32_t DDR0_Ip:        5;  /*!<   DDR0 */
    //  GICD_IPRIORITY22
    __IO uint32_t : 3;  __IO uint32_t DDR1_Ip:        5;  /*!<   DDR1 */
    __IO uint32_t : 3;  __IO uint32_t GTIM_Ip:        5;  /*!<   gp_global_timers */
    __IO uint32_t : 3;  __IO uint32_t CRG_SYS_Ip:     5;  /*!<    PLL  crg_sys */
    __IO uint32_t : 3;  __IO uint32_t CRG_DDR_Ip:     5;  /*!<    PLL  crg_ddr */
    //  GICD_IPRIORITY23
    __IO uint32_t : 3;  __IO uint32_t PMU_Ip:         5;  /*!<   PMU */
    __IO uint32_t : 3;  __IO uint32_t L2C_Ip:         5;  /*!<   L2C */
    __IO uint32_t : 3;  __IO uint32_t L2C_SlvErr_Ip:  5;  /*!<  Slave Error L2C */
    __IO uint32_t : 3;  __IO uint32_t L2C_DecErr_Ip:  5;  /*!<  Decode Error L2C */
  }PRIORITY;                  /*!<    32  95 */
  __I  uint32_t res8[(0xC00-0x45C-4)/4];
  __IO uint32_t ICFGR0;       /*!<  0      0  7 */
  __I  uint32_t res9;
  struct{//   32  95  ...
  // ICFGR2
    __IO uint32_t DDC_RX_Ic:      2;  /*!<   (DCC) RX */
    __IO uint32_t DDC_TX_Ic:      2;  /*!<   (DCC) TX */
    __IO uint32_t IP_34:          2;  /*!<   34 */
    __IO uint32_t TERMO_Ic:       2;  /*!<     */
    __IO uint32_t MDIO0_Ic:       2;  /*!<  0  MDIO */
    __IO uint32_t MDIO1_Ic:       2;  /*!<  1  MDIO */
    __IO uint32_t MDIO2_Ic:       2;  /*!<  2  MDIO */
    __IO uint32_t MDIO3_Ic:       2;  /*!<  3  MDIO */
    __IO uint32_t MDIO_GPIO0_Ic:  2;  /*!<  0 GPIO  MDIO */
    __IO uint32_t MDIO_GPIO1_Ic:  2;  /*!<  1 GPIO  MDIO */
    __IO uint32_t DIT0_TIM1_Ic:   2;  /*!<    DIT0    1 */
    __IO uint32_t DIT0_TIM2_Ic:   2;  /*!<    DIT0    2 */
    __IO uint32_t DIT1_TIM1_Ic:   2;  /*!<    DIT1    1 */
    __IO uint32_t DIT1_TIM2_Ic:   2;  /*!<    DIT1    2 */
    __IO uint32_t DIT2_TIM1_Ic:   2;  /*!<    DIT2    1 */
    __IO uint32_t DIT2_TIM2_Ic:   2;  /*!<    DIT2    2 */
  // ICFGR3
    __IO uint32_t DIT3_TIM1_Ic:   2;  /*!<    DIT3    1 */
    __IO uint32_t DIT3_TIM2_Ic:   2;  /*!<    DIT3    2 */
    __IO uint32_t IP_50:          2;  /*!<   50 */
    __IO uint32_t MGETH0_Ic:      2;  /*!<    MGETH0 */
    __IO uint32_t MGETH1_Ic:      2;  /*!<    MGETH1 */
    __IO uint32_t MGETH2_Ic:      2;  /*!<    MGETH2 */
    __IO uint32_t MGETH3_Ic:      2;  /*!<    MGETH3 */
    __IO uint32_t PCIE_DMA_Ic:    2;  /*!< DMA   PCIe */
    __IO uint32_t PCIE_Local_Ic:  2;  /*!< Local error   PCIe */
    __IO uint32_t PCIEa_Ic:       2;  /*!< PCI legacy interrupt a  PCIe */
    __IO uint32_t PCIEb_Ic:       2;  /*!< PCI legacy interrupt b  PCIe */
    __IO uint32_t PCIEc_Ic:       2;  /*!< PCI legacy interrupt c  PCIe */
    __IO uint32_t PCIEd_Ic:       2;  /*!< PCI legacy interrupt d  PCIe */
    __IO uint32_t PCIE_Slv_Ic:    2;  /*!<      AXI   PCIe */
    __IO uint32_t PCIE_Mst_Ic:    2;  /*!<      AXI   PCIe */
    __IO uint32_t WDT_Ic:         2;  /*!<     */
    // ICFGR4
    __IO uint32_t MDMA0_Ic:       2;  /*!<    MDMA0 */
    __IO uint32_t MDMA1_Ic:       2;  /*!<    MDMA1 */
    __IO uint32_t MDMA2_Ic:       2;  /*!<    MDMA2 */
    __IO uint32_t MDMA3_Ic:       2;  /*!<    MDMA3 */
    __IO uint32_t MKIO0_Ic:       2;  /*!<    MKIO0 */
    __IO uint32_t MKIO1_Ic:       2;  /*!<    MKIO1 */
    __IO uint32_t MKIO2_Ic:       2;  /*!<    MKIO2 */
    __IO uint32_t MKIO3_Ic:       2;  /*!<    MKIO3 */
    __IO uint32_t SDIO0_Ic:       2;  /*!<    GSPI_SDIO0 */
    __IO uint32_t SDIO1_Ic:       2;  /*!<    GSPI_SDIO1 */
    __IO uint32_t I2C0_Ic:        2;  /*!<    I2C0 */
    __IO uint32_t I2C1_Ic:        2;  /*!<    I2C1 */
    __IO uint32_t UART0_Ic:       2;  /*!<     UART0 */
    __IO uint32_t UART1_Ic:       2;  /*!<     UART1 */
    __IO uint32_t GPIO0_Ic:       2;  /*!<     GPIO0 */
    __IO uint32_t GPIO1_Ic:       2;  /*!<     GPIO1 */
    //  ICFGR5
    __IO uint32_t GPIO2_Ic:       2;  /*!<     GPIO2 */
    __IO uint32_t GPIO3_Ic:       2;  /*!<     GPIO3 */
    __IO uint32_t GPIO4_Ic:       2;  /*!<     GPIO4 */
    __IO uint32_t CAN0_Ic:        2;  /*!<  CAN0 */
    __IO uint32_t CAN1_Ic:        2;  /*!<  CAN1 */
    __IO uint32_t ARINC_Ic:       2;  /*!<   ARINC */
    __IO uint32_t SGMII_Ic:       2;  /*!<   SGMII */
    __IO uint32_t DDR0_Ic:        2;  /*!<   DDR0 */
    __IO uint32_t DDR1_Ic:        2;  /*!<   DDR1 */
    __IO uint32_t GTIM_Ic:        2;  /*!<   gp_global_timers */
    __IO uint32_t CRG_SYS_Ic:     2;  /*!<    PLL  crg_sys */
    __IO uint32_t CRG_DDR_Ic:     2;  /*!<    PLL  crg_ddr */
    __IO uint32_t PMU_Ic:         2;  /*!<   PMU */
    __IO uint32_t L2C_Ic:         2;  /*!<   L2C */
    __IO uint32_t L2C_SlvErr_Ic:  2;  /*!<  Slave Error L2C */
    __IO uint32_t L2C_DecErr_Ic:  2;  /*!<  Decode Error L2C */
  }CONFIG;  /*!<     32  95 */
  __I  uint32_t resA[(0xD04-0xC14-4)/4];
  __I  uint32_t IRAWST1;      /*!<  1      */
  __I  uint32_t IRAWST2;      /*!<  2      */
  __I  uint32_t resB[(0xF00-0xD08-4)/4];
  struct{
    __O  uint32_t SGIIINTID:        4;  /*!<    (Interrupt ID),     .  : 0-7 ( 0-7 ) */
    __O  uint32_t :                 11;
    __O  uint32_t NSATT:            1;
    __O  uint32_t CPUTargetList:    8;  /*!<       1 */
    __O  uint32_t TargetListFilter:	2;  /*!<       0 */
    __O  uint32_t :                 6;    
  }SGIR;                      /*!<      */
}GIC_DIS_TypeDef;  

/**
  * @brief  GIC_CPU structure definition  s
  */
typedef struct{
  struct{
    __IO uint32_t EnableGrp0: 1;  /*!<    0.  0  ,  1   */
    __IO uint32_t EnableGrp1: 1;  /*!<    1.  0  ,  1   */  
    __IO uint32_t AckCtl:     1;  /*!<      1.     GICC_IAR  GICC_EOIR */
    __IO uint32_t	FIQEn:      1;  /*!<   .  0    0   ARM   IRQ,  1    FIQ */
    __IO uint32_t SBPR:       1;  /*!<  ,         1. 
 0    GICC_BPR   1 (,    ,  GICC_ABPR), 
 1    GICC_BPR   0 (      0   1) */
    __IO uint32_t:            27;
  }CTRL;                      /*!<       */
  struct{
    __IO uint32_t:            3;
    __IO uint32_t Priority:   5;  /*!<   .             . */
    __IO uint32_t:            24;
  }PMR;                       /*!<    */
  __IO uint32_t BPR;    /*!<    - 3   */
  __I  uint32_t IAR;    /*!<    - 7   */
  __O  uint32_t EOIR;   /*!<     - 10   */
  struct{
    __I uint32_t:            3;
    __I uint32_t Priority:   5;  /*!<  ,    .     ,          */
    __I uint32_t:            24;
  }RPR;                 /*!<      */
  __I  uint32_t HPPIR;  /*!<        - 10   */
  __IO uint32_t ABPR;   /*!<     */
}GIC_CPU_TypeDef;

/** @addtogroup Peripheral_declaration
  * @{
  */  
#define GIC_DIS           ((volatile GIC_DIS_TypeDef *)GIC_DIS_BASE)
#define GIC_CPU           ((volatile GIC_CPU_TypeDef *)GIC_CPU_BASE)
//   GIC_DIS
#define GICD_CTLR	        GIC_DIS->Enable
#define GICD_TYPER	      GIC_DIS->TYPER
#define GICD_IIDR	        GIC_DIS->IIDR
#define GICD_IGROUPR0	    GIC_DIS->IRQ_GROUP0
#define GICD_IGROUPR1	    GIC_DIS->IRQ_GROUP1
#define GICD_IGROUPR2	    GIC_DIS->IRQ_GROUP2
//
#define GICD_ISENABLER(x) (*((uint32_t*)&(GIC_DIS->SET_IRQ) + (x)))
#define GICD_ISENABLER0	  GICD_ISENABLER(0)
#define GICD_ISENABLER1	  GICD_ISENABLER(1)
#define GICD_ISENABLER2	  GICD_ISENABLER(2)
//
#define GICD_ICENABLER(x) (*((uint32_t*)&(GIC_DIS->CLR_IRQ) + (x)))
#define GICD_ICENABLER0	  GICD_ICENABLER(0)
#define GICD_ICENABLER1	  GICD_ICENABLER(1)
#define GICD_ICENABLER2	  GICD_ICENABLER(2)
//
#define GICD_ISPENDR(x)   (*((uint32_t*)&(GIC_DIS->SET_PEND) + (x)))
#define GICD_ISPENDR0	    GICD_ISPENDR(0)
#define GICD_ISPENDR1	    GICD_ISPENDR(1)
#define GICD_ISPENDR2	    GICD_ISPENDR(2)
//
#define GICD_ICPENDR(x)   (*((uint32_t*)&(GIC_DIS->CLR_PEND) + (x)))
#define GICD_ICPENDR0	    GICD_ICPENDR(0)
#define GICD_ICPENDR1	    GICD_ICPENDR(1)
#define GICD_ICPENDR2	    GICD_ICPENDR(2)
//
#define GICD_ICDABR(x)    (*((uint32_t*)&(GIC_DIS->IN_ACT) + (x)))
#define GICD_ICDABR0	    GICD_ICDABR(0)
#define GICD_ICDABR1	    GICD_ICDABR(1)
#define GICD_ICDABR2	    GICD_ICDABR(2)
//
#define GICD_IPRIORITY0	  GIC_DIS->IPRIORITY0
#define GICD_IPRIORITY1	  GIC_DIS->IPRIORITY1
//
#define GICD_IPRIORITY(x) (*((uint32_t*)&(GIC_DIS->PRIORITY) + (x) - 8))
#define GICD_IPRIORITY8   GICD_IPRIORITY(8)
#define GICD_IPRIORITY9   GICD_IPRIORITY(9)
#define GICD_IPRIORITY10  GICD_IPRIORITY(10)
#define GICD_IPRIORITY11  GICD_IPRIORITY(11)
#define GICD_IPRIORITY12  GICD_IPRIORITY(12)
#define GICD_IPRIORITY13  GICD_IPRIORITY(13)
#define GICD_IPRIORITY14  GICD_IPRIORITY(14)
#define GICD_IPRIORITY15  GICD_IPRIORITY(15)
#define GICD_IPRIORITY16  GICD_IPRIORITY(16)
#define GICD_IPRIORITY17  GICD_IPRIORITY(17)
#define GICD_IPRIORITY18  GICD_IPRIORITY(18)
#define GICD_IPRIORITY19  GICD_IPRIORITY(19)
#define GICD_IPRIORITY20  GICD_IPRIORITY(20)
#define GICD_IPRIORITY21  GICD_IPRIORITY(21)
#define GICD_IPRIORITY22  GICD_IPRIORITY(22)
#define GICD_IPRIORITY23  GICD_IPRIORITY(23)
//
#define GICD_ICFGR(x)     (*((uint32_t*)&(GIC_DIS->CONFIG) + (x) - 2))
#define GICD_ICFGR0	      GIC_DIS->ICFGR0
#define GICD_ICFGR2	      GICD_ICFGR(2)
#define GICD_ICFGR3	      GICD_ICFGR(3)
#define GICD_ICFGR4	      GICD_ICFGR(4)
#define GICD_ICFGR5	      GICD_ICFGR(5)
//
#define GICD_IRAWST1      GIC_DIS->IRAWST1
#define GICD_IRAWST2      GIC_DIS->IRAWST2
#define GICD_SGIR         GIC_DIS->SGIR
//
//  GIC_CPU
#define GICC_CTLR         GIC_CPU->CTRL
#define GICC_PMR          GIC_CPU->PMR 
#define GICC_BPR          GIC_CPU->BPR
#define GICC_IAR          GIC_CPU->IAR
#define GICC_EOIR         GIC_CPU->EOIR 
#define GICC_RPR          GIC_CPU->RPR
#define GICC_HPPIR        GIC_CPU->HPPIR
#define GICC_ABPR         GIC_CPU->ABPR
//
/** @defgroup ***
  * @{
  */
#define SGIR_INTID_Msk          0x00000007  
#define SGIR_NSATT_GROUP_0      0x00000000  
#define SGIR_NSATT_GROUP_1      0x00008000
#define SGIR_CTRL_Msk           0x00010000

/* Initialization and Configuration functions *********************************/

/* GPIO Read and Write functions **********************************************/
extern void init_dit0_gic(void);
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __1888BC048_GIC_H */
/**
  * @}
  */
  
