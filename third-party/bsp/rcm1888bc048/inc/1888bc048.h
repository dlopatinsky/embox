/**
  ******************************************************************************
  * @file    1888bc048.h
  * @author  A. Maksimov, V. Syrtsov 
  * @version V1.0.1
  * @date    12.12.2019
  * @brief        4.1. 
  ******************************************************************************
  */
  
#ifndef __1888BC048_H
#define __1888BC048_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define ISB	asm volatile ("isb sy" : : : "memory")
#define DSB	asm volatile ("dsb sy" : : : "memory")
#define DMB	asm volatile ("dmb sy" : : : "memory")

#define isb()	ISB
#define dsb()	DSB
#define dmb()   DMB

#define MEM_ALIGN_SIZE(__size, __align)         ((uint32_t)((__size) + ( ((__size) % (__align)) ? ((__align) - ((__size) % (__align))) : 0 )))

/*!<   */
#define     __I     volatile const            /*!< defines 'read only' permissions      */
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus;

#define IS_BIT_STATUS(STATUS)	(((STATUS) == RESET) || ((STATE) == SET))

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/*!< Peripheral memory map */
/**
  * @{
  */

#define APB_DBGPERIPH_BASE  0x01000000
#define APBF0PERIPH_BASE    0x01010000
#define APBS0PERIPH_BASE    0x01020000
#define APBS1PERIPH_BASE    0x01040000
#define APBS2PERIPH_BASE    0x01080000
#define AXI32PERIPH_BASE    0x01100000
// APB_DBG
#define DAP_ROM_BASE        (APB_DBGPERIPH_BASE + 0x0000)
#define TMC_ETF_BASE        (APB_DBGPERIPH_BASE + 0x1000)
#define TMC_ETR_BASE        (APB_DBGPERIPH_BASE + 0x2000)
#define CTI1_BASE           (APB_DBGPERIPH_BASE + 0x3000)
#define ATB_REPL_BASE       (APB_DBGPERIPH_BASE + 0x4000)
#define TPIU_BASE           (APB_DBGPERIPH_BASE + 0x5000)
#define TS_GEN_BASE         (APB_DBGPERIPH_BASE + 0x6000)
//--
#define APBIC_ROM0_BASE     (APB_DBGPERIPH_BASE + 0x8000)
#define ETM0_BASE           (APB_DBGPERIPH_BASE + 0x9000)
#define DBG_REG0_BASE       (APB_DBGPERIPH_BASE + 0xA000)
#define PERF_MON_BASE       (APB_DBGPERIPH_BASE + 0xB000)
#define CTI0_BASE           (APB_DBGPERIPH_BASE + 0xC000)
//---
// APBF0
#define MDMA_GP0_BASE       (APBF0PERIPH_BASE + 0x0000)
#define MDMA_GP1_BASE       (APBF0PERIPH_BASE + 0x1000)
#define MDMA_GP2_BASE       (APBF0PERIPH_BASE + 0x2000)
#define MDMA_GP3_BASE       (APBF0PERIPH_BASE + 0x3000)
#define MGETH0_BASE         (APBF0PERIPH_BASE + 0x4000)
#define MGETH1_BASE         (APBF0PERIPH_BASE + 0x5000)
#define MGETH2_BASE         (APBF0PERIPH_BASE + 0x6000)
#define MGETH3_BASE         (APBF0PERIPH_BASE + 0x7000)
#define EXT_IRQ_GEN_BASE    (APBF0PERIPH_BASE + 0x8000)
#define ADDR_TRANS_SLV_BASE (APBF0PERIPH_BASE + 0x9000)
#define ADDR_TRANS_MST_BASE (APBF0PERIPH_BASE + 0xA000)
//--
// APBS0
#define CAN0_BASE           (APBS0PERIPH_BASE + 0x0000)
#define CAN1_BASE           (APBS0PERIPH_BASE + 0x8000) 
#define ARINC429_BASE       (APBS0PERIPH_BASE + 0x10000)
#define TVSENS_BASE         (APBS0PERIPH_BASE + 0x12000)
#define DDR0_BIST_CTRL_BASE (APBS0PERIPH_BASE + 0x13000)
#define DDR1_BIST_CTRL_BASE (APBS0PERIPH_BASE + 0x14000) 
//--
#define BISR_L2C_BASE       (APBS0PERIPH_BASE + 0x1E000) 
// APBS1
#define GSPI0_BASE          (APBS1PERIPH_BASE + 0x0000)
#define GSPI1_BASE          (APBS1PERIPH_BASE + 0x1000)
#define SDIO0_BASE          (APBS1PERIPH_BASE + 0x2000)
#define SDIO1_BASE          (APBS1PERIPH_BASE + 0x3000)
#define MUART0_BASE         (APBS1PERIPH_BASE + 0x4000)
#define MUART1_BASE         (APBS1PERIPH_BASE + 0x5000)
#define I2C0_BASE           (APBS1PERIPH_BASE + 0x6000)
#define I2C1_BASE           (APBS1PERIPH_BASE + 0x7000)
#define DIT0_BASE           (APBS1PERIPH_BASE + 0x8000)
#define DIT1_BASE           (APBS1PERIPH_BASE + 0x9000)
#define DIT2_BASE           (APBS1PERIPH_BASE + 0xA000)
#define DIT3_BASE           (APBS1PERIPH_BASE + 0xB000)
#define CRG_SYS_BASE        (APBS1PERIPH_BASE + 0xC000)
#define CRG_DDR_BASE        (APBS1PERIPH_BASE + 0xD000)
#define PCI_CONT_BASE       (APBS1PERIPH_BASE + 0x18000)
#define PCI_PHY_BASE        (APBS1PERIPH_BASE + 0x20000)
// APBS2
#define MDIO0_BASE          (APBS2PERIPH_BASE + 0x0000)
#define MDIO1_BASE          (APBS2PERIPH_BASE + 0x1000)
#define MDIO2_BASE          (APBS2PERIPH_BASE + 0x2000)
#define MDIO3_BASE          (APBS2PERIPH_BASE + 0x3000)
#define MGPIO0_BASE         (APBS2PERIPH_BASE + 0x4000)
#define MGPIO1_BASE         (APBS2PERIPH_BASE + 0x5000)
#define SGMII_PHY_BASE      (APBS2PERIPH_BASE + 0x6000)
#define GPIO0_BASE          (APBS2PERIPH_BASE + 0x8000)
#define GPIO1_BASE          (APBS2PERIPH_BASE + 0x9000)
#define GPIO2_BASE          (APBS2PERIPH_BASE + 0xA000)
#define GPIO3_BASE          (APBS2PERIPH_BASE + 0xB000)
#define GPIO4_BASE          (APBS2PERIPH_BASE + 0xC000)
#define SCTL_BASE           (APBS2PERIPH_BASE + 0xD000)
#define GTIMERS_BASE        (APBS2PERIPH_BASE + 0xE000)
#define WDT_BASE            (APBS2PERIPH_BASE + 0xF000)
#define MKIO0_BASE          (APBS2PERIPH_BASE + 0x10000)
#define MKIO1_BASE          (APBS2PERIPH_BASE + 0x11000)
// AXI32
#define DDR0_BASE           (AXI32PERIPH_BASE + 0x0000)
#define DDR1_BASE           (AXI32PERIPH_BASE + 0x2000)
#define GIC_DIS_BASE        (AXI32PERIPH_BASE + 0x4000)
#define GIC_CPU_BASE        (AXI32PERIPH_BASE + 0x5000)
#define PL_L2C_BASE         (AXI32PERIPH_BASE + 0x6000)
#define NIC_GPV_BASE        (AXI32PERIPH_BASE + 0x10000)
  
/**
  * @}
  */
  
/*!<     */
typedef enum IRQn{
//  SGI -  
  SGI0_IRQn           = 0,  /*!<   0  GIC390 */
  SGI1_IRQn           = 1,  /*!<   1  GIC390 */
//  SPI  -    
  DDC_RX_IRQn         = 32,  /*!<   (DCC) RX */
  DDC_TX_IRQn         = 33,  /*!<   (DCC) TX */
  TERMO_IRQn          = 35,  /*!<     */
  MDIO0_IRQn          = 36,  /*!<  0  MDIO */
  MDIO1_IRQn          = 37,  /*!<  1  MDIO */
  MDIO2_IRQn          = 38,  /*!<  2  MDIO */
  MDIO3_IRQn          = 39,  /*!<  3  MDIO */
  MDIO_GPIO0_IRQn     = 40,  /*!<  0 GPIO  MDIO */
  MDIO_GPIO1_IRQn     = 41,  /*!<  1 GPIO  MDIO */
  DIT0_TIM1_IRQn      = 42,  /*!<    DIT0    1 */
  DIT0_TIM2_IRQn      = 43,  /*!<    DIT0    2 */
  DIT1_TIM1_IRQn      = 44,  /*!<    DIT1    1 */
  DIT1_TIM2_IRQn      = 45,  /*!<    DIT1    2 */
  DIT2_TIM1_IRQn      = 46,  /*!<    DIT2    1 */
  DIT2_TIM2_IRQn      = 47,  /*!<    DIT2    2 */
  DIT3_TIM1_IRQn      = 48,  /*!<    DIT3    1 */
  DIT3_TIM2_IRQn      = 49,  /*!<    DIT3    2 */
  MGETH0_IRQn         = 51,  /*!<    MGETH0 */
  MGETH1_IRQn         = 52,  /*!<    MGETH1 */
  MGETH2_IRQn         = 53,  /*!<    MGETH2 */
  MGETH3_IRQn         = 54,  /*!<    MGETH3 */
  PCIE_DMA_IRQn       = 55,  /*!< DMA   PCIe */
  PCIE_Local_IRQn     = 56,  /*!< Local error   PCIe */
  PCIEa_IRQn          = 57,  /*!< PCI legacy interrupt a  PCIe */
  PCIEb_IRQn          = 58,  /*!< PCI legacy interrupt b  PCIe */
  PCIEc_IRQn          = 59,  /*!< PCI legacy interrupt c  PCIe */
  PCIEd_IRQn          = 60,  /*!< PCI legacy interrupt d  PCIe */
  PCIE_Slv_IRQn       = 61,  /*!<      AXI   PCIe */
  PCIE_Mst_IRQn       = 62,  /*!<      AXI   PCIe */
  WDT_IRQn            = 63,  /*!<     */
  MDMA0_IRQn          = 64,  /*!<    MDMA0 */
  MDMA1_IRQn          = 65,  /*!<    MDMA1 */
  MDMA2_IRQn          = 66,  /*!<    MDMA2 */
  MDMA3_IRQn          = 67,  /*!<    MDMA3 */
  MKIO0_IRQn          = 68,  /*!<    MKIO0 */
  MKIO1_IRQn          = 69,  /*!<    MKIO1 */
  MKIO2_IRQn          = 70,  /*!<    MKIO2 */
  MKIO3_IRQn          = 71,  /*!<    MKIO3 */
  SDIO0_IRQn          = 72,  /*!<    GSPI_SDIO0 */
  SDIO1_IRQn          = 73,  /*!<    GSPI_SDIO1 */
  I2C0_IRQn           = 74,  /*!<    I2C0 */
  I2C1_IRQn           = 75,  /*!<    I2C1 */
  UART0_IRQn          = 76,  /*!<     UART0 */
  UART1_IRQn          = 77,  /*!<     UART1 */
  GPIO0_IRQn          = 78,  /*!<     GPIO0 */
  GPIO1_IRQn          = 79,  /*!<     GPIO1 */
  GPIO2_IRQn          = 80,  /*!<     GPIO2 */
  GPIO3_IRQn          = 81,  /*!<     GPIO3 */
  GPIO4_IRQn          = 82,  /*!<     GPIO4 */
  CAN0_IRQn           = 83,  /*!<  CAN0 */
  CAN1_IRQn           = 84,  /*!<  CAN1 */
  ARINC_IRQn          = 85,  /*!<   ARINC */
  SGMII_IRQn          = 86,  /*!<   SGMII */
  DDR0_IRQn           = 87,  /*!<   DDR0 */
  DDR1_IRQn           = 88,  /*!<   DDR1 */
  GTIM_IRQn           = 89,  /*!<   gp_global_timers */
  CRG_SYS_IRQn        = 90,  /*!<    PLL  crg_sys */
  CRG_DDR_IRQn        = 91,  /*!<    PLL  crg_ddr */
  PMU_IRQn            = 92,  /*!<   PMU */
  L2C_IRQn            = 93,  /*!<   L2C */
  L2C_SlvErr_IRQn     = 94,  /*!<  Slave Error L2C */
  L2C_DecErr_IRQn     = 95,  /*!<  Decode Error L2C */
} IRQn_Type;

/**
  * @}
  */
static __attribute__((no_instrument_function)) inline uint32_t __get_DFAR(void)
{
  register uint32_t result; 
  asm volatile ("MRC p15, 0, %0, c6, c0, 0\n" : "=r" (result) ); 
  return(result); 
}

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
 }
#endif
  
#endif /* __1888BC048_H */

