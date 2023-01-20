#ifndef __RCM1888BC048_SGMII_H
#define __RCM1888BC048_SGMII_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

enum mgeth_SPEED
{
	SPEED_10   ,
	SPEED_100  ,
	SPEED_1000 ,
};

struct mgeth_conf
{
	bool is_full_duplex;
	enum mgeth_SPEED speed;
};

#define MDIO_ID            0x00
#define MDIO_VER           0x04
#define MDIO_STATUS        0x08
#define MDIO_IRQ_MASK      0x0C
#define MDIO_PHY_IRQ_STATE 0x10
#define MDIO_CONTROL       0x14
#define MDIO_ETH_RST_N     0x18
#define MDIO_FREQ_DIVIDER  0x1C
#define MDIO_EN            0x20

#define MDIO_ID_RESET             0x4F49444D
#define MDIO_VER_RESET            0x00640101
#define MDIO_STATUS_RESET         0x00
#define MDIO_IRQ_MASK_RESET       0x00
#define MDIO_PHY_IRQ_STATE_RESET  0x00
#define MDIO_CONTROL_RESET        0x00
#define MDIO_ETH_RST_N_RESET      0x00
#define MDIO_FREQ_DIVIDER_RESET   0x13
#define MDIO_EN_RESET             0x00

#define ETH_RST_N                 0x00
#define MDC_EN                    0x00
#define PHY_IRQ                   0x00
#define RD_IRQ                    0x01
#define WR_IRQ                    0x02

#define START_WR                  0x00
#define START_RD                  0x01
#define BUSY                      0x02
#define ADDR_PHY                  0x03
#define ADDR_REG                  0x08
#define CTRL_DATA                 0x10 

#define ETH_PHY_ID                0xBBCD

#define AN_EN   				 0

// Common registers
#define MGETH_ID              0x0000
#define MGETH_VERSION         0x0004
#define MGETH_SW_RST          0x0008
#define MGETH_GLOBAL_STATUS   0x000C

// MGETH registers
#define MGETH_STATUS          0x0010
#define MGETH_IRQ_MASK        0x0018
#define MGETH_CONTROL         0x001C
#define MGETH_LEN_MASK_CH0    0x0020
#define MGETH_LEN_MASK_CHd       0x4
#define MGETH_TX0_DELAY_TIMER 0x0030
#define MGETH_TXd_DELAY_TIMER    0x4
#define HD_SGMII_MODE         0x0040

// Receive channel registers (relative begin of channel)
#define RXx_ETH_MASK_VALUE_0 -0x0100
#define RXx_ETH_MASK_VALUE_d     0x4
#define RXx_ETH_MASK_VALUE_c      32
#define RXx_ETH_MASK_ACTIV_0 -0x0080
#define RXx_ETH_MASK_ACTIV_d     0x4
#define RXx_ETH_MASK_ACTIV_c      32

// Receive channels
#define MGETH_RECV_CH_0       0x0300
#define MGETH_RECV_CH_d        0x200

// Transmit channels
#define MGETH_SEND_CH_0       0x0A00
#define MGETH_SEND_CH_d        0x100

// Setup registers of DMA channel (relative begin of channel)
#define ENABLE_x                0x00
#define SETTINGS_x              0x10
#define IRQ_MASK_x              0x14
#define STATUS_x                0x18
#define DESC_ADDR_x             0x20

void mgeth_reset_mac(uint32_t base_addr);
void mgeth_init_mac(const uint32_t base, const struct mgeth_conf *conf);
int mgeth_init_sgmii(uint32_t sgmii_base_addr, uint32_t sctl_base_addr, uint32_t AN_en);
void init_gpio_and_en_eth_phy(uint32_t mgpio_base, uint32_t mdio_base, uint32_t AN_en);
void mdio_write(uint32_t mdio_addr,uint32_t reg_addr,uint32_t write_data);

#ifdef __cplusplus
}
#endif

#endif /* __RCM1888BC048_SGMII_H */