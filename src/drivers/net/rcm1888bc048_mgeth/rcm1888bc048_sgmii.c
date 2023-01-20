#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include "rcm1888bc048_sgmii.h"

#include <kernel/time/time.h>

static __attribute__((no_instrument_function)) inline uint32_t ioread32(uint32_t const base_addr)
{
    return *((volatile uint32_t*)(base_addr));
}

static __attribute__((no_instrument_function)) inline void iowrite32(uint32_t const value, uint32_t const base_addr)
{
    *((volatile uint32_t*)(base_addr)) = value;
}

void mgeth_reset_mac(uint32_t base_addr)
{
	iowrite32(0x1, base_addr + MGETH_SW_RST);

	while (ioread32(base_addr + MGETH_SW_RST) & 0x1);
}

void mgeth_init_mac(const uint32_t base, const struct mgeth_conf *conf)
{
	uint32_t ctrl, ctrl_set = 0x0;

	if (conf->is_full_duplex)
		ctrl_set |= 0x1; // full_duplex (0 bit)

	ctrl_set |= conf->speed << 1; // speed (2 - 1 bits)

	ctrl = ioread32(base + MGETH_CONTROL);

	ctrl &= ~0x7;
	ctrl |= ctrl_set;

	iowrite32(ctrl, base + MGETH_CONTROL);
}

int mgeth_init_sgmii(uint32_t sgmii_base_addr, uint32_t sctl_base_addr, uint32_t AN_en)
{
	uint32_t
		sgmii_ctrl_stat_val,
		OFFSET_SPCS_0, OFFSET_SPCS_1, OFFSET_SPCS_2, OFFSET_SPCS_3,
		OFFSET_TXS_0, OFFSET_TXS_1, OFFSET_TXS_2, OFFSET_TXS_3,
		OFFSET_CM,
		OFFSET_RXS_0, OFFSET_RXS_1, OFFSET_RXS_2, OFFSET_RXS_3;

	sgmii_ctrl_stat_val = ioread32(sctl_base_addr + 0x14);

	if (sgmii_ctrl_stat_val)
	{
		return 1;
	}

	OFFSET_SPCS_0 = sgmii_base_addr + 0x0200;
	OFFSET_SPCS_1 = sgmii_base_addr + 0x0600;
	OFFSET_SPCS_2 = sgmii_base_addr + 0x0A00;
	OFFSET_SPCS_3 = sgmii_base_addr + 0x0E00;
	OFFSET_TXS_0  = sgmii_base_addr + 0x0000;
	OFFSET_TXS_1  = sgmii_base_addr + 0x0400;
	OFFSET_TXS_2  = sgmii_base_addr + 0x0800;
	OFFSET_TXS_3  = sgmii_base_addr + 0x0C00;
	OFFSET_CM     = sgmii_base_addr + 0x1000;
	OFFSET_RXS_0  = sgmii_base_addr + 0x0100;
	OFFSET_RXS_1  = sgmii_base_addr + 0x0500;
	OFFSET_RXS_2  = sgmii_base_addr + 0x0900;
	OFFSET_RXS_3  = sgmii_base_addr + 0x0D00;
        
    if (!AN_en) {
          iowrite32(0x00000140, OFFSET_SPCS_0 + 0x00);
        iowrite32(0x00000140, OFFSET_SPCS_1 + 0x00);
        iowrite32(0x00000140, OFFSET_SPCS_2 + 0x00);
        iowrite32(0x00000140, OFFSET_SPCS_3 + 0x00);
    }  

	iowrite32(0x40803004, OFFSET_TXS_0 + 0x00);
	iowrite32(0x40803004, OFFSET_TXS_1 + 0x00);
	iowrite32(0x40803004, OFFSET_TXS_2 + 0x00);
	iowrite32(0x40803004, OFFSET_TXS_3 + 0x00);

	iowrite32(0x00130000, OFFSET_CM + 0x04);
	iowrite32(0x710001F0, OFFSET_CM + 0x08);
	iowrite32(0x00000002, OFFSET_CM + 0x0C);
	iowrite32(0x07000000, OFFSET_CM + 0x20);

	iowrite32(0x0000CEA6, OFFSET_RXS_0 + 0x08);
	iowrite32(0x0000CEA6, OFFSET_RXS_1 + 0x08);
	iowrite32(0x0000CEA6, OFFSET_RXS_2 + 0x08);
	iowrite32(0x0000CEA6, OFFSET_RXS_3 + 0x08);

	iowrite32(0x1, sctl_base_addr + 0x14);

	while (sgmii_ctrl_stat_val != 0x000001F1)
	{
		sgmii_ctrl_stat_val = ioread32(sctl_base_addr + 0x14);
	}

	return 0;
}

void mdio_write(uint32_t mdio_addr,uint32_t reg_addr,uint32_t write_data)
{
    uint32_t phy_addr =0x0,
             mdio_control_data,
             read_data = 0x4;
    
    mdio_control_data = 0x00000001 | phy_addr << 4 | reg_addr << 8 | write_data << 16;
	iowrite32(mdio_control_data, mdio_addr + MDIO_CONTROL);
    
	while (read_data & 0x4) {
        read_data = ioread32(mdio_addr + MDIO_CONTROL);
    }
}

void init_gpio_and_en_eth_phy(uint32_t mgpio_base, uint32_t mdio_base, uint32_t AN_en)
{
    uint32_t i;
	
#define GPIO_SWITCH_SOURCE 0x024
	
    iowrite32(0x00000000, mgpio_base          + GPIO_SWITCH_SOURCE);
    iowrite32(0x00000000, mgpio_base + 0x1000 + GPIO_SWITCH_SOURCE); 
    usleep(1000000);
    for (i = 0; i < 4; i++){
        iowrite32(0x1, mdio_base + 0x1000*i + MDIO_ETH_RST_N);
        iowrite32(0x1, mdio_base + 0x1000*i + MDIO_EN);
        usleep(100000);
    }
    usleep(1000000);
    
    for (i = 0; i < 4; i++){
        if (!AN_en) {
            mdio_write (mdio_base + 0x1000*i, 0x14, 0x2947);
        }
    }
}
