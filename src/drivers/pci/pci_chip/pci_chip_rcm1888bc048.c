//--------------------------------------------------------------
// @author  V. Syrtsov
//--------------------------------------------------------------

#include <stdint.h>
#include <hal/reg.h>
#include <util/log.h>

#include <drivers/pci/pci_chip/pci_utils.h>
#include <drivers/pci/pci.h>

#include <embox/unit.h>

#include <1888bc048_pci.h>
#include <1888bc048_tim.h>

#define TRAINING_COMPLETE_TIMEOUT     1000

static bool __pci_func_list[8] = {
  false, false, false, false, false, false, false, false
};

uint32_t pci_read_config8(uint32_t bus, uint32_t dev_fn, uint32_t where, uint8_t *value) 
{
  uint32_t busdev = PCIE_ADDR(bus, dev_fn >> 3, dev_fn & 0x7, where);
  if ( !(__pci_func_list[dev_fn & 0x7]) ) {
    *value = 0xFF;
    return PCIUTILS_SUCCESS;
  }
	log_debug("%p", (PCIE_WINDOW + busdev));
	*value = *((__IO uint8_t*)(PCIE_WINDOW + busdev));
	return PCIUTILS_SUCCESS;
}

uint32_t pci_read_config16(uint32_t bus, uint32_t dev_fn, uint32_t where, uint16_t *value)
{
  uint32_t busdev = PCIE_ADDR(bus, dev_fn >> 3, dev_fn & 0x7, where);
  if ( !(__pci_func_list[dev_fn & 0x7]) ) {
    *value = 0xFFFF;
    return PCIUTILS_SUCCESS;
  }
	log_debug("%p", (PCIE_WINDOW + busdev));
	*value = *((__IO uint16_t*)(PCIE_WINDOW + busdev));
	return PCIUTILS_SUCCESS; 
}

uint32_t pci_read_config32(uint32_t bus, uint32_t dev_fn,	uint32_t where, uint32_t *value) 
{
  uint32_t busdev = PCIE_ADDR(bus, dev_fn >> 3, dev_fn & 0x7, where);
  if ( !(__pci_func_list[dev_fn & 0x7]) ) {
    *value = 0xFFFFFFFF;
    return PCIUTILS_SUCCESS;
  }
	log_debug("%p", (PCIE_WINDOW + busdev));
	*value = *((__IO uint32_t*)(PCIE_WINDOW + busdev)); 
	return PCIUTILS_SUCCESS; 
}

uint32_t pci_write_config8(uint32_t bus, uint32_t dev_fn, uint32_t where, uint8_t value) 
{
	uint32_t busdev = PCIE_ADDR(bus, dev_fn >> 3, dev_fn & 0x7, where);
  if ( !(__pci_func_list[dev_fn & 0x7]) ) {
    return PCIUTILS_SUCCESS;
  }
	log_debug("%p", (PCIE_WINDOW + busdev));
	*((__IO uint8_t*)(PCIE_WINDOW + busdev)) = value;
	return PCIUTILS_SUCCESS;
}

uint32_t pci_write_config16(uint32_t bus, uint32_t dev_fn, uint32_t where, uint16_t value) 
{
	uint32_t busdev = PCIE_ADDR(bus, dev_fn >> 3, dev_fn & 0x7, where);
  if ( !(__pci_func_list[dev_fn & 0x7]) ) {
    return PCIUTILS_SUCCESS;
  }
	log_debug("%p", (PCIE_WINDOW + busdev));
	*((__IO uint16_t*)(PCIE_WINDOW + busdev)) = value;
	return PCIUTILS_SUCCESS;
}

uint32_t pci_write_config32(uint32_t bus, uint32_t dev_fn, uint32_t where,	uint32_t value) 
{
	uint32_t busdev = PCIE_ADDR(bus, dev_fn >> 3, dev_fn & 0x7, where);
  if ( !(__pci_func_list[dev_fn & 0x7]) ) {
    return PCIUTILS_SUCCESS;
  }
	log_debug("%p", (PCIE_WINDOW + busdev));
	*((__IO uint32_t*)(PCIE_WINDOW + busdev)) = value;
	return PCIUTILS_SUCCESS;
}

static void PCIE_RC_OutboundRegion_Init(int region_no, int type, uint8_t num_pass_bits, uint32_t addr)
{
  PCIE->AXI.OutboundRegion[region_no].AddrTrans0 = (addr | num_pass_bits);
  PCIE->AXI.OutboundRegion[region_no].AddrTrans1 = 0;

  PCIE->AXI.OutboundRegion[region_no].BaseAddr0 = (addr | num_pass_bits); 
  PCIE->AXI.OutboundRegion[region_no].BaseAddr1 = 0;

  PCIE->AXI.OutboundRegion[region_no].HeaderDesc0 &= ~(AXI_OBR_DESC0_TYPE_Msk);
  PCIE->AXI.OutboundRegion[region_no].HeaderDesc0 |= (type | AXI_OBR_DESC0_HARDCODED_RID);
  PCIE->AXI.OutboundRegion[region_no].HeaderDesc1 = 0;
}

static void PCIE_RC_InboundRegion_Init(int region_no, uint8_t num_pass_bits, uint32_t addr)
{
  PCIE->AXI.InboundRegion[region_no].AddrTrans0 = (addr | num_pass_bits);
  PCIE->AXI.InboundRegion[region_no].AddrTrans1 = 0;
}

static uint8_t __log2(uint32_t x)
{
  uint8_t ret;

  for ( ret = 31; ret != 0; ret-- ) {
    if ( (1 << ret) & x ) {
      break;
    }
  }
  return ret;
}

static uint32_t* PCIE_RC_Alloc(uint8_t bus, uint8_t dev, uint8_t func, uint8_t bar_num, uint32_t *addr)
{
  uint32_t busdev = PCIE_ADDR(bus, dev, func, (PCI_BASE_ADDR_REG_0 + bar_num * 4));
  uint32_t val;
  uint32_t mask;
  uint8_t num_bits;

  *((__IO uint32_t*)(PCIE_WINDOW + busdev)) = 0xFFFFFFFF;
  val = *((__IO uint32_t*)(PCIE_WINDOW + busdev));

  if ( val ) {
    // IO
    if ( val & 0x1 ) {
      mask = val & 0x1;
      val &= 0xFFFFFF00;
      val = ~val + 1;
      num_bits = __log2(val);
      if ( ((uint32_t)addr) % val ) {
        addr = (uint32_t *)((uint32_t)addr + val - (uint32_t)(((uint32_t)addr) % val));
      }
      // Configuration AXI OutboundRegion for IO_WRITE
      PCIE_RC_OutboundRegion_Init((func * 8 + bar_num + 1), AXI_OBR_DESC0_TYPE_IO_WRITE, (num_bits - 1), (uint32_t)addr);
      log_debug("Function %d BAR%d = %p size = 0x%08X mask = %d type = IO", func, bar_num, addr, val, num_bits);
    } else { // MEM
      mask = val & 0xF;
      val &= 0xFFFFFF00;
      val = ~val + 1;
      num_bits = __log2(val);
      if ( ((uint32_t)addr) % val ) {
        addr = (uint32_t *)((uint32_t)addr + val - (uint32_t)(((uint32_t)addr) % val));
      }
      // Configuration AXI OutboundRegion for MEM_WRITE
      PCIE_RC_OutboundRegion_Init((func * 8 + bar_num + 1), AXI_OBR_DESC0_TYPE_MEM_WRITE, (num_bits - 1), (uint32_t)addr);
      log_debug("Function %d BAR%d = %p size = 0x%08X mask = %d type = MEM", func, bar_num, addr, val, num_bits);
    }
    // Set BARx for EP
    *((__IO uint32_t*)(PCIE_WINDOW + busdev)) = (uint32_t)addr + mask;
  }
  return (uint32_t *)((uint32_t)addr + val);
}

/* check whether correct pci device in the slot bus */
uint32_t PCIE_RC_Check(uint8_t bus, uint8_t dev, uint8_t func) 
{
  uint32_t vendor = 0;
  uint32_t busdev = PCIE_ADDR(bus, dev, func, PCI_VENDOR_ID);
  vendor = *((__IO uint32_t*)(PCIE_WINDOW + busdev));
  if ( (PCIE_WINDOW + busdev) == __get_DFAR() ) {
    return (uint32_t) -1;
  }
  log_debug("Device ID = 0x%04X Vendor ID = 0x%04X\r\n", (uint16_t)(vendor >> 16), (uint16_t)vendor);
  return vendor;
}

static uint32_t HAL_GetTick(void)
{
	uint64_t tick;
	
	GTIM->FIX_CMD = 1;
	tick = ((uint64_t)GTIM->FREE_RUN_H << 32) | GTIM->FREE_RUN_L;
	tick /= 100000;
	return (uint32_t)tick;
}

static uint32_t* PCIE_RC_Init(void)
{
  uint32_t *ret = NULL;
  uint32_t *pBAR = (uint32_t *)(PCIE_WINDOW | 0x10000000);
  uint32_t busdev;
  uint8_t BARi;
  uint8_t FUNCi;
  uint8_t Header_Type;
  uint32_t tickstart;

  // PCIe Software Reset On
  PCIE_PHY->Reset = RESET;
  // Disable Link Training
  PCIE_PHY->BaseSettings.link_training_enable_reg = RESET;
  // Set PCIe Mode 
  PCIE_PHY->BaseSettings.mode_select = PCIE_RootComplex;
  // PCIe Software Reset Off
  PCIE_PHY->Reset = SET;
  // Wait until PLL locked
  while ( PCIE_PHY->Reset != SET );
  // Set PCI CLASS BRIDGE
  *((__IO uint16_t*)(((uint32_t)&PCIE->RootPort.RevisionClass) + sizeof(uint16_t) + PCIE_RC_APB_ACCESS)) = PCI_CLASS_BRIDGE;
  // Enable Link Training
  PCIE_PHY->BaseSettings.link_training_enable_reg = SET;

  tickstart = HAL_GetTick();
  // Wait until training complete
  while ( (!(PCIE->LocMgmt.PhyConf0 & LINK_UP_Msk)) && ((HAL_GetTick() - tickstart) < TRAINING_COMPLETE_TIMEOUT) );

  if ( (HAL_GetTick() - tickstart) >= TRAINING_COMPLETE_TIMEOUT ) {
    return ret;
  }

  // Check that the link is up
  if ( PCIE->LocMgmt.PhyConf0 & LINK_UP_Msk ) {
    log_debug("PCIe link up");
    // Clear AXI link-down status
    PCIE->AXI.Axi10 = RESET;

    switch ( PCIE->LocMgmt.PhyConf0 & NEGOTIATED_LANE_COUNT_Msk ) {
      case LINK_WIDTH_X1:
        log_debug("link width:  x1");
        break;
      case LINK_WIDTH_X2:
        log_debug("link width:  x2");
        break;
      case LINK_WIDTH_X4:
        log_debug("link width:  x4");
        break;
      case LINK_WIDTH_X8:
        log_debug("link width:  x8");
        break;
    }

    switch ( PCIE->LocMgmt.PhyConf0 & NEGOTIATED_SPEED_Msk ) {
      case NEGOTIATED_SPEED_2_5G:
        log_debug("link speed:  2.5GTs");
        break;
      case NEGOTIATED_SPEED_5G:
        log_debug("link speed:  5.0GTs");
        break;
    }

    // Set Bus Master
    PCIE->RootPort.Command.IO_SpaceEnable = SET;
    PCIE->RootPort.Command.MEM_SpaceEnable = SET;
    PCIE->RootPort.Command.BusMasterEnable = SET;

    PCIE->RootPort.DevCtrlStatus.MaxPayloadSize = MAX_PAYLOAD_256B;

    PCIE->LocMgmt.BarConf = BAR_CFG_CTRL_MEM | BAR_APERTURE_4GB;

    // Configuration AXI OutboundRegion for TYPE0_CFG
    PCIE_RC_OutboundRegion_Init(0, AXI_OBR_DESC0_TYPE_CFG_TYPE0, (27 - 1), PCIE_WINDOW);
    // Configuration AXI InboundRegion for BAR0 needed for MSI
    PCIE_RC_InboundRegion_Init(0, (32 - 1), 0);

    // For each function
    for ( FUNCi = 0; FUNCi < 8; FUNCi++ ) {    
      if ( PCIE_RC_Check(0, 0, FUNCi) == -1 ) {
        continue;
      }
      __pci_func_list[FUNCi] = true;
      // Configuration EP BAR[0-5]
      for ( BARi = 0; BARi < 6; BARi++ ) {
        pBAR = PCIE_RC_Alloc(0, 0, FUNCi, BARi, pBAR);
      }
      busdev = PCIE_ADDR(0, 0, FUNCi, PCI_HEADER_TYPE);
      Header_Type = *((__IO uint8_t*)(PCIE_WINDOW + busdev));
      if ( !(Header_Type & (1 << 7)) ) {
        break;
      }
    }
    ret = (uint32_t *)PCIE_WINDOW;
  }
  return ret;
}

EMBOX_UNIT_INIT(rcm_pci_init);

static int rcm_pci_init(void) {
	PCIE_RC_Init();
	return 0;
}

/* NOTE: this may be inaccurate! */
unsigned int pci_irq_number(struct pci_slot_dev *dev) {
	return (unsigned int) dev->irq_line;
}
