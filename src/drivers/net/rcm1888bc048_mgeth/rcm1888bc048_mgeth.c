//--------------------------------------------------------------
// @author  V. Syrtsov
//--------------------------------------------------------------

#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <embox/unit.h>

#include <kernel/irq.h>
#include <hal/reg.h>
#include <net/l0/net_entry.h>

#include <net/l2/ethernet.h>
#include <net/l3/arp.h>
#include <net/netdevice.h>
#include <net/inetdevice.h>
#include <net/skbuff.h>
#include <net/util/show_packet.h>
#include <util/binalign.h>


#include <kernel/printk.h>

#include <1888bc048_tim.h>
#include "rcm1888bc048_sgmii.h"

EMBOX_UNIT_INIT(mgeth_init);

#define MGETH_IRQ   OPTION_GET(NUMBER, irq_num)

#define MGETH_VER 0x01900144

// RXBD_COUNT should be multiply of 4
#define MGETH_RXBD_CNT 256
#define MGETH_TXBD_CNT 1

#define MGETH_BD_POLL_ALIGN 0x1000

// descriptor flags
#define MGETH_BD_OWN 0x80000000
#define MGETH_BD_LINK 0x40000000
#define MGETH_BD_INT  0x20000000
#define MGETH_BD_STOP 0x10000000

// channel settings
#define MGETH_CHAN_DESC_NORMAL 0x00000000
#define MGETH_CHAN_DESC_LONG 0x00000002
#define MGETH_CHAN_DESC_PITCH 0x00000003
#define MGETH_CHAN_ADD_INFO 0x00000010
#define MGETH_CHAN_DESC_GAP_SHIFT 16

#define ETH_ALEN	6	/* Octets in one ethernet addr   */

#define ARCH_DMA_MINALIGN	128

/* CONFIG_SYS_CACHELINE_SIZE is used a lot in drivers */
#define CONFIG_SYS_CACHELINE_SIZE	ARCH_DMA_MINALIGN

// generic flags
#define MGETH_ENABLE 0x1

#define MGETH_MIN_PACKET_LEN  60
#define MGETH_MAX_PACKET_LEN  0x3fff

#define MGETH_RXBUF_SIZE      1540

#define MGETH_TX_TIMEOUT      5

typedef const volatile unsigned int roreg32;
typedef volatile unsigned int rwreg32;
typedef const volatile unsigned long long roreg64;
typedef volatile unsigned long long rwreg64;

#define BUF_SIZE 2048

typedef struct _mgeth_rx_regs {
/************************* RX Channel registers *****************************/
    rwreg32 rx_eth_mask_value[32];          /* 0x000-0x07C - packets mask   */
    rwreg32 rx_eth_mask_activ[32];          /* 0x080-0x0FC - mask activation*/
/*********************** WDMA Channel registers *****************************/
    rwreg32 enable;                         /* 0x100 - enable channel       */
    rwreg32 suspend;                        /* 0x104 - suspend channel      */
    rwreg32 cancel;                         /* 0x108 - cancel channel       */
    roreg32 _skip01;                        /* 0x10C                        */
    rwreg32 settings;                       /* 0x110 - channel settings     */
    rwreg32 irq_mask;                       /* 0x114 - channel irq mask     */
    roreg32 status;                         /* 0x118 - channel status       */
    roreg32 _skip02;                        /* 0x11C                        */
    rwreg32 desc_addr;                      /* 0x120 - first decriptor addr */
    roreg32 _skip03;                        /* 0x124                        */
    roreg32 curr_desc_addr;                 /* 0x128 - current decript addr */
    roreg32 curr_addr;                      /* 0x12C - current trans addr   */
    roreg32 dma_state;                      /* 0x130 - state of DMA         */
    roreg32 _skip04[3];                     /* 0x134 - 0x13C                */
    rwreg32 desc_axlen;                     /* 0x140 - axlen for desc ops   */
    rwreg32 desc_axcache;                   /* 0x144 - axcache for desc ops */
    rwreg32 desc_axprot;                    /* 0x148 - axprot for desc ops  */
    rwreg32 desc_axlock;                    /* 0x14C - axlock for desc ops  */
    roreg32 desc_rresp;                     /* 0x150 - rresp of desc ops    */
    roreg32 desc_raxi_err_addr;             /* 0x154 - addr of axi read err */
    roreg32 desc_bresp;                     /* 0x158 - bresp of desc ops    */
    roreg32 desc_waxi_err_addr;             /* 0x15C - addr of axi write err*/
    rwreg32 desc_permut;                    /* 0x160 - byte swapping scheme */
    roreg32 _skip05[7];                     /* 0x164 - 0x17C                */
    rwreg32 max_trans;                      /* 0x180 - max unfinished trans */
    rwreg32 awlen;                          /* 0x184 - axi awlen            */
    rwreg32 awcache;                        /* 0x188 - axi awcache          */
    rwreg32 awprot;                         /* 0x18C - axi awprot           */
    rwreg32 awlock;                         /* 0x190 - axi awlock           */
    roreg32 bresp;                          /* 0x194 - axi operation bresp  */
    roreg32 waxi_err_addr;                  /* 0x198 - addr of axi write err*/
    roreg32 _skip06;                        /* 0x19C                        */
    roreg32 state;                          /* 0x1A0 - axi state            */
    roreg32 avaliable_space;                /* 0x1A4 - num of free bytes    */
    rwreg32 permutation;                    /* 0x1A8 - byte swapping scheme */
    roreg32 _skip07[5];                     /* 0x1AC - 0x1BC                */
/************************** Statistc counters *******************************/
    roreg32 a_frames_received_ok;           /* 0x1C0                        */
    roreg64 a_octets_received_ok;           /* 0x1C4                        */
    roreg32 if_in_ucast_pkts;               /* 0x1CC                        */
    roreg32 if_in_multicast_pkts;           /* 0x1D0                        */
    roreg32 if_in_broadcast_pkts;           /* 0x1D4                        */
    roreg32 descriptor_short;               /* 0x1D8                        */
    roreg32 rtp_overmuch_line;              /* 0x1DC                        */
    roreg32 _skip08[8];                     /* 0x1E0 - 0x1FC                */
} __attribute__ ((packed)) mgeth_rx_regs;

typedef struct _mgeth_tx_regs {
    rwreg32 enable;                         /* 0x000 - enable channel       */
    rwreg32 suspend;                        /* 0x004 - suspend channel      */
    rwreg32 cancel;                         /* 0x008 - cancel channel       */
    roreg32 _skip01;                        /* 0x00C                        */
    rwreg32 settings;                       /* 0x010 - channel settings     */
    rwreg32 irq_mask;                       /* 0x014 - channel irq mask     */
    roreg32 status;                         /* 0x018 - channel status       */
    roreg32 _skip02;                        /* 0x01C                        */
    rwreg32 desc_addr;                      /* 0x020 - first decriptor addr */
    roreg32 _skip03;                        /* 0x024                        */
    roreg32 curr_desc_addr;                 /* 0x028 - current decript addr */
    roreg32 curr_addr;                      /* 0x02C - current trans addr   */
    roreg32 dma_state;                      /* 0x030 - state of DMA         */
    roreg32 _skip04[3];                     /* 0x034 - 0x13C                */
    rwreg32 desc_axlen;                     /* 0x040 - axlen for desc ops   */
    rwreg32 desc_axcache;                   /* 0x044 - axcache for desc ops */
    rwreg32 desc_axprot;                    /* 0x048 - axprot for desc ops  */
    rwreg32 desc_axlock;                    /* 0x04C - axlock for desc ops  */
    roreg32 desc_rresp;                     /* 0x050 - rresp of desc ops    */
    roreg32 desc_raxi_err_addr;             /* 0x054 - addr of axi read err */
    roreg32 desc_bresp;                     /* 0x058 - bresp of desc ops    */
    roreg32 desc_waxi_err_addr;             /* 0x05C - addr of axi write err*/
    rwreg32 desc_permut;                    /* 0x060 - byte swapping scheme */
    roreg32 _skip05[7];                     /* 0x064 - 0x17C                */
    rwreg32 max_trans;                      /* 0x080 - max unfinished trans */
    rwreg32 arlen;                          /* 0x084 - axi arlen            */
    rwreg32 arcache;                        /* 0x088 - axi arcache          */
    rwreg32 arprot;                         /* 0x08C - axi arprot           */
    rwreg32 arlock;                         /* 0x090 - axi arlock           */
    roreg32 rresp;                          /* 0x094 - axi operation rresp  */
    roreg32 waxi_err_addr;                  /* 0x098 - addr of axi write err*/
    roreg32 _skip06;                        /* 0x09C                        */
    roreg32 state;                          /* 0x0A0 - axi state            */
    roreg32 avaliable_space;                /* 0x0A4 - num of free bytes    */
    rwreg32 permutation;                    /* 0x0A8 - byte swapping scheme */
    roreg32 _skip07[5];                     /* 0x0AC - 0x1BC                */
/************************** Statistc counters *******************************/
    roreg32 a_frames_transmitted_ok;        /* 0x0C0                        */
    roreg64 a_octets_transmitted_ok;        /* 0x0C4                        */
    roreg32 if_out_ucast_pkts;              /* 0x0CC                        */
    roreg32 if_out_multicast_pkts;          /* 0x0D0                        */
    roreg32 if_out_broadcast_pkts;          /* 0x0D4                        */
    roreg32 _skip08[10];                    /* 0x0D8 - 0x0FC                */
} __attribute__ ((packed)) mgeth_tx_regs;

typedef struct _mgeth_regs {
/***************** Common registers for MGETH and MDMA **********************/
    roreg32 id;                             /* 0x000 - device id            */
    roreg32 version;                        /* 0x004 - device version       */
    rwreg32 sw_rst;                         /* 0x008 - program reset        */
    roreg32 global_status;                  /* 0x00C - status               */
/**************************** MGETH registers *******************************/
    roreg32 mg_status;                      /* 0x010 - mgeth status         */
    roreg32 _skip01;                        /* 0x014                        */
    rwreg32 mg_irq_mask;                    /* 0x018 - mgeth irq mask       */
    rwreg32 mg_control;                     /* 0x01C - mgeth control reg    */
    rwreg32 mg_len_mask_ch0;                /* 0x020 - mgeth mask len ch0   */
    rwreg32 mg_len_mask_ch1;                /* 0x024 - mgeth mask len ch1   */
    rwreg32 mg_len_mask_ch2;                /* 0x028 - mgeth mask len ch2   */
    rwreg32 mg_len_mask_ch3;                /* 0x02C - mgeth mask len ch3   */
    rwreg32 tx0_delay_timer;                /* 0x030 - delay timer for tx0  */
    rwreg32 tx1_delay_timer;                /* 0x034 - delay timer for tx1  */
    rwreg32 tx2_delay_timer;                /* 0x038 - delay timer for tx2  */
    rwreg32 tx3_delay_timer;                /* 0x03C - delay timer for tx3  */
    rwreg32 hd_sgmii_mode;                  /* 0x040 - SGMII mode           */
    roreg32 _skip02[47];                    /* 0x044 - 0x0FC                */
/************************** Statistc counters *******************************/
    roreg32 a_frames_received_ok;           /* 0x100                        */
    roreg64 a_octets_received_ok;           /* 0x104                        */
    roreg32 if_in_ucast_pkts;               /* 0x10C                        */
    roreg32 if_in_multicast_pkts;           /* 0x110                        */
    roreg32 if_in_broadcast_pkts;           /* 0x114                        */
    roreg32 a_frame_check_sequence_errors;  /* 0x118                        */
    roreg32 if_in_errors;                   /* 0x11C                        */
    roreg32 ether_stats_drop_events;        /* 0x120                        */
    roreg64 ether_stats_octets;             /* 0x124                        */
    roreg32 ether_stats_pkts;               /* 0x12C                        */
    roreg32 ether_stats_undersize_pkts;     /* 0x130                        */
    roreg32 ether_stats_oversize_pkts;      /* 0x134                        */
    roreg32 ether_stats_64_octets;          /* 0x138                        */
    roreg32 ether_stats_65_127_octets;      /* 0x13C                        */
    roreg32 ether_stats_128_255_octets;     /* 0x140                        */
    roreg32 ether_stats_256_511_octets;     /* 0x144                        */
    roreg32 ether_stats_512_1023_octets;    /* 0x148                        */
    roreg32 ether_stats_1024_1518_octets;   /* 0x14C                        */
    roreg32 ether_stats_1519_10240_octets;  /* 0x150                        */
    roreg32 ether_stats_jabbers;            /* 0x154                        */
    roreg32 ether_stats_fragments;          /* 0x158                        */
    roreg32 _skip03[9];                     /* 0x15C - 0x17C                */
    roreg32 a_frames_transmitted_ok;        /* 0x180                        */
    roreg64 a_octets_transmitted_ok;        /* 0x184                        */
    roreg32 if_out_ucast_pkts;              /* 0x18C                        */
    roreg32 if_out_multicast_pkts;          /* 0x190                        */
    roreg32 if_out_broadcast_pkts;          /* 0x194                        */
    roreg32 _skip04[26];                    /* 0x198 - 0x1FC                */
/****************************** RX channels *********************************/
    mgeth_rx_regs rx[4];                    /* 0x200 - 0x9FC                */
/****************************** TX channels *********************************/
    mgeth_tx_regs tx[4];                    /* 0xA00 - 0xDFC                */
/******************************* Reserved ***********************************/
    roreg32 _skip05[128];                   /* 0xE00 - 0xFFC                */
}   __attribute__ ((packed)) mgeth_regs;


struct __long_desc {
	uint32_t usrdata_l;
	uint32_t usrdata_h;
	uint32_t memptr;
	uint32_t flags_length;
} __attribute__ ((packed, aligned(16)));

typedef struct {
	mgeth_regs *regs;

	struct mdma_chan *tx_chan;
	struct mdma_chan *rx_chan;

	struct __long_desc *rxbd_base;
	struct __long_desc *txbd_base;
        
        int rxbd_no;

        unsigned int speed;
	unsigned int duplex;
	unsigned int link;

	unsigned char *buffer;

	unsigned char dev_addr[ETH_ALEN];
} mgeth_priv;

#define CTRL_FD_S 0
#define CTRL_FD_M 0x00000001
#define CTRL_SPEED_S 1
#define CTRL_SPEED_M 0x00000006

typedef struct{
  uint32_t length;
  uint32_t buffer;
}FrameTypeDef;

mgeth_priv mgeth;

uint8_t rxbd[MGETH_RXBD_CNT * sizeof(struct __long_desc)] __attribute__ ((aligned (16))); // Intermediate buffer
uint8_t txbd[MGETH_TXBD_CNT * sizeof(struct __long_desc)] __attribute__ ((aligned (16))); // Intermediate buffer

uint8_t EthFrameRX[MGETH_RXBD_CNT - 1][MGETH_RXBUF_SIZE] __attribute__ ((aligned (8))); // Intermediate buffer
uint8_t EthFrameTX[MGETH_TXBD_CNT][MGETH_RXBUF_SIZE] __attribute__ ((aligned (8))); // Intermediate buffer

static int mgeth_xmit(struct net_device *dev, struct sk_buff *skb);
static int mgeth_open(struct net_device *dev);
static int mgeth_close(struct net_device *dev);
static int mgeth_set_mac(struct net_device *dev, const void *addr);

static void init_descr(mgeth_priv *priv);
static void mgeth_set_packet_filter(mgeth_priv *priv);
static int mgeth_start(mgeth_priv *priv, uint8_t *mac);
static void mgeth_stop(mgeth_priv *priv);
static int mgeth_send(mgeth_priv *priv, void *eth_data, int data_length);
static FrameTypeDef mgeth_recv(mgeth_priv *priv, uint32_t descr);
static int mgeth_free_pkt(mgeth_priv *priv, uint32_t descr);

static const struct net_driver mgeth_ops = {
		.xmit = mgeth_xmit,
		.start = mgeth_open,
		.stop = mgeth_close,
		.set_macaddr = mgeth_set_mac,
};


static uint32_t HAL_GetTick(void)
{
	uint64_t tick;
	
	GTIM->FIX_CMD = 1;
	tick = ((uint64_t)GTIM->FREE_RUN_H << 32) | GTIM->FREE_RUN_L;
	tick /= 100000;
	return (uint32_t)tick;
}

static void init_descr(mgeth_priv *priv)
{
  int i;

  /* initate rx decriptors */
  for ( i = 0; i < MGETH_RXBD_CNT; i++ ) {
    priv->rxbd_base[i].usrdata_h = 0;
    priv->rxbd_base[i].usrdata_l = 0;
    /* enable desciptor & set wrap last to first descriptor */
    if ( i >= (MGETH_RXBD_CNT - 1) ) {
      priv->rxbd_base[i].flags_length = MGETH_BD_INT | MGETH_BD_LINK; // link descriptor
      priv->rxbd_base[i].memptr = (unsigned int)&priv->rxbd_base[0];
    } else {
      priv->rxbd_base[i].flags_length =	MGETH_BD_INT | MGETH_RXBUF_SIZE; // usual descriptor
      priv->rxbd_base[i].memptr = (unsigned int)(&EthFrameRX[i][0]);	
    }
  }

  /* initiate indexes */
  priv->rxbd_no = 0;

  /* initate tx decriptor */
  priv->txbd_base[0].usrdata_h = 0;
  priv->txbd_base[0].usrdata_l = 0;
  priv->txbd_base[0].memptr = 0;
  priv->txbd_base[0].flags_length = MGETH_BD_STOP;

  /* Set pointer to rx descriptor areas */
  *((volatile uint32_t*)&priv->regs->rx[0].settings) = MGETH_CHAN_DESC_LONG | MGETH_CHAN_ADD_INFO | (sizeof(struct __long_desc) << MGETH_CHAN_DESC_GAP_SHIFT);
  *((volatile uint32_t*)&priv->regs->rx[0].desc_addr) = (uint32_t)&priv->rxbd_base[0];
}

static void mgeth_set_packet_filter(mgeth_priv *priv)
{
  *((volatile uint32_t*)&priv->regs->rx[0].rx_eth_mask_value[0]) = priv->dev_addr[0] | priv->dev_addr[1] << 8 | priv->dev_addr[2] << 16 | priv->dev_addr[3] << 24;
  *((volatile uint32_t*)&priv->regs->rx[0].rx_eth_mask_value[1]) = priv->dev_addr[4] | priv->dev_addr[5] << 8;
  *((volatile uint32_t*)&priv->regs->rx[0].rx_eth_mask_activ[0]) = 0xffffffff;
  *((volatile uint32_t*)&priv->regs->rx[0].rx_eth_mask_activ[1]) = 0x0000ffff;

  *((volatile uint32_t*)&priv->regs->mg_len_mask_ch0) = 0;
}



/* init/start hardware and allocate descriptor buffers for rx side
 *
 */
static int mgeth_start(mgeth_priv *priv, uint8_t *mac)
{
  struct mgeth_conf cfg;
  
  mgeth_reset_mac((uint32_t)priv->regs);
    
  cfg.is_full_duplex = true;
  cfg.speed = SPEED_1000;
  mgeth_init_mac((uint32_t)priv->regs, &cfg);

  // disable interrupts
  *((volatile uint32_t*)&priv->regs->mg_irq_mask )= 0;

  // set filters for given MAC
  mgeth_set_packet_filter(priv);

  // setup RX/TX queues
  init_descr(priv);

  // 1.4.1.6.4.4.2.12	IRQ_MASK_R(W) (R – 0xA14, W – 0x314)
  // int_desc	2	Завершено выполнение дескриптора с флагом Int
  *((volatile uint32_t*)&priv->regs->rx[0].irq_mask) = (1 << 2);
  
  // enable rx
  *((volatile uint32_t*)&priv->regs->rx[0].enable) = MGETH_ENABLE;
 
  return 0;
}

/* Stop the hardware from looking for packets - may be called even if
 *	 state == PASSIVE
 */
static void mgeth_stop(mgeth_priv *priv)
{
  *((volatile uint32_t*)&priv->regs->rx[0].enable) = 0;
  *((volatile uint32_t*)&priv->regs->tx[0].enable) = 0;
}

/* Send the bytes passed in "packet" as a packet on the wire */
static int mgeth_send(mgeth_priv *priv, void *eth_data, int data_length)
{
  uint32_t tickstart;

  // copy to internal buffer if size less than min
  if ( data_length < MGETH_MIN_PACKET_LEN ) {
    memset(&EthFrameTX[0][0], 0, MGETH_MIN_PACKET_LEN);
    memcpy(&EthFrameTX[0][0], eth_data, data_length);
    data_length = MGETH_MIN_PACKET_LEN;
  } else {	  
	  memcpy(&EthFrameTX[0][0], eth_data, data_length);
  }

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  // wait for completion of previous transfer     
  while ( ((*((volatile uint32_t*)&priv->regs->tx[0].enable) & MGETH_ENABLE) != 0) && ( (HAL_GetTick() - tickstart) < MGETH_TX_TIMEOUT) ); 

  if ( (HAL_GetTick() - tickstart) >= MGETH_TX_TIMEOUT ) {
		log_error("wait for completion of previous transfer failed");
		return -1;
	}

  priv->txbd_base[0].flags_length = MGETH_BD_STOP | data_length;
  priv->txbd_base[0].memptr = (unsigned int)&EthFrameTX[0][0];

  // enable tx
  *((volatile uint32_t*)&priv->regs->tx[0].desc_addr) = (uint32_t)&priv->txbd_base[0];
  *((volatile uint32_t*)&priv->regs->tx[0].enable) = MGETH_ENABLE;

  return 0;
}

static FrameTypeDef mgeth_recv(mgeth_priv *priv, uint32_t descr)
{
  struct __long_desc *curr_bd = &priv->rxbd_base[descr];
  FrameTypeDef frame = {0,0};

  if ( !(curr_bd->flags_length & MGETH_BD_OWN) ) {
    return (frame);  
  }

  frame.length = curr_bd->flags_length & MGETH_MAX_PACKET_LEN;
  frame.buffer = curr_bd->memptr;

  return (frame);
}

/* Give the driver an opportunity to manage its packet buffer memory
 *	     when the network stack is finished processing it. This will only be
 *	     called when no error was returned from recv
 */
static int mgeth_free_pkt(mgeth_priv *priv, uint32_t descr)
{
  priv->rxbd_base[descr].usrdata_h = 0;
  priv->rxbd_base[descr].usrdata_l = 0;
  // enable desciptor & set wrap last to first descriptor
  if ( descr >= (MGETH_RXBD_CNT - 1) ) {
    priv->rxbd_base[descr].flags_length = MGETH_BD_INT | MGETH_BD_LINK; // link descriptor
    priv->rxbd_base[descr].memptr = (unsigned int)&priv->rxbd_base[0];
	  //*((volatile uint32_t*)&priv->regs->rx[0].irq_mask) = (1 << 2);
    *((volatile uint32_t*)&priv->regs->rx[0].enable) = MGETH_ENABLE;
  } else {
    priv->rxbd_base[descr].flags_length = MGETH_BD_INT | MGETH_RXBUF_SIZE; // usual descriptor
    priv->rxbd_base[descr].memptr = (unsigned int)(&EthFrameRX[descr][0]);	
  }

  return 0;
}

static irq_return_t mgeth_irq_handler(unsigned int irq_num, void *dev_id) {
	struct net_device *nic_p = dev_id;
	uint32_t MDAM_status;
	FrameTypeDef frame;
	
	MDAM_status = *((volatile uint32_t*)&mgeth.regs->rx[0].status);
	
	if ( !(MDAM_status & (1 << 2)) ) {
		return IRQ_NONE;
	}
	
	if (!nic_p) {
		return IRQ_NONE;
	}
	
	irq_lock();
	
	for (mgeth.rxbd_no = 0; mgeth.rxbd_no < MGETH_RXBD_CNT; mgeth.rxbd_no++) {

		frame = mgeth_recv(&mgeth, mgeth.rxbd_no);
		
		if ( frame.length > 0 ) {
			/* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
			struct sk_buff *skb = skb_alloc(frame.length);
			/* copy received frame to pbuf chain */
			if (skb != NULL) {
				memcpy(skb->mac.raw, (const void *)frame.buffer, frame.length);						  
				skb->dev = nic_p;
				show_packet(skb->mac.raw, skb->len, "rx");
				netif_rx(skb);
			} else {
				log_error("skb_alloc failed");
			}
		}
		
		mgeth_free_pkt(&mgeth, mgeth.rxbd_no);
	}
	
	irq_unlock();
	
	return IRQ_HANDLED;
}

static int mgeth_xmit(struct net_device *dev, struct sk_buff *skb)
{
	if (dev == NULL) {
		return -EINVAL;
	}
	
	mgeth_send(&mgeth, skb->mac.raw, skb->len);	
	show_packet(skb->mac.raw, skb->len, "tx");	
	skb_free(skb);
	
	return ENOERR;
}


static int mgeth_open(struct net_device *dev)
{
	if (dev == NULL) {
		return -EINVAL;
	}	
	mgeth_start(&mgeth, (uint8_t *)&mgeth.dev_addr);
	
	return ENOERR;
}

static int mgeth_close(struct net_device *dev) {
	if (dev == NULL) {
		return -EINVAL;
	}	
	mgeth_stop(&mgeth);

	return ENOERR;
}

static int mgeth_set_mac(struct net_device *dev, const void *addr)
{
	if ((dev == NULL) || (addr == NULL)) {
		return -EINVAL;
	}
	
	memcpy(mgeth.dev_addr, addr, ETH_ALEN); 

	mgeth_stop(&mgeth);
	mgeth_start(&mgeth, (uint8_t *)&mgeth.dev_addr);
	
	return ENOERR;
}

/*
 * initializing procedure
 */
static struct net_device *mgeth_netdev;
static int mgeth_init(void) {
	int res;
	struct net_device *nic;
	unsigned char hw_addr[] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x02};
	
	nic = (struct net_device *) etherdev_alloc(0);
	if (nic == NULL) {
		return -ENOMEM;
	}
	
	nic->drv_ops = &mgeth_ops;
	nic->irq = MGETH_IRQ;
	nic->base_addr = OPTION_GET(NUMBER,mgeth_base);
	
	mgeth.regs = ((mgeth_regs *)nic->base_addr);
	mgeth.rxbd_base = (struct __long_desc *)&rxbd[0];
	mgeth.txbd_base = (struct __long_desc *)&txbd[0];
	
	/* Load current MAC address */
	memcpy(mgeth.dev_addr, hw_addr, ETH_ALEN);

	memset(mgeth.rxbd_base, 0, MGETH_RXBD_CNT * sizeof(struct __long_desc));
	memset(mgeth.txbd_base, 0, MGETH_TXBD_CNT * sizeof(struct __long_desc));
	
	mgeth_netdev = nic;
	
	res = irq_attach(nic->irq, mgeth_irq_handler, 0, nic, "");
	if (res < 0) {
		return res;
	}
	
	init_gpio_and_en_eth_phy(MGPIO0_BASE, MDIO0_BASE, AN_EN);
	mgeth_init_sgmii(SGMII_PHY_BASE, SCTL_BASE, AN_EN);
	
	return inetdev_register_dev(nic);	
}

STATIC_IRQ_ATTACH(MGETH_IRQ, mgeth_irq_handler, mgeth_netdev);
