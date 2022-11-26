/****************************************************************************
 * arch/arm/src/c5471/c5471_ethernet.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET)

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/ethernet.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

/* CONFIG_C5471_NET_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_C5471_NET_NINTERFACES
# define CONFIG_C5471_NET_NINTERFACES 1
#endif

/* CONFIG_C5471_NET_STATS will enabled collection of driver statistics.
 * Default is disabled.
 */

/* Mode of operation defaults to AUTONEGOTIATION */

#if defined(CONFIG_C5471_AUTONEGOTIATION)
# undef CONFIG_C5471_BASET100
# undef CONFIG_C5471_BASET10
#elif defined(CONFIG_C5471_BASET100)
# undef CONFIG_C5471_AUTONEGOTIATION
# undef CONFIG_C5471_BASET10
#elif defined(CONFIG_C5471_BASET10)
# undef CONFIG_C5471_AUTONEGOTIATION
# undef CONFIG_C5471_BASET100
#else
# define CONFIG_C5471_AUTONEGOTIATION 1
# undef CONFIG_C5471_BASET100
# undef CONFIG_C5471_BASET10
#endif

/* This should be disabled unless you are performing very low level debug */

#undef CONFIG_C5471_NET_DUMPBUFFER

/* Timing values ************************************************************/

/* TX timeout = 1 minute */

#define C5471_TXTIMEOUT       (60*CLK_TCK)

/* Ethernet GPIO bit settings ***********************************************/

#define GPIO_CIO_MDIO         0x00004000
#define GPIO_IO_MDCLK         0x00008000

/* Ethernet interface bit settings ******************************************/

/* TX descriptor, word #0 */

#define EIM_TXDESC_OWN_HOST   0x80000000 /* Bit 15: Ownership bit */
#define EIM_TXDESC_OWN_ENET   0x00000000
#define EIM_TXDESC_WRAP_NEXT  0x40000000 /* Bit 14: Descriptor chain wrap */
#define EIM_TXDESC_WRAP_FIRST 0x00000000
#define EIM_TXDESC_FIF        0x20000000 /* Bit 13: First in frame */
#define EIM_TXDESC_LIF        0x10000000 /* Bit 12: Last in frame */
                                         /* Bits 8-11: Retry count status */
#define EIM_TXDESC_INTRE      0x00800000 /* Bit 7: TX_IRQ int enable */
#define EIM_TXDESC_STATUSMASK 0x007f0000 /* Bits 0-6: Status */
#define EIM_TXDESC_RETRYERROR 0x00400000 /*   Exceed retry error */
#define EIM_TXDESC_HEARTBEAT  0x00200000 /*   Heartbeat (SQE) */
#define EIM_TXDESC_LCOLLISON  0x00100000 /*   Late collision error */
#define EIM_TXDESC_COLLISION  0x00080000 /*   Collision */
#define EIM_TXDESC_CRCERROR   0x00040000 /*   CRC error */
#define EIM_TXDESC_UNDERRUN   0x00020000 /*   Underrun error */
#define EIM_TXDESC_LOC        0x00010000 /*   Loss of carrier */

/* Packet bytes value used for both TX and RX descriptors */

#define EIM_PACKET_BYTES      0x00000040

/* Count of descriptors */

#define NUM_DESC_TX           32
#define NUM_DESC_RX           64

/* TX descriptor, word #1 */

                                         /* Bit 15: reserved */
#define EIM_TXDESC_PADCRC     0x00004000 /* Bit 14: Enable padding small frames */
                                         /* Bits 11-13: reserved */
#define EIM_TXDESC_BYTEMASK   0x000007ff /* Bits 0-10: Descriptor byte count */

/* RX descriptor, word #0 */

#define EIM_RXDESC_OWN_HOST   0x80000000 /* Bit 15: Ownership bit */
#define EIM_RXDESC_OWN_ENET   0x00000000
#define EIM_RXDESC_WRAP_NEXT  0x40000000 /* Bit 14: Descriptor chain wrap */
#define EIM_RXDESC_WRAP_FIRST 0x00000000
#define EIM_RXDESC_FIF        0x20000000 /* Bit 13: First in frame */
#define EIM_RXDESC_LIF        0x10000000 /* Bit 12: Last in frame */
                                         /* Bits 8-11: reserved */
#define EIM_RXDESC_INTRE      0x00800000 /* Bit 7: RX_IRQ int enable */
#define EIM_RXDESC_STATUSMASK 0x007f0000 /* Bits 0-6: Status */
#define EIM_RXDESC_MISS       0x00400000 /*   Miss */
#define EIM_RXDESC_VLAN       0x00200000 /*   VLAN */
#define EIM_RXDESC_LFRAME     0x00100000 /*   Long frame error */
#define EIM_RXDESC_SFRAME     0x00080000 /*   Short frame error */
#define EIM_RXDESC_CRCERROR   0x00040000 /*   CRC error */
#define EIM_RXDESC_OVERRUN    0x00020000 /*   Overrun error */
#define EIM_RXDESC_ALIGN      0x00010000 /*   Non-octect align error */

#define EIM_RXDESC_PADCRC     0x00004000 /* Enable padding for small frames */

/* RX descriptor, word #1 */

                                         /* Bits 11-15: reserved */
#define EIM_RXDESC_BYTEMASK   0x000007ff /* Bits 0-10: Descriptor byte count */

/* EIM_CPU_FILTER bit settings */

                                         /* Bits 5-31: reserved */
#define EIM_FILTER_MACLA      0x00000010 /* Bit 4: Enable logical address+multicast filtering */
#define EIM_FILTER_LOGICAL    0x00000008 /* Bit 3: Enable ENET logical filtering */
#define EIM_FILTER_MULTICAST  0x00000004 /* Bit 2: Enable multicast filtering */
#define EIM_FILTER_BROADCAST  0x00000002 /* Bit 1: Enable broadcast matching */
#define EIM_FILTER_UNICAST    0x00000001 /* Bit 0: Enable dest CPU address matching */

/* EIM_CTRL bit settings */

                                         /* Bits 16-31: Reserved */
#define EIM_CTRL_ESM_EN       0x00008000 /* Bit 15: Ethernet state machine enable */
                                         /* Bits 9-14: reserved */
#define EIM_CTRL_ENET0_EN     0x00000100 /* Bit 8: Enable routing of RX packets CPU->ENET0 */
                                         /* Bit 7: reserved */
#define EIM_CTRL_ENET0_FLW    0x00000040 /* Bit 6: Enable ENET0 flow control RX threshold */
#define EIM_CTRL_RXENET0_EN   0x00000020 /* Bit 5: Enable processing of ENET0 RX queue */
#define EIM_CTRL_TXENET0_EN   0x00000010 /* Bit 4: Enable processing of ENET0 TX queue */
                                         /* Bits 2-3: reserved */
#define EIM_CTRL_RXCPU_EN     0x00000002 /* Bit 1: Enable processing of CPU RX queue */
#define EIM_CTRL_TXCPU_EN     0x00000001 /* Bit 0: Enable processing of CPU TX queue */

/* EIM_STATUS bit settings */

                                         /* Bits 10-31: reserved */
#define EIM_STATUS_CPU_TXLIF  0x00000200 /* Bit 9: Last descriptor of TX packet filled */
#define EIM_STATUS_CPU_RXLIF  0x00000100 /* Bit 8: Last descriptor of RX queue processed */
#define EIM_STATUS_CPU_TX     0x00000080 /* Bit 7: Descriptor filled in TX queue */
#define EIM_STATUS_CPU_RX     0x00000040 /* Bit 6: Descriptor filled in RX queue */
                                         /* Bits 3-5: reserved */
#define EIM_STATUS_ENET0_ERR  0x00000004 /* Bit 2: ENET0 error interrupt */
#define EIM_STATUS_ENET0_TX   0x00000002 /* Bit 1: ENET0 TX interrupt */
#define EIM_STATUS_ENET0_RX   0x00000001 /* Bit 0" ENET0 RX interrupt */

/* EIM_INTEN bit settings */

#define EIM_INTEN_CPU_TXLIF  0x00000200 /* Bit 9: Last descriptor of TX packet filled */
#define EIM_INTEN_CPU_RXLIF  0x00000100 /* Bit 8: Last descriptor of RX queue processed */
#define EIM_INTEN_CPU_TX     0x00000080 /* Bit 7: Descriptor filled in TX queue */
#define EIM_INTEN_CPU_RX     0x00000040 /* Bit 6: Descriptor filled in RX queue */
                                        /* Bits 3-5: reserved */
#define EIM_INTEN_ENET0_ERR  0x00000004 /* Bit 2: ENET0 error interrupt */
#define EIM_INTEN_ENET0_TX   0x00000002 /* Bit 1: ENET0 TX interrupt */
#define EIM_INTEN_ENET0_RX   0x00000001 /* Bit 0: ENET0 RX interrupt */

/* ENET0_ADRMODE_EN bit settings */

#define ENET_ADR_PROMISCUOUS  0x00000008 /* Bit 3: Enable snoop address comparison */
#define ENET_ADR_BROADCAST    0x00000004 /* Bit 2: Enable broadcast address comparison */
#define ENET_ADDR_LCOMPARE    0x00000002 /* Bit 1: Enable logical address comparison */
#define ENET_ADDR_PCOMPARE    0x00000001 /* Bit 0: Enable physical address comparison */

/* ENET0_MODE bit settings */

                                         /* Bits 16-31: reserved */
#define ENET_MODO_FIFO_EN     0x00008000 /* Bit 15: Fifo enable */
                                         /* Bits 8-14: reserved */
#define ENET_MODE_RJCT_SFE    0x00000080 /* Bit 7: Reject short frames during receive */
#define ENET_MODE_DPNET       0x00000040 /* Bit 6: Demand priority networkd vs CSMA/CD */
#define ENET_MODE_MWIDTH      0x00000020 /* Bit 5: Select nibble mode MII port */
#define ENET_MODE_WRAP        0x00000010 /* Bit 4: Internal MAC loopback */
#define ENET_MODE_FDWRAP      0x00000008 /* Bit 3: Full duplex wrap */
#define ENET_MODE_FULLDUPLEX  0x00000004 /* Bit 2: 1:Full duplex */
#define ENET_MODE_HALFDUPLEX  0x00000000 /*        0:Half duplex */
                                         /* Bit 1: reserved */
#define ENET_MODE_ENABLE      0x00000001 /* Bit 0: Port enable */

/* PHY registers */

#define MD_PHY_CONTROL_REG    0x00
#define MD_PHY_MSB_REG        0x02
#define MD_PHY_LSB_REG        0x03
#define MD_PHY_CTRL_STAT_REG  0x17

/*  Lucent LU3X31T-T64 transeiver ID */

#define LU3X31_T64_PHYID      0x00437421

/* PHY control register bit settings */

#define MODE_AUTONEG          0x1000
#define MODE_10MBIT_HALFDUP   0x0000
#define MODE_10MBIT_FULLDUP   0x0100
#define MODE_100MBIT_FULLDUP  0x2100
#define MODE_100MBIT_HALFDUP  0x2000

/* Inserts an ARM "nop" instruction */

#define nop() asm("    nop");

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->c_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[CONFIG_C5471_NET_NINTERFACES]
                       [MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* The c5471_driver_s encapsulates all state information for a single c5471
 * hardware interface
 */

struct c5471_driver_s
{
  bool    c_bifup;           /* true:ifup false:ifdown */
  struct wdog_s c_txtimeout; /* TX timeout timer */
  struct work_s c_irqwork;   /* For deferring interrupt work to the work queue */
  struct work_s c_pollwork;  /* For deferring poll work to the work queue */

  /* Note: According to the C547x documentation: "The software has to
   * maintain two pointers to the current RX-CPU and TX-CPU descriptors.
   * At init time, they have to be set to the first descriptors of each
   * queue, and they have to be incremented each time a descriptor
   * ownership is give to the SWITCH".
   */

  volatile uint32_t c_txcpudesc;
  volatile uint32_t c_rxcpudesc;

  /* Last TX descriptor saved for error handling */

  uint32_t c_lastdescstart;
  uint32_t c_lastdescend;

  /* Shadowed registers */

  uint32_t c_eimstatus;

#ifdef CONFIG_C5471_NET_STATS
  /* TX statistics */

  uint32_t c_txpackets;      /* Number of packets sent */
  uint32_t c_txmiss;         /* Miss */
  uint32_t c_txvlan;         /* VLAN */
  uint32_t c_txlframe;       /* Long frame errors */
  uint32_t c_txsframe;       /* Short frame errors */
  uint32_t c_txcrc;          /* CRC errors */
  uint32_t c_txoverrun;      /* Overrun errors */
  uint32_t c_txalign;        /* Non-octect align errors */
  uint32_t c_txtimeouts;     /* TX timeouts */

  uint32_t c_rxpackets;      /* Number of packets received */
  uint32_t c_rxretries;      /* Exceed retry errors */
  uint32_t c_rxheartbeat;    /* Heartbeat (SQE) */
  uint32_t c_rxlcollision;   /* Late collision errors */
  uint32_t c_rxcollision;    /* Collision */
  uint32_t c_rxcrc;          /* CRC errors */
  uint32_t c_rxunderrun;     /* Underrun errors */
  uint32_t c_rxloc;          /* Loss of carrier */
  uint32_t c_rxdropped;      /* Packets dropped because of size */
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s c_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct c5471_driver_s g_c5471[CONFIG_C5471_NET_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transceiver interface */

static void c5471_mdtxbit (int bit_state);
static int  c5471_mdrxbit (void);
static void c5471_mdwrite (int adr, int reg, int data);
static int  c5471_mdread (int adr, int reg);
static int  c5471_phyinit (void);

/* Support logic */

static inline void c5471_inctxcpu(struct c5471_driver_s *priv);
static inline void c5471_incrxcpu(struct c5471_driver_s *priv);

/* Common TX logic */

static int  c5471_transmit(struct c5471_driver_s *priv);
static int  c5471_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

#ifdef CONFIG_C5471_NET_STATS
static void c5471_rxstatus(struct c5471_driver_s *priv);
#endif
static void c5471_receive(struct c5471_driver_s *priv);
#ifdef CONFIG_C5471_NET_STATS
static void c5471_txstatus(struct c5471_driver_s *priv);
#endif
static void c5471_txdone(struct c5471_driver_s *priv);

static void c5471_interrupt_work(void *arg);
static int  c5471_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void c5471_txtimeout_work(void *arg);
static void c5471_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int c5471_ifup(struct net_driver_s *dev);
static int c5471_ifdown(struct net_driver_s *dev);

static void c5471_txavail_work(void *arg);
static int c5471_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int c5471_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int c5471_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

/* Initialization functions */

static void c5471_eimreset (struct c5471_driver_s *priv);
static void c5471_eimconfig(struct c5471_driver_s *priv);
static void c5471_reset(struct c5471_driver_s *priv);
static void c5471_macassign(struct c5471_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: c5471_dumpbuffer
 *
 * Description:
 *   Debug only
 *
 ****************************************************************************/

#ifdef CONFIG_C5471_NET_DUMPBUFFER
static inline void c5471_dumpbuffer(const char *msg, const uint8_t *buffer,
                                    unsigned int nbytes)
{
  /* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_NET have to
   * be defined or the following does nothing.
   */

  ninfodumpbuffer(msg, buffer, nbytes);
}
#else
# define c5471_dumpbuffer(msg, buffer,nbytes)
#endif

/****************************************************************************
 * Name: c5471_mdtxbit
 *
 * Description:
 *   A helper routine used when serially communicating with the c547X's
 *   external ethernet transeiver device. GPIO pins are connected to the
 *   transeiver's MDCLK and MDIO pins and are used to accomplish the serial
 *   comm.
 *
 *   protocol:
 *                      ___________
 *     MDCLK   ________/           \_
 *              ________:____
 *     MDIO    <________:____>--------
 *                      :
 *                      ^
 *              Pin state internalized
 *
 ****************************************************************************/

static void c5471_mdtxbit (int bit_state)
{
  /* Note: any non-zero "bit_state" supplied by the caller means we should
   * clk a "1" out the MDIO pin.
   */

  /* Config MDIO as output pin. */

  putreg32((getreg32(GPIO_CIO) & ~GPIO_CIO_MDIO), GPIO_CIO);

  /* Select the bit output state */

  if (bit_state)
    {
      /* set MDIO state high. */

      putreg32((getreg32(GPIO_IO) | GPIO_CIO_MDIO), GPIO_IO);
    }
  else
    {
      /* set MDIO state low. */

      putreg32((getreg32(GPIO_IO) & ~GPIO_CIO_MDIO), GPIO_IO);
    }

  nop();
  nop();
  nop();
  nop();

  /* MDCLK rising edge */

  putreg32((getreg32(GPIO_IO) | GPIO_IO_MDCLK), GPIO_IO);
  nop();
  nop();

  /* release MDIO */

  putreg32((getreg32(GPIO_CIO) | GPIO_CIO_MDIO), GPIO_CIO);
  nop();
  nop();

  /* MDCLK falling edge. */

  putreg32((getreg32(GPIO_IO) & ~GPIO_IO_MDCLK), GPIO_IO);
}

/****************************************************************************
 * Name: c5471_mdrxbit
 *
 * Description:
 *    A helper routine used when serially communicating with the c547X's
 *    external ethernet transeiver device. GPIO pins are connected to the
 *    transeiver's MDCLK and MDIO pins and are used to accomplish the serial
 *    comm.
 *
 *    protocol:
 *                       ___________
 *       MDCLK  ________/           \_
 *              _______:_____
 *       MDIO   _______:_____>--------
 *                     :
 *                     ^
 *            pin state sample point
 *
 ****************************************************************************/

static int c5471_mdrxbit (void)
{
  register volatile uint32_t bit_state;

  /* config MDIO as input pin. */

  putreg32((getreg32(GPIO_CIO) | GPIO_CIO_MDIO), GPIO_CIO);

  /* Make sure the MDCLK is low */

  putreg32((getreg32(GPIO_IO) & ~GPIO_IO_MDCLK), GPIO_IO);
  nop();
  nop();
  nop();
  nop();

  /* Sample MDIO */

  bit_state = getreg32(GPIO_IO) & GPIO_CIO_MDIO;

  /* MDCLK rising edge */

  putreg32((getreg32(GPIO_IO) | GPIO_IO_MDCLK), GPIO_IO);
  nop();
  nop();
  nop();
  nop();

  /* MDCLK falling edge. */

  putreg32((getreg32(GPIO_IO) & ~GPIO_IO_MDCLK), GPIO_IO); /* MDCLK falling edge */
  if (bit_state)
    {
      return 1;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: c5471_mdwrite
 *
 * Description:
 *    A helper routine used when serially communicating with the c547X's
 *    external ethernet transeiver device. GPIO pins are connected to the
 *    transeiver's MDCLK and MDIO pins and are used to accomplish the serial
 *    comm.
 *
 ****************************************************************************/

static void c5471_mdwrite (int adr, int reg, int data)
{
  int i;

  /* preamble: 11111111111111111111111111111111 */

  for (i = 0; i < 32; i++)
    {
      c5471_mdtxbit(1);
    }

  /* start of frame: 01 */

  c5471_mdtxbit(0);
  c5471_mdtxbit(1);

  /* operation code: 01 - write */

  c5471_mdtxbit(0);
  c5471_mdtxbit(1);

  /* PHY device address: AAAAA, msb first */

  for (i = 0; i < 5; i++)
    {
      c5471_mdtxbit(adr & 0x10);
      adr = adr << 1;
    }

  /* MII register address: RRRRR, msb first */

  for (i = 0; i < 5; i++)
    {
      c5471_mdtxbit(reg & 0x10);
      reg = reg << 1;
    }

  /* Turnaround time: ZZ */

  c5471_mdtxbit(1);
  c5471_mdtxbit(0);

  /* data: DDDDDDDDDDDDDDDD, msb first */

  for (i = 0; i < 16; i++)
    {
      c5471_mdtxbit(data & 0x8000);
      data = data << 1;
    }
}

/****************************************************************************
 * Name: c5471_mdread
 *
 * Description:
 *    A helper routine used when serially communicating with the c547X's
 *    external ethernet transeiver device. GPIO pins are connected to the
 *    transeiver's MDCLK and MDIO pins and are used to accomplish the serial
 *    comm.
 *
 ****************************************************************************/

static int c5471_mdread (int adr, int reg)
{
  int i;
  int data = 0;

  /* preamble: 11111111111111111111111111111111 */

  for (i = 0; i < 32; i++)
    {
      c5471_mdtxbit(1);
    }

  /* start of frame: 01 */

  c5471_mdtxbit(0);
  c5471_mdtxbit(1);

  /* operation code: 10 - read */

  c5471_mdtxbit(1);
  c5471_mdtxbit(0);

  /* PHY device address: AAAAA, msb first */

  for (i = 0; i < 5; i++)
    {
      c5471_mdtxbit(adr & 0x10);
      adr = adr << 1;
    }

  /* MII register address: RRRRR, msb first */

  for (i = 0; i < 5; i++)
    {
      c5471_mdtxbit(reg & 0x10);
      reg = reg << 1;
    }

  /* turnaround time: ZZ */

  c5471_mdrxbit();
  c5471_mdrxbit(); /* PHY should drive a 0 */

  /* data: DDDDDDDDDDDDDDDD, msb first */

  for (i = 0; i < 16; i++)
    {
      data = data << 1;
      data |= c5471_mdrxbit();
    }

  return data;
}

/****************************************************************************
 * Name: c5471_phyinit
 *
 * Description:
 *   The c547X EVM board uses a Lucent LU3X31T-T64 transeiver device to
 *   handle the physical layer (PHY). It's a h/w block that on the one end
 *   offers a Media Independent Interface (MII) which is connected to the
 *   Ethernet Interface Module (EIM) internal to the C547x and on the other
 *   end offers either the 10baseT or 100baseT electrical interface
 *   connecting to an RJ45 onboard network connector. The PHY transeiver has
 *   several internal registers allowing host configuration and status
 *   access. These internal registers are accessible by clocking serial data
 *   in/out of the MDIO pin of the LU3X31T-T64 chip. For c547X, the MDC and
 *   the MDIO pins are connected to the C547x GPIO15 and GPIO14 pins
 *   respectively. Host software twiddles the GPIO pins appropriately to get
 *   data serially into and out of the chip. This is typically a one time
 *   operation at boot and normal operation of the transeiver involves
 *   EIM/Transeiver interaction at the other pins of the transeiver chip and
 *   doesn't require host intervention at the MDC and MDIO pins.
 *
 ****************************************************************************/

#if defined(CONFIG_C5471_PHY_LU3X31T_T64)
static int c5471_phyinit (void)
{
  int phyid;
  int status;

  /* Next, Setup GPIO pins to talk serially to the Lucent transeiver chip */

  /* enable gpio bits 15,14 */

  putreg32((getreg32(GPIO_EN) | 0x0000c000), GPIO_EN);

  /* config gpio(15); out -> MDCLK */

  putreg32((getreg32(GPIO_CIO) & ~0x00008000), GPIO_CIO);

  /* config gpio(14); in <- MDIO */

  putreg32((getreg32(GPIO_CIO) | 0x00004000), GPIO_CIO);

  /* initial pin state; MDCLK = 0 */

  putreg32((getreg32(GPIO_IO) & 0x000f3fff), GPIO_IO);

  /* Next, request a chip reset */

  c5471_mdwrite(0, MD_PHY_CONTROL_REG, 0x8000);
  while (c5471_mdread(0, MD_PHY_CONTROL_REG) & 0x8000)
    {
      /* wait for chip reset to complete */
    }

  /* Next, Read out the chip ID */

  phyid = (c5471_mdread(0, MD_PHY_MSB_REG) << 16) |
          c5471_mdread(0, MD_PHY_LSB_REG);
  if (phyid != LU3X31_T64_PHYID)
    {
      nerr("ERROR: Unrecognized PHY ID: %08x\n", phyid);
      return -ENODEV;
    }

  /* Next, Set desired network rate, 10BaseT, 100BaseT, or auto. */

#ifdef CONFIG_C5471_AUTONEGOTIATION
  ninfo("Setting PHY Transceiver for Autonegotiation\n");
  c5471_mdwrite(0, MD_PHY_CONTROL_REG, MODE_AUTONEG);
#endif
#ifdef CONFIG_C5471_BASET100
  ninfo("Setting PHY Transceiver for 100BaseT FullDuplex\n");
  c5471_mdwrite(0, MD_PHY_CONTROL_REG, MODE_100MBIT_FULLDUP);
#endif
#ifdef CONFIG_C5471_BASET10
  ninfo("Setting PHY Transceiver for 10BaseT FullDuplex\n");
  c5471_mdwrite(0, MD_PHY_CONTROL_REG, MODE_10MBIT_FULLDUP);
#endif

  status = c5471_mdread(0, MD_PHY_CTRL_STAT_REG);
  return status;
}

#elif defined(CONFIG_C5471_PHY_AC101L)

static int c5471_phyinit (void)
{
  int phyid;
  int status;

  /* Next, Setup GPIO pins to talk serially to the Lucent transeiver chip */

  putreg32((getreg32(GPIO_EN)  |  0x0000c000), GPIO_EN);  /* enable gpio bits 15,14 */
  putreg32((getreg32(GPIO_CIO) & ~0x00008000), GPIO_CIO); /* config gpio(15); out -> MDCLK */
  putreg32((getreg32(GPIO_CIO) |  0x00004000), GPIO_CIO); /* config gpio(14); in <- MDIO */
  putreg32((getreg32(GPIO_IO)  &  0x000f3fff), GPIO_IO);  /* initial pin state; MDCLK = 0 */

  return 1;
}

#else
#  define c5471_phyinit()
#  warning "Assuming no PHY"
#endif

/****************************************************************************
 * Name: c5471_inctxcpu
 *
 * Description:
 *
 ****************************************************************************/

static inline void c5471_inctxcpu(struct c5471_driver_s *priv)
{
  if (EIM_TXDESC_WRAP_NEXT & getreg32(priv->c_txcpudesc))
    {
      /* Loop back around to base of descriptor queue */

      priv->c_txcpudesc = getreg32(EIM_CPU_TXBA) + EIM_RAM_START;
    }
  else
    {
      priv->c_txcpudesc += 2*sizeof(uint32_t);
    }

  ninfo("TX CPU desc: %08" PRIx32 "\n", priv->c_txcpudesc);
}

/****************************************************************************
 * Name: c5471_incrxcpu
 *
 * Description:
 *
 ****************************************************************************/

static inline void c5471_incrxcpu(struct c5471_driver_s *priv)
{
  if (EIM_RXDESC_WRAP_NEXT & getreg32(priv->c_rxcpudesc))
    {
      /* Loop back around to base of descriptor queue */

      priv->c_rxcpudesc = getreg32(EIM_CPU_RXBA) + EIM_RAM_START;
    }
  else
    {
      priv->c_rxcpudesc += 2*sizeof(uint32_t);
    }

  ninfo("RX CPU desc: %08" PRIx32 "\n", priv->c_rxcpudesc);
}

/****************************************************************************
 * Function: c5471_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int c5471_transmit(struct c5471_driver_s *priv)
{
  struct net_driver_s *dev = &priv->c_dev;
  volatile uint16_t *packetmem;
  uint16_t framelen;
  bool bfirstframe;
  int nbytes;
  int nshorts;
  unsigned int i;
  unsigned int j;

  nbytes                = (dev->d_len + 1) & ~1;
  j                     = 0;
  bfirstframe           = true;
  priv->c_lastdescstart = priv->c_rxcpudesc;

  ninfo("Packet size: %d RX CPU desc: %08" PRIx32 "\n",
        nbytes, priv->c_rxcpudesc);
  c5471_dumpbuffer("Transmit packet", dev->d_buf, dev->d_len);

  while (nbytes)
    {
      /* Verify that the hardware is ready to send another packet
       * Words #0 and #1 of descriptor
       */

      while (EIM_TXDESC_OWN_HOST & getreg32(priv->c_rxcpudesc))
        {
          /* Loop until the SWITCH lets go of the descriptor giving us access
           * rights to submit our new ether frame to it.
           */
        }

      if (bfirstframe)
        {
          putreg32((getreg32(priv->c_rxcpudesc) | EIM_RXDESC_FIF),
                   priv->c_rxcpudesc);
        }
      else
        {
          putreg32((getreg32(priv->c_rxcpudesc) & ~EIM_RXDESC_FIF),
                   priv->c_rxcpudesc);
        }

      putreg32((getreg32(priv->c_rxcpudesc) & ~EIM_RXDESC_PADCRC),
               priv->c_rxcpudesc);

      if (bfirstframe)
        {
          putreg32((getreg32(priv->c_rxcpudesc) | EIM_RXDESC_PADCRC),
                   priv->c_rxcpudesc);
        }

      if (nbytes >= EIM_PACKET_BYTES)
        {
          framelen = EIM_PACKET_BYTES;
        }
      else
        {
          framelen = nbytes;
        }

      /* Submit ether frame bytes to the C5472 Ether Module packet memory
       * space. Get the number of 16-bit values to transfer by dividing by
       * 2 with round up.
       */

      nshorts = (framelen + 1) >> 1;

      /* Words #2 and #3 of descriptor */

      packetmem = (uint16_t *)getreg32(priv->c_rxcpudesc + sizeof(uint32_t));
      for (i = 0; i < nshorts; i++, j++)
        {
          /* 16-bits at a time. */

          packetmem[i] = HTONS(((uint16_t *)dev->d_buf)[j]);
        }

      putreg32(((getreg32(priv->c_rxcpudesc) & ~EIM_RXDESC_BYTEMASK) |
                framelen), priv->c_rxcpudesc);
      nbytes -= framelen;
      ninfo("Wrote framelen: %d nbytes: %d nshorts: %d\n",
            framelen, nbytes, nshorts);

      if (0 == nbytes)
        {
          putreg32((getreg32(priv->c_rxcpudesc) | EIM_RXDESC_LIF),
                   priv->c_rxcpudesc);
        }
      else
        {
          putreg32((getreg32(priv->c_rxcpudesc) & ~EIM_RXDESC_LIF),
                   priv->c_rxcpudesc);
        }

      /* We're done with that descriptor; give access rights back to h/w */

      putreg32((getreg32(priv->c_rxcpudesc) | EIM_RXDESC_OWN_HOST),
               priv->c_rxcpudesc);

      /* Next, tell Ether Module that
       * those submitted bytes are ready for the wire
       */

      putreg32(0x00000001, EIM_CPU_RXREADY);
      priv->c_lastdescend = priv->c_rxcpudesc;

      /* Advance to the next free descriptor */

      c5471_incrxcpu(priv);
      bfirstframe = false;
    }

  /* Packet transferred .. Update statistics */

#ifdef CONFIG_C5471_NET_STATS
  priv->c_txpackets++;
#endif

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->c_txtimeout, C5471_TXTIMEOUT,
           c5471_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: c5471_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send. This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int c5471_txpoll(struct net_driver_s *dev)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)dev->d_private;

  /* Send the packet */

  c5471_transmit(priv);

  /* Check if the ESM has let go of the RX descriptor giving us
   * access rights to submit another Ethernet frame.
   */

  if ((EIM_TXDESC_OWN_HOST & getreg32(priv->c_rxcpudesc)) != 0)
    {
      /* No, then return non-zero to terminate the poll */

      return 1;
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: c5471_rxstatus
 *
 * Description:
 *   An interrupt was received indicating that the last RX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_C5471_NET_STATS
static void c5471_rxstatus(struct c5471_driver_s *priv)
{
  uint32_t desc = priv->c_txcpudesc;
  uint32_t rxstatus;

  /* Walk that last packet we just received to collect xmit status bits. */

  rxstatus = 0;
  for (; ; )
    {
      if (EIM_TXDESC_OWN_HOST & getreg32(desc))
        {
          /* The incoming packet queue is empty. */

          break;
        }

      rxstatus |= (getreg32(desc) & EIM_TXDESC_STATUSMASK);

      if ((getreg32(desc) & EIM_TXDESC_LIF) != 0)
        {
          break;
        }

      /* This packet is made up of several descriptors,
       * find next one in chain.
       */

      if (EIM_TXDESC_WRAP_NEXT & getreg32(desc))
        {
          /* Loop back around to base of descriptor queue. */

          desc = getreg32(EIM_CPU_TXBA) + EIM_RAM_START;
        }
      else
        {
          desc += 2 * sizeof(uint32_t);
        }
    }

  if (rxstatus != 0)
    {
      if ((rxstatus & EIM_TXDESC_RETRYERROR) != 0)
        {
          priv->c_rxretries++;
          ninfo("c_rxretries: %d\n", priv->c_rxretries);
        }

      if ((rxstatus & EIM_TXDESC_HEARTBEAT) != 0)
        {
          priv->c_rxheartbeat++;
          ninfo("c_rxheartbeat: %d\n", priv->c_rxheartbeat);
        }

      if ((rxstatus & EIM_TXDESC_LCOLLISON) != 0)
        {
          priv->c_rxlcollision++;
          ninfo("c_rxlcollision: %d\n", priv->c_rxlcollision);
        }

      if ((rxstatus & EIM_TXDESC_COLLISION) != 0)
        {
          priv->c_rxcollision++;
          ninfo("c_rxcollision: %d\n", priv->c_rxcollision);
        }

      if ((rxstatus & EIM_TXDESC_CRCERROR) != 0)
        {
          priv->c_rxcrc++;
          ninfo("c_rxcrc: %d\n", priv->c_rxcrc);
        }

      if ((rxstatus & EIM_TXDESC_UNDERRUN) != 0)
        {
          priv->c_rxunderrun++;
          ninfo("c_rxunderrun: %d\n", priv->c_rxunderrun);
        }

      if ((rxstatus & EIM_TXDESC_LOC) != 0)
        {
          priv->c_rxloc++;
          ninfo("c_rxloc: %d\n", priv->c_rxloc);
        }
    }
}
#endif

/****************************************************************************
 * Function: c5471_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void c5471_receive(struct c5471_driver_s *priv)
{
  struct net_driver_s *dev = &priv->c_dev;
  uint16_t *packetmem;
  bool bmore = true;
  int packetlen = 0;
  int framelen;
  int nshorts;
  int i;
  int j = 0;

  /* Walk the newly received packet contained within the EIM and transfer
   * its contents to the network buffer. This frees up the memory contained
   * within the EIM for additional packets that might be received later from
   * the network.
   */

  ninfo("Reading TX CPU desc: %08" PRIx32 "\n", priv->c_txcpudesc);
  while (bmore)
    {
      /* Words #0 and #1 of descriptor */

      if (EIM_TXDESC_OWN_HOST & getreg32(priv->c_txcpudesc))
        {
          /* No further packets to receive. */

          break;
        }

      /* Get the size of the frame from words #0 and #1 of the descriptor
       * and update the accumulated packet size
       */

      framelen   = (getreg32(priv->c_txcpudesc) & EIM_TXDESC_BYTEMASK);
      packetlen += framelen;

      /* Check if the received packet will fit
       * within the network packet buffer
       */

      if (packetlen < (CONFIG_NET_ETH_PKTSIZE + 4))
        {
          /* Get the packet memory from words #2 and #3 of descriptor */

          packetmem = (uint16_t *)
            getreg32(priv->c_txcpudesc + sizeof(uint32_t));

          /* Divide by 2 with round up to get the number of 16-bit words. */

          nshorts = (framelen + 1) >> 1;
          ninfo("Reading framelen: "
                "%d packetlen: %d nshorts: %d packetmen: %p\n",
                 framelen, packetlen, nshorts, packetmem);

          for (i = 0 ; i < nshorts; i++, j++)
            {
              /* Copy the data data from the hardware to priv->c_dev.d_buf
               * 16-bits at a time.
               */

              ((uint16_t *)dev->d_buf)[j] = HTONS(packetmem[i]);
            }
        }
      else
        {
          ninfo("Discarding framelen: %d packetlen %d\n",
                framelen, packetlen);
        }

      if (getreg32(priv->c_txcpudesc) & EIM_TXDESC_LIF)
        {
          bmore = false;
        }

      /* Next, Clear all bits of words0/1 of the emptied descriptor
       * except preserve the settings of a select few. Can leave
       * descriptor words 2/3 alone.
       */

      putreg32((getreg32(priv->c_txcpudesc) &
               (EIM_TXDESC_WRAP_NEXT | EIM_TXDESC_INTRE)),
               priv->c_txcpudesc);

      /* Next, Give ownership of now emptied descriptor back to
       * the Ether Module's SWITCH
       */

      putreg32((getreg32(priv->c_txcpudesc) | EIM_TXDESC_OWN_HOST),
               priv->c_txcpudesc);

      /* Advance to the next data buffer */

      c5471_inctxcpu(priv);
    }

  /* Adjust the packet length to remove the CRC bytes
   * that the network doesn't care about.
   */

  packetlen -= 4;

#ifdef CONFIG_C5471_NET_STATS
  /* Increment the count of received packets */

  priv->c_rxpackets++;
#endif

  /* If we successfully transferred the data into the network buffer,
   * then pass it on to the network for processing.
   */

  if (packetlen > 0 && packetlen < CONFIG_NET_ETH_PKTSIZE)
    {
      /* Set amount of data in priv->c_dev.d_len. */

      dev->d_len = packetlen;
      ninfo("Received packet, packetlen: %d type: %02x\n",
            packetlen, NTOHS(BUF->type));
      c5471_dumpbuffer("Received packet", dev->d_buf, dev->d_len);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Receive an IPv4 packet from the network device */

          ipv4_input(dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           * Send that data now if ESM has let go of the RX descriptor giving
           * us access rights to submit another Ethernet frame.
           */

          if (dev->d_len > 0 &&
             (EIM_TXDESC_OWN_HOST & getreg32(priv->c_rxcpudesc)) == 0)
            {
              /* And send the packet */

               c5471_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           * Send that data now if ESM has let go of the RX descriptor giving
           * us access rights to submit another Ethernet frame.
           */

          if (dev->d_len > 0 &&
             (EIM_TXDESC_OWN_HOST & getreg32(priv->c_rxcpudesc)) == 0)
            {
              /* And send the packet */

               c5471_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          arp_arpin(dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           * Send that data now if ESM has let go of the RX descriptor giving
           * us access rights to submit another Ethernet frame.
           */

          if (dev->d_len > 0 &&
             (EIM_TXDESC_OWN_HOST & getreg32(priv->c_rxcpudesc)) == 0)
            {
              c5471_transmit(priv);
            }
        }
#endif
    }
#ifdef CONFIG_C5471_NET_STATS
  else
    {
      /* Increment the count of dropped packets */

      nwarn("WARNING: Too big! packetlen: %d\n", packetlen);
      priv->c_rxdropped++;
    }
#endif
}

/****************************************************************************
 * Function: c5471_txstatus
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_C5471_NET_STATS
static void c5471_txstatus(struct c5471_driver_s *priv)
{
  uint32_t desc = priv->c_lastdescstart;
  uint32_t txstatus;

  /* Walk that last packet we just sent to collect xmit status bits. */

  txstatus = 0;
  if (priv->c_lastdescstart && priv->c_lastdescend)
    {
      for (; ; )
        {
          txstatus |= (getreg32(desc) & EIM_RXDESC_STATUSMASK);
          if (desc == priv->c_lastdescend)
            {
              break;
            }

          /* This packet is made up of several descriptors,
           * find next one in chain.
           */

          if (EIM_RXDESC_WRAP_NEXT & getreg32(priv->c_rxcpudesc))
            {
              /* Loop back around to base of descriptor queue. */

              desc = getreg32(EIM_CPU_RXBA) + EIM_RAM_START;
            }
          else
            {
              desc += 2 * sizeof(uint32_t);
            }
        }
    }

  if (txstatus)
    {
      if ((txstatus & EIM_RXDESC_MISS) != 0)
        {
          priv->c_txmiss++;
          ninfo("c_txmiss: %d\n", priv->c_txmiss);
        }

      if ((txstatus & EIM_RXDESC_VLAN) != 0)
        {
          priv->c_txvlan++;
          ninfo("c_txvlan: %d\n", priv->c_txvlan);
        }

      if ((txstatus & EIM_RXDESC_LFRAME) != 0)
        {
          priv->c_txlframe++;
          ninfo("c_txlframe: %d\n", priv->c_txlframe);
        }

      if ((txstatus & EIM_RXDESC_SFRAME) != 0)
        {
          priv->c_txsframe++;
          ninfo("c_txsframe: %d\n", priv->c_txsframe);
        }

      if ((txstatus & EIM_RXDESC_CRCERROR) != 0)
        {
          priv->c_txcrc++;
          ninfo("c_txcrc: %d\n", priv->c_txcrc);
        }

      if ((txstatus & EIM_RXDESC_OVERRUN) != 0)
        {
          priv->c_txoverrun++;
          ninfo("c_txoverrun: %d\n", priv->c_txoverrun);
        }

      if ((txstatus & EIM_RXDESC_OVERRUN) != 0)
        {
          priv->c_txalign++;
          ninfo("c_txalign: %d\n", priv->c_txalign);
        }
    }
}
#endif

/****************************************************************************
 * Function: c5471_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void c5471_txdone(struct c5471_driver_s *priv)
{
  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(&priv->c_txtimeout);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->c_dev, c5471_txpoll);
}

/****************************************************************************
 * Function: c5471_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void c5471_interrupt_work(void *arg)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)arg;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Get and clear interrupt status bits */

  priv->c_eimstatus = getreg32(EIM_STATUS);

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call c5471_receive() */

  if ((EIM_STATUS_CPU_TX & priv->c_eimstatus) != 0)
    {
      /* An incoming packet has been received by the EIM from the network and
       * the interrupt associated with EIM's CPU TX queue has been asserted.
       * It is the EIM's CPU TX queue that we need to read from to get those
       * packets.  We use this terminology to stay consistent with the Orion
       * documentation.
       */

#ifdef CONFIG_C5471_NET_STATS
      /* Check for RX errors */

      c5471_rxstatus(priv);
#endif

      /* Process the received packet */

      c5471_receive(priv);
    }

  /* Check is a packet transmission just completed.
   * If so, call c5471_txdone
   */

  if ((EIM_STATUS_CPU_RX & priv->c_eimstatus) != 0)
    {
      /* An outgoing packet has been processed by the EIM and the interrupt
       * associated with EIM's CPU RX queue has been asserted. It is EIM's
       * CPU RX queue that we put packets on to send them *out*. TWe use this
       * terminology to stay consistent with the Orion documentation.
       */

#ifdef CONFIG_C5471_NET_STATS
      /* Check for TX errors */

      c5471_txstatus(priv);
#endif

      /* Handle the transmission done event */

      c5471_txdone(priv);
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(C5471_IRQ_ETHER);
}

/****************************************************************************
 * Function: c5471_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int c5471_interrupt(int irq, void *context, void *arg)
{
#if CONFIG_C5471_NET_NINTERFACES == 1
  register struct c5471_driver_s *priv = &g_c5471[0];
#else
# error "Additional logic needed to support multiple interfaces"
#endif

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(C5471_IRQ_ETHER);

  /* TODO: Determine if a TX transfer just completed */

    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(&priv->c_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->c_irqwork, c5471_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: c5471_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void c5471_txtimeout_work(void *arg)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)arg;

  /* Increment statistics */

  net_lock();
#ifdef CONFIG_C5471_NET_STATS
  priv->c_txtimeouts++;
  ninfo("c_txtimeouts: %d\n", priv->c_txtimeouts);
#endif

  /* Then try to restart the hardware */

  c5471_ifdown(&priv->c_dev);
  c5471_ifup(&priv->c_dev);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->c_dev, c5471_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: c5471_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void c5471_txtimeout_expiry(wdparm_t arg)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(C5471_IRQ_ETHER);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * canceling any pending IRQ work.
   */

  work_queue(ETHWORK, &priv->c_irqwork, c5471_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: c5471_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The user has assigned a MAC to the driver
 *
 ****************************************************************************/

static int c5471_ifup(struct net_driver_s *dev)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)dev->d_private;
  volatile uint32_t clearbits;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));

  /* Initialize Ethernet interface */

  c5471_reset(priv);

  /* Assign the MAC to the device */

  c5471_macassign(priv);

  /* Clear pending interrupts by reading the EIM status register */

  clearbits = getreg32(EIM_STATUS);
  UNUSED(clearbits);

  /* Enable interrupts going from EIM Module to Interrupt Module. */

  putreg32(((getreg32(EIM_INTEN) | EIM_INTEN_CPU_TX | EIM_INTEN_CPU_RX)),
           EIM_INTEN);

  /* Next, go on-line. According to the C547X documentation the enables have
   * to occur in this order to insure proper operation; ESM first then ENET.
   */

  putreg32((getreg32(EIM_CTRL) | EIM_CTRL_ESM_EN), EIM_CTRL);      /* enable ESM */
  putreg32((getreg32(ENET0_MODE) | ENET_MODE_ENABLE), ENET0_MODE); /* enable ENET */
  up_mdelay(100);

  /* Enable the Ethernet interrupt */

  priv->c_bifup = true;
  up_enable_irq(C5471_IRQ_ETHER);
  return OK;
}

/****************************************************************************
 * Function: c5471_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int c5471_ifdown(struct net_driver_s *dev)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Stopping\n");

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(C5471_IRQ_ETHER);

  /* Disable interrupts going from EIM Module to Interrupt Module. */

  putreg32((getreg32(EIM_INTEN) & ~(EIM_INTEN_CPU_TX | EIM_INTEN_CPU_RX)),
           EIM_INTEN);

  /* Disable ENET */

  putreg32((getreg32(ENET0_MODE) & ~ENET_MODE_ENABLE), ENET0_MODE); /* disable ENET */

  /* Disable ESM */

  putreg32((getreg32(EIM_CTRL) & ~EIM_CTRL_ESM_EN), EIM_CTRL);  /* disable ESM */

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->c_txtimeout);

  /* Reset the device */

  priv->c_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: c5471_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void c5471_txavail_work(void *arg)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)arg;

  ninfo("Polling\n");

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->c_bifup)
    {
      /* Check if the ESM has let go of the RX descriptor giving us access
       * rights to submit another Ethernet frame.
       */

      if ((EIM_TXDESC_OWN_HOST & getreg32(priv->c_rxcpudesc)) == 0)
        {
          /* If so, then poll the network for new XMIT data */

          devif_poll(&priv->c_dev, c5471_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: c5471_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int c5471_txavail(struct net_driver_s *dev)
{
  struct c5471_driver_s *priv = (struct c5471_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->c_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->c_pollwork, c5471_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: c5471_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int c5471_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct c5471_driver_s *priv =
    (struct c5471_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Function: c5471_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int c5471_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct c5471_driver_s *priv =
    (struct c5471_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Name: c5471_eimreset
 *
 * Description:
 *   The C547x docs states that a module should generally be reset according
 *   to the following algorithm:
 *
 *   1. Put the module in reset.
 *   2. Switch on the module clock.
 *   3. Wait for eight clock cycles.
 *   4. Release the reset.
 *
 ****************************************************************************/

static void c5471_eimreset (struct c5471_driver_s *priv)
{
  /* Stop the EIM module clock */

  putreg32((getreg32(CLKM) | CLKM_EIM_CLK_STOP), CLKM);

  /* Put EIM module in reset */

  putreg32((getreg32(CLKM_RESET) & ~CLKM_RESET_EIM), CLKM_RESET);

  /* Start the EIM module clock */

  putreg32((getreg32(CLKM) & ~CLKM_EIM_CLK_STOP), CLKM);

  /* Assert nRESET to reset the board's PHY0/1 chips */

  putreg32((CLKM_CTL_RST_EXT_RESET | CLKM_CTL_RST_LEAD_RESET),
           CLKM_CTL_RST);
  up_mdelay(2);

  /* Release the peripheral nRESET signal */

  putreg32(CLKM_CTL_RST_LEAD_RESET, CLKM_CTL_RST);

  /* Release EIM module reset */

  putreg32((getreg32(CLKM_RESET) | CLKM_RESET_EIM), CLKM_RESET);

  /* All EIM register should now be in there power-up default states */

  priv->c_lastdescstart = 0;
  priv->c_lastdescend   = 0;
}

/****************************************************************************
 * Name: c5471_eimconfig
 *
 * Description:
 *    Assumes that all registers are currently in the power-up reset state.
 *    This routine then modifies that state to provide our specific ethernet
 *    configuration.
 *
 ****************************************************************************/

static void c5471_eimconfig(struct c5471_driver_s *priv)
{
  volatile uint32_t pbuf;
  volatile uint32_t desc;
  volatile uint32_t val;
  int i;

  desc = EIM_RAM_START;
  pbuf = EIM_RAM_START + 0x6c0;

  /* TX ENET 0 */

  ninfo("TX ENET0 desc: %08" PRIx32 " pbuf: %08" PRIx32 "\n", desc, pbuf);
  putreg32((desc & 0x0000ffff), ENET0_TDBA); /* 16-bit offset address */
  for (i = NUM_DESC_TX - 1; i >= 0; i--)
    {
      if (i == 0)
        val = EIM_TXDESC_WRAP_NEXT;
      else
        val = EIM_TXDESC_WRAP_FIRST;

      val |= EIM_TXDESC_OWN_HOST | EIM_TXDESC_INTRE | EIM_TXDESC_PADCRC |
             EIM_PACKET_BYTES;

      putreg32(val, desc);
      desc += sizeof(uint32_t);

      putreg32(pbuf, desc);
      desc += sizeof(uint32_t);

      putreg32(0, pbuf);
      pbuf += EIM_PACKET_BYTES;

      putreg32(0, pbuf);
      pbuf += sizeof(uint32_t); /* Ether Module's "Buffer Usage Word" */
    }

  /* RX ENET 0 */

  ninfo("RX ENET0 desc: %08" PRIx32 " pbuf: %08" PRIx32 "\n", desc, pbuf);
  putreg32((desc & 0x0000ffff), ENET0_RDBA); /* 16-bit offset address */
  for (i = NUM_DESC_RX - 1; i >= 0; i--)
    {
      if (i == 0)
        val = EIM_RXDESC_WRAP_NEXT;
      else
        val = EIM_RXDESC_WRAP_FIRST;

      val |= EIM_RXDESC_OWN_ENET | EIM_RXDESC_INTRE | EIM_RXDESC_PADCRC |
             EIM_PACKET_BYTES;

      putreg32(val, desc);
      desc += sizeof(uint32_t);

      putreg32(pbuf, desc);
      desc += sizeof(uint32_t);

      putreg32(0, pbuf);
      pbuf += EIM_PACKET_BYTES;

      putreg32(0, pbuf);
      pbuf += sizeof(uint32_t); /* Ether Module's "Buffer Usage Word" */
    }

  /* TX CPU */

  ninfo("TX CPU desc: %08" PRIx32 " pbuf: %08" PRIx32 "\n", desc, pbuf);
  priv->c_txcpudesc = desc;
  putreg32((desc & 0x0000ffff), EIM_CPU_TXBA); /* 16-bit offset address */
  for (i = NUM_DESC_TX - 1; i >= 0; i--)
    {
      /* Set words 1+2 of the TXDESC */

      if (i == 0)
        val = EIM_TXDESC_WRAP_NEXT;
      else
        val = EIM_TXDESC_WRAP_FIRST;

      val |= EIM_TXDESC_OWN_HOST | EIM_TXDESC_INTRE | EIM_TXDESC_PADCRC |
             EIM_PACKET_BYTES;

      putreg32(val, desc);
      desc += sizeof(uint32_t);

      putreg32(pbuf, desc);
      desc += sizeof(uint32_t);

      putreg32(0, pbuf);
      pbuf += EIM_PACKET_BYTES;

      putreg32(0, pbuf);
      pbuf += sizeof(uint32_t); /* Ether Module's "Buffer Usage Word" */
    }

  /* RX CPU */

  ninfo("RX CPU desc: %08" PRIx32 " pbuf: %08" PRIx32 "\n", desc, pbuf);
  priv->c_rxcpudesc = desc;
  putreg32((desc & 0x0000ffff), EIM_CPU_RXBA); /* 16-bit offset address */
  for (i = NUM_DESC_RX - 1; i >= 0; i--)
    {
      /* Set words 1+2 of the RXDESC */

      if (i == 0)
        val = EIM_RXDESC_WRAP_NEXT;
      else
        val = EIM_RXDESC_WRAP_FIRST;

      val |= EIM_RXDESC_OWN_ENET | EIM_RXDESC_INTRE | EIM_RXDESC_PADCRC |
             EIM_PACKET_BYTES;

      putreg32(val, desc);
      desc += sizeof(uint32_t);

      putreg32(pbuf, desc);
      desc += sizeof(uint32_t);

      putreg32(0, pbuf);
      pbuf += EIM_PACKET_BYTES;

      putreg32(0, pbuf);
      pbuf += sizeof(uint32_t); /* Ether Module's "Buffer Usage Word" */
    }

  ninfo("END desc: %08" PRIx32 " pbuf: %08" PRIx32 "\n", desc, pbuf);

  /* Save the descriptor packet size */

  putreg32(EIM_PACKET_BYTES, EIM_BUFSIZE);

  /* Set the filter mode */

#if 0
  putreg32(EIM_FILTER_UNICAST, EIM_CPU_FILTER);
#elif 0
  putreg32(EIM_FILTER_LOGICAL | EIM_FILTER_UNICAST | EIM_FILTER_MULTICAST |
           EIM_FILTER_BROADCAST, EIM_CPU_FILTER);
#else
  putreg32(EIM_FILTER_UNICAST | EIM_FILTER_MULTICAST | EIM_FILTER_BROADCAST,
           EIM_CPU_FILTER);
#endif

  /* Disable all Ethernet interrupts */

  putreg32(0x00000000, EIM_INTEN);

  /* Setup the EIM control register */

#if 1
  putreg32(EIM_CTRL_ENET0_EN | EIM_CTRL_RXENET0_EN | EIM_CTRL_TXENET0_EN |
           EIM_CTRL_RXCPU_EN | EIM_CTRL_TXCPU_EN, EIM_CTRL);
#else
  putreg32(EIM_CTRL_ENET0_EN | EIM_CTRL_ENET0_FLW | EIM_CTRL_RXENET0_EN |
           EIM_CTRL_TXENET0_EN | EIM_CTRL_RXCPU_EN | EIM_CTRL_TXCPU_EN,
           EIM_CTRL);
#endif

#if 1
  putreg32(0x00000000, EIM_MFVHI);
#else
  putreg32(0x0000ff00, EIM_MFVHI);
#endif

  putreg32(0x00000000, EIM_MFVLO);
  putreg32(0x00000000, EIM_MFMHI);
  putreg32(0x00000000, EIM_MFMLO);
  putreg32(0x00000018, EIM_RXTH);
  putreg32(0x00000000, EIM_CPU_RXREADY);

  /* Setup the ENET0 mode register */

#if 1
  putreg32(ENET_MODE_RJCT_SFE | ENET_MODE_MWIDTH | ENET_MODE_FULLDUPLEX,
           ENET0_MODE);
#else
  putreg32(ENET_MODE_RJCT_SFE | ENET_MODE_MWIDTH | ENET_MODE_HALFDUPLEX,
           ENET0_MODE);
#endif

  putreg32(0x00000000, ENET0_BOFFSEED);
  putreg32(0x00000000, ENET0_FLWPAUSE);
  putreg32(0x00000000, ENET0_FLWCONTROL);
  putreg32(0x00000000, ENET0_VTYPE);

#if 0
  putreg32(ENET_ADR_BROADCAST | ENET_ADR_PROMISCUOUS, ENET0_ADRMODE_EN);
#else
  /* The CPU port is not PROMISCUOUS, it wants a no-promiscuous address
   * match yet the SWITCH receives packets from the PROMISCUOUS ENET0
   * which routes all packets for filter matching at the CPU port which
   * then allows the s/w to see the new incoming packetes that passed
   * the filter. Here we are setting the main SWITCH closest the ether
   * wire.
   */

  putreg32(ENET_ADR_PROMISCUOUS, ENET0_ADRMODE_EN);
#endif

  putreg32(0x00000000, ENET0_DRP);
  up_mdelay(500);
}

/****************************************************************************
 * Name: c5471_reset
 *
 * Description:
 *
 ****************************************************************************/

static void c5471_reset(struct c5471_driver_s *priv)
{
#if defined(CONFIG_C5471_PHY_LU3X31T_T64)
  ninfo("EIM reset\n");
  c5471_eimreset(priv);
#endif
  ninfo("PHY init\n");
  c5471_phyinit();

  ninfo("EIM config\n");
  c5471_eimconfig(priv);
}

/****************************************************************************
 * Name: c5471_macassign
 *
 * Description:
 *    Set the mac address of our CPU ether port so that when the SWITCH
 *    receives packets from the PROMISCUOUS ENET0 it will switch them to the
 *    CPU port and cause a packet arrival event on the Switch's CPU TX queue
 *    when an address match occurs. The CPU port is not PROMISCUOUS and wants
 *    to see only packets specifically addressed to this device.
 *
 ****************************************************************************/

static void c5471_macassign(struct c5471_driver_s *priv)
{
  struct net_driver_s *dev = &priv->c_dev;
  uint8_t *mptr = dev->d_mac.ether.ether_addr_octet;
  register uint32_t tmp;

  ninfo("MAC: %0x:%0x:%0x:%0x:%0x:%0x\n",
        mptr[0], mptr[1], mptr[2], mptr[3], mptr[4], mptr[5]);

  /* Set CPU port MAC address. S/W will only see incoming packets that match
   * this destination address.
   */

  tmp = (((uint32_t)mptr[0]) << 8) | ((uint32_t)mptr[1]);
  putreg32(tmp, EIM_CPU_DAHI);

  tmp = (((uint32_t)mptr[2]) << 24) | (((uint32_t)mptr[3]) << 16) |
        (((uint32_t)mptr[4]) <<  8) |  ((uint32_t)mptr[5]);
  putreg32(tmp, EIM_CPU_DALO);

#if 0
  /* Set the ENET MAC address */

  putreg32(getreg32(EIM_CPU_DAHI), ENET0_PARHI);
  putreg32(getreg32(EIM_CPU_DALO), ENET0_PARLO);
  putreg32(getreg32(EIM_CPU_DAHI), ENET0_LARHI);
  putreg32(getreg32(EIM_CPU_DALO), ENET0_LARLO);

#else
  /* ENET MAC assignment not needed for its PROMISCUOUS mode */

  putreg32(0x00000000, ENET0_PARHI);
  putreg32(0x00000000, ENET0_PARLO);
  putreg32(0x00000000, ENET0_LARHI);
  putreg32(0x00000000, ENET0_LARLO);

#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: c5471_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the DM90x0 chip and driver */

void arm_netinitialize(void)
{
  /* Attach the IRQ to the driver */

  if (irq_attach(C5471_IRQ_ETHER, c5471_interrupt, NULL))
    {
      /* We could not attach the ISR to the ISR */

      nerr("ERROR: irq_attach() failed\n");
      return;
    }

  /* Initialize the driver structure */

  memset(g_c5471, 0,
    CONFIG_C5471_NET_NINTERFACES * sizeof(struct c5471_driver_s));
  g_c5471[0].c_dev.d_buf     = g_pktbuf[0];     /* Single packet buffer */
  g_c5471[0].c_dev.d_ifup    = c5471_ifup;      /* I/F down callback */
  g_c5471[0].c_dev.d_ifdown  = c5471_ifdown;    /* I/F up (new IP address) callback */
  g_c5471[0].c_dev.d_txavail = c5471_txavail;   /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  g_c5471[0].c_dev.d_addmac  = c5471_addmac;    /* Add multicast MAC address */
  g_c5471[0].c_dev.d_rmmac   = c5471_rmmac;     /* Remove multicast MAC address */
#endif
  g_c5471[0].c_dev.d_private = g_c5471;         /* Used to recover private state from dev */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&g_c5471[0].c_dev, NET_LL_ETHERNET);
}

#endif /* CONFIG_NET */
