/****************************************************************************
 * drivers/net/w5500.c
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
 * References:
 *   [W5500]  W5500 Datasheet, Version 1.0.9, May 2019, WIZnet Co., Ltd.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/w5500.h>
#include <nuttx/signal.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#ifdef CONFIG_NET_W5500

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ETHWORK LPWORK

/* CONFIG_NET_W5500_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_NET_W5500_NINTERFACES
#  define CONFIG_NET_W5500_NINTERFACES 1
#endif

/* TX timeout = 1 minute
 * CLK_TCK is the number of clock ticks per second
 */

#define W5500_TXTIMEOUT (60 * CLK_TCK)

/* Packet buffer size */

#define PKTBUF_SIZE (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)self->w_dev.d_buf)

/* Number of Ethernet frame transmission buffers maintained in W5500's 16 KiB
 * Tx RAM.  A maximum size is conservatively assumed per Ethernet frame in
 * order to simplify buffer management logic.
 */

#define NUM_TXBUFS ((16 * 1024) / (CONFIG_NET_ETH_PKTSIZE))

/* W5500 SPI Host Interface *************************************************/

#define W5500_BSB_COMMON_REGS         0x00u
#define W5500_BSB_SOCKET_REGS(n)      (((n & 0x7u) << 5) | 0x08u)
#define W5500_BSB_SOCKET_TX_BUFFER(n) (((n & 0x7u) << 5) | 0x10u)
#define W5500_BSB_SOCKET_RX_BUFFER(n) (((n & 0x7u) << 5) | 0x18u)

#define W5500_RWB_READ                0x00u
#define W5500_RWB_WRITE               0x04u

#define W5500_OM_DATA_LEN_VAR         0x00u
#define W5500_OM_DATA_LEN_1           0x01u
#define W5500_OM_DATA_LEN_2           0x02u
#define W5500_OM_DATA_LEN_4           0x03u

/* W5500 Register Addresses *************************************************/

/* Common Register Block */

#define W5500_MR             0x0000  /* Mode */
#define W5500_GAR0           0x0001  /* Gateway Address */
#define W5500_GAR1           0x0002
#define W5500_GAR2           0x0003
#define W5500_GAR3           0x0004
#define W5500_SUBR0          0x0005  /* Subnet Mask Address */
#define W5500_SUBR1          0x0006
#define W5500_SUBR2          0x0007
#define W5500_SUBR3          0x0008
#define W5500_SHAR0          0x0009  /* Source Hardware Address */
#define W5500_SHAR1          0x000a
#define W5500_SHAR2          0x000b
#define W5500_SHAR3          0x000c
#define W5500_SHAR4          0x000d
#define W5500_SHAR5          0x000e
#define W5500_SIPR0          0x000f  /* Source IP Address */
#define W5500_SIPR1          0x0010
#define W5500_SIPR2          0x0011
#define W5500_SIPR3          0x0012
#define W5500_INTLEVEL0      0x0013  /* Interrupt Low Level Timer */
#define W5500_INTLEVEL1      0x0014
#define W5500_IR             0x0015  /* Interrupt */
#define W5500_IMR            0x0016  /* Interrupt Mask */
#define W5500_SIR            0x0017  /* Socket Interrupt */
#define W5500_SIMR           0x0018  /* Socket Interrupt Mask */
#define W5500_RTR0           0x0019  /* Retry Time */
#define W5500_RTR1           0x001a
#define W5500_RCR            0x001b  /* Retry Count */
#define W5500_PTIMER         0x001c  /* PPP LCP Request Timer */
#define W5500_PMAGIC         0x001d  /* PPP LCP Magic number */
#define W5500_PHAR0          0x001e  /* PPP Destination MAC Address */
#define W5500_PHAR1          0x001f
#define W5500_PHAR2          0x0020
#define W5500_PHAR3          0x0021
#define W5500_PHAR4          0x0022
#define W5500_PHAR5          0x0023
#define W5500_PSID0          0x0024  /* PPP Session Identification */
#define W5500_PSID1          0x0025
#define W5500_PMRU0          0x0026  /* PPP Maximum Segment Size */
#define W5500_PMRU1          0x0027
#define W5500_UIPR0          0x0028  /* Unreachable IP address */
#define W5500_UIPR1          0x0029
#define W5500_UIPR2          0x002a
#define W5500_UIPR3          0x002b
#define W5500_UPORTR0        0x002c  /* Unreachable Port */
#define W5500_UPORTR1        0x002d
#define W5500_PHYCFGR        0x002e  /* PHY Configuration */
                                     /* 0x002f-0x0038: Reserved */
#define W5500_VERSIONR       0x0039  /* Chip version */
                                     /* 0x003a-0xffff: Reserved */

/* Socket Register Block */

#define W5500_SN_MR          0x0000  /* Socket n Mode */
#define W5500_SN_CR          0x0001  /* Socket n Command */
#define W5500_SN_IR          0x0002  /* Socket n Interrupt */
#define W5500_SN_SR          0x0003  /* Socket n Status */
#define W5500_SN_PORT0       0x0004  /* Socket n Source Port */
#define W5500_SN_PORT1       0x0005
#define W5500_SN_DHAR0       0x0006  /* Socket n Destination Hardware Address */
#define W5500_SN_DHAR1       0x0007
#define W5500_SN_DHAR2       0x0008
#define W5500_SN_DHAR3       0x0009
#define W5500_SN_DHAR4       0x000a
#define W5500_SN_DHAR5       0x000b
#define W5500_SN_DIPR0       0x000c  /* Socket n Destination IP Address */
#define W5500_SN_DIPR1       0x000d
#define W5500_SN_DIPR2       0x000e
#define W5500_SN_DIPR3       0x000f
#define W5500_SN_DPORT0      0x0010  /* Socket n Destination Port */
#define W5500_SN_DPORT1      0x0011
#define W5500_SN_MSSR0       0x0012  /* Socket n Maximum Segment Size */
#define W5500_SN_MSSR1       0x0013
                                     /* 0x0014: Reserved */
#define W5500_SN_TOS         0x0015  /* Socket n IP TOS */
#define W5500_SN_TTL         0x0016  /* Socket n IP TTL */
                                     /* 0x0017-0x001d: Reserved */
#define W5500_SN_RXBUF_SIZE  0x001e  /* Socket n Receive Buffer Size */
#define W5500_SN_TXBUF_SIZE  0x001f  /* Socket n Transmit Buffer Size */
#define W5500_SN_TX_FSR0     0x0020  /* Socket n TX Free Size */
#define W5500_SN_TX_FSR1     0x0021
#define W5500_SN_TX_RD0      0x0022  /* Socket n TX Read Pointer */
#define W5500_SN_TX_RD1      0x0023
#define W5500_SN_TX_WR0      0x0024  /* Socket n TX Write Pointer */
#define W5500_SN_TX_WR1      0x0025
#define W5500_SN_RX_RSR0     0x0026  /* Socket n RX Received Size */
#define W5500_SN_RX_RSR1     0x0027
#define W5500_SN_RX_RD0      0x0028  /* Socket n RX Read Pointer */
#define W5500_SN_RX_RD1      0x0029
#define W5500_SN_RX_WR0      0x002a  /* Socket n RX Write Pointer */
#define W5500_SN_RX_WR1      0x002b
#define W5500_SN_IMR         0x002c  /* Socket n Interrupt Mask */
#define W5500_SN_FRAG0       0x002d  /* Socket n Fragment Offset in IP header */
#define W5500_SN_FRAG1       0x002e
#define W5500_SN_KPALVTR     0x002f  /* Keep alive timer */
                                     /* 0x0030-0xffff: Reserved */

/* W5500 Register Bitfield Definitions **************************************/

/* Common Register Block */

/* Mode Register (MR) */

#define MR_FARP              (1 << 1)  /* Bit 1: Force ARP */
#define MR_PPPOE             (1 << 3)  /* Bit 3: PPPoE Mode */
#define MR_PB                (1 << 4)  /* Bit 4: Ping Block Mode */
#define MR_WOL               (1 << 5)  /* Bit 5: Wake on LAN */
#define MR_RST               (1 << 7)  /* Bit 7: Reset registers */

/* Interrupt Register (IR), Interrupt Mask Register (IMR) */

#define INT_MP               (1 << 4)  /* Bit 4:  Magic Packet */
#define INT_PPPOE            (1 << 5)  /* Bit 5:  PPPoE Connection Close */
#define INT_UNREACH          (1 << 6)  /* Bit 6:  Destination unreachable */
#define INT_CONFLICT         (1 << 7)  /* Bit 7:  IP Conflict */

/* Socket Interrupt Register (SIR) */

#define SIR(n)               (1 << (n))

/* Socket Interrupt Mask Register (SIMR)) */

#define SIMR(n)              (1 << (n))

/* PHY Configuration Register (PHYCFGR) */

#define PHYCFGR_LNK          (1 << 0)  /* Bit 0:  Link Status */
#define PHYCFGR_SPD          (1 << 1)  /* Bit 1:  Speed Status */
#define PHYCFGR_DPX          (1 << 2)  /* Bit 2:  Duplex Status */
#define PHYCFGR_OPMDC_SHIFT  (3)       /* Bits 3-5: Operation Mode Configuration */
#define PHYCFGR_OPMDC_MASK   (7 << PHYCFGR_OPMDC_SHIFT)
#  define PHYCFGR_OPMDC_10BT_HD_NAN  (0 << PHYCFGR_OPMDC_SHIFT) /* 10BT Half-duplex */
#  define PHYCFGR_OPMDC_10BT_HFD_NAN (1 << PHYCFGR_OPMDC_SHIFT) /* 10BT Full-duplex */
#  define PHYCFGR_OPMDC_100BT_HD_NAN (2 << PHYCFGR_OPMDC_SHIFT) /* 100BT Half-duplex */
#  define PHYCFGR_OPMDC_10BT_FD_NAN  (3 << PHYCFGR_OPMDC_SHIFT) /* 100BT Full-duplex,
                                                                 * Auto-negotiation */
#  define PHYCFGR_OPMDC_100BT_HD_AN  (4 << PHYCFGR_OPMDC_SHIFT) /* 100BT Half-duplex,
                                                                 * Auto-negotiation */
#  define PHYCFGR_OPMDC_POWER_DOWN   (6 << PHYCFGR_OPMDC_SHIFT) /* Power Down mode */
#  define PHYCFGR_OPMDC_ALLCAP_AN    (7 << PHYCFGR_OPMDC_SHIFT) /* All capable,
                                                                 * Auto-negotiation */

#define PHYCFGR_OPMD         (1 << 6)  /* Bit 6:  Configure PHY Operation Mode */
#define PHYCFGR_RST          (1 << 7)  /* Bit 7:  Reset */

/* Socket Register Block */

/* Socket n Mode Register (SN_MR) */

#define SN_MR_PROTOCOL_SHIFT (0)       /* Bits 0-3:  Protocol */
#define SN_MR_PROTOCOL_MASK  (15 << SN_MR_PROTOCOL_SHIFT)
#  define SN_MR_P0           (1 << (SN_MR_PROTOCOL_SHIFT + 0))
#  define SN_MR_P1           (1 << (SN_MR_PROTOCOL_SHIFT + 1))
#  define SN_MR_P2           (1 << (SN_MR_PROTOCOL_SHIFT + 2))
#  define SN_MR_P3           (1 << (SN_MR_PROTOCOL_SHIFT + 3))
#  define SM_MR_CLOSED       0
#  define SM_MR_TCP          SN_MR_P0
#  define SM_MR_UDP          SN_MR_P1
#  define SM_MR_MACRAW       SN_MR_P2
#define SN_MR_UCASTB         (1 << 4)  /* Bit 4:  UNICAST Blocking in UDP mode */
#define SN_MR_MIP6B          (1 << 4)  /* Bit 4:  IPv6 packet Blocking in MACRAW mode */
#define SN_MR_ND             (1 << 5)  /* Bit 5:  Use No Delayed ACK */
#define SN_MR_MC             (1 << 5)  /* Bit 5:  Multicast */
#define SN_MR_MMB            (1 << 5)  /* Bit 5:  Multicast Blocking in MACRAW mode */
#define SN_MR_BCASTB         (1 << 6)  /* Bit 6:  Broadcast Blocking in MACRAW and
                                        *         UDP mode */
#define SN_MR_MULTI          (1 << 7)  /* Bit 7:  Multicasting in UDP mode */
#define SN_MR_MFEN           (1 << 7)  /* Bit 7:  MAC Filter Enable in MACRAW mode */

/* Socket n Command Register (SN_CR) */

#define SN_CR_OPEN           0x01      /* Socket n is initialized and opened according
                                        * to the protocol selected in SN_MR */
#define SN_CR_LISTEN         0x02      /* Socket n operates as a 'TCP server' and waits
                                        * for connection request from any 'TCP client' */
#define SN_CR_CONNECT        0x04      /* 'TCP client' connection request */
#define SN_CR_DISCON         0x08      /* TCP disconnection request */
#define SN_CR_CLOSE          0x10      /* Close socket n */
#define SN_CR_SEND           0x20      /* Transmit all data in Socket n TX buffer */
#define SN_CR_SEND_MAC       0x21      /* Transmit all UDP data (no ARP) */
#define SN_CR_SEND_KEEP      0x22      /* Send TCP keep-alive packet */
#define SN_CR_RECV           0x40      /* Complete received data in Socket n RX buffer */

/* Socket n Interrupt Register (SN_IR) and
 * Socket n Interrupt Mask Register (SN_IMR)
 */

#define SN_INT_CON           (1 << 0)  /* Bit 0:  Connection with peer successful */
#define SN_INT_DISCON        (1 << 1)  /* Bit 1:  FIN or FIN/ACK received from peer */
#define SN_INT_RECV          (1 << 2)  /* Bit 2:  Data received from peer */
#define SN_INT_TIMEOUT       (1 << 3)  /* Bit 3:  ARP or TCP timeout */
#define SN_INT_SEND_OK       (1 << 4)  /* Bit 4:  SEND command completed */

/* Socket n Status Register (SN_SR) */

#define SN_SR_SOCK_CLOSED      0x00
#define SN_SR_SOCK_INIT        0x13
#define SN_SR_SOCK_LISTEN      0x14
#define SN_SR_SOCK_ESTABLISHED 0x17
#define SN_SR_SOCK_CLOSE_WAIT  0x1c
#define SN_SR_SOCK_UDP         0x22
#define SN_SR_SOCK_MACRAW      0x42

#define SN_SR_SOCK_SYNSENT     0x15    /* Transitional status */
#define SN_SR_SOCK_SYNRECV     0x16
#define SN_SR_SOCK_FIN_WAIT    0x18
#define SN_SR_SOCK_CLOSING     0x1a
#define SN_SR_SOCK_TIME_WAIT   0x1b
#define SN_SR_SOCK_LAST_ACK    0x1d

/* Socket n RX Buffer Size Register (SN_RXBUF) */

#define SN_RXBUF_0KB          0
#define SN_RXBUF_1KB          1
#define SN_RXBUF_2KB          2
#define SN_RXBUF_4KB          4
#define SN_RXBUF_8KB          5
#define SN_RXBUF_16KB         16

/* Socket n TX Buffer Size Register (SN_TXBUF) */

#define SN_TXBUF_0KB          0
#define SN_TXBUF_1KB          1
#define SN_TXBUF_2KB          2
#define SN_TXBUF_4KB          4
#define SN_TXBUF_8KB          5
#define SN_TXBUF_16KB         16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The w5500_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct w5500_driver_s
{
  bool w_bifup;               /* true:ifup false:ifdown */
  struct wdog_s w_txtimeout;  /* TX timeout timer */
  struct work_s w_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s w_pollwork;   /* For deferring poll work to the work queue */

  /* Ethernet frame transmission buffer management */

  uint16_t txbuf_offset[NUM_TXBUFS + 1];
  uint8_t  txbuf_rdptr;
  uint8_t  txbuf_wrptr;

  /* The hardware interconnect to the W5500 chip */

  FAR struct spi_dev_s           *spi_dev; /* SPI hardware access    */
  FAR const struct w5500_lower_s *lower;   /* Low-level MCU specific */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s w_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint16_t g_pktbuf[CONFIG_NET_W5500_NINTERFACES]
                        [(PKTBUF_SIZE + 1) / 2];

/* Driver state structure */

static struct w5500_driver_s g_w5500[CONFIG_NET_W5500_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static void w5500_transmit(FAR struct w5500_driver_s *priv);
static int  w5500_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void w5500_reply(FAR struct w5500_driver_s *priv);
static void w5500_receive(FAR struct w5500_driver_s *priv);
static void w5500_txdone(FAR struct w5500_driver_s *priv);

static void w5500_interrupt_work(FAR void *arg);
static int  w5500_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void w5500_txtimeout_work(FAR void *arg);
static void w5500_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  w5500_ifup(FAR struct net_driver_s *dev);
static int  w5500_ifdown(FAR struct net_driver_s *dev);

static void w5500_txavail_work(FAR void *arg);
static int  w5500_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  w5500_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int  w5500_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void w5500_ipv6multicast(FAR struct w5500_driver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  w5500_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Hardware interface to W5500 **********************************************/

/****************************************************************************
 * Name: w5500_reset
 *
 * Description:
 *   Apply the hardware reset sequence to the W5500 (See [W5500], section
 *   5.5.1 Reset Timing).  Optinonally, keep hardware reset asserted.
 *
 * Input Parameters:
 *   self  - The respective w5500 device
 *   keep  - Whether the hardware reset shall be left asserted.
 *
 * Assumptions:
 *   This function must only be called in interrupt context, if argument
 *   keep is set to false (otherwise it sleeps, which is not allowed in
 *   interrupt context).
 *
 ****************************************************************************/

static void w5500_reset(FAR struct w5500_driver_s *self, bool keep)
{
  self->lower->reset(self->lower, true);

  if (!keep)
    {
      nxsig_usleep(500);   /* [W5500]: T_RC (Reset Cycle Time) min 500 us   */

      self->lower->reset(self->lower, false);

      nxsig_usleep(1000);  /* [W5500]: T_PL (RSTn to internal PLL lock) 1ms */
    }
}

/****************************************************************************
 * Name: w5500_lock
 *
 * Description:
 *   Acquire exclusive access to the W5500's SPI bus and configure it
 *   accordingly.
 *
 * Input Parameters:
 *   self  - The respective w5500 device
 *
 ****************************************************************************/

static void w5500_lock(FAR struct w5500_driver_s *self)
{
  int ret;

  ret = SPI_LOCK(self->spi_dev, true);
  DEBUGASSERT(ret == OK);

  SPI_SETMODE(self->spi_dev, self->lower->mode);
  SPI_SETBITS(self->spi_dev, 8);
  SPI_HWFEATURES(self->spi_dev, 0);
  SPI_SETFREQUENCY(self->spi_dev, self->lower->frequency);
  SPI_SELECT(self->spi_dev, SPIDEV_ETHERNET(self->lower->spidevid), true);
}

/****************************************************************************
 * Name: w5500_unlock
 *
 * Description:
 *   Relinquish exclusive access to the W5500's SPI bus
 *
 * Input Parameters:
 *   self  - The respective w5500 device
 *
 ****************************************************************************/

static void w5500_unlock(FAR struct w5500_driver_s *self)
{
  int ret;

  SPI_SELECT(self->spi_dev, SPIDEV_ETHERNET(self->lower->spidevid), false);
  ret = SPI_LOCK(self->spi_dev, false);
  DEBUGASSERT(ret == OK);
}

/****************************************************************************
 * Name: w5500_read
 *
 * Description:
 *   Read a number of bytes from one of the W5500's register or buffer
 *   blocks.
 *
 * Input Parameters:
 *   self              - The respective w5500 device
 *   block_select_bits - Register or buffer block to read from
 *   offset            - The offset address within the block
 *   buffer            - Buffer to copy read data into
 *   len               - Number of bytes to read from block
 *
 ****************************************************************************/

static void w5500_read(FAR struct w5500_driver_s *self,
                       uint8_t                   block_select_bits,
                       uint16_t                  offset,
                       FAR void                  *buffer,
                       uint16_t                  len)
{
  uint8_t addr_cntl[3];

  addr_cntl[0] = (uint8_t)(offset >> 8);
  addr_cntl[1] = (uint8_t)offset;
  addr_cntl[2] = block_select_bits | W5500_RWB_READ | W5500_OM_DATA_LEN_VAR;

  w5500_lock(self);
  SPI_SNDBLOCK(self->spi_dev, addr_cntl, sizeof(addr_cntl));
  SPI_RECVBLOCK(self->spi_dev, buffer, len);
  w5500_unlock(self);
}

/****************************************************************************
 * Name: w5500_write
 *
 * Description:
 *   Write a number of bytes to one of the W5500's register or buffer
 *   blocks.
 *
 * Input Parameters:
 *   self              - The respective w5500 device
 *   block_select_bits - Register or buffer block to read from
 *   offset            - The offset address within the block
 *   Data              - Data to write to block
 *   len               - Number of bytes to write to block
 *
 ****************************************************************************/

static void w5500_write(FAR struct w5500_driver_s *self,
                        uint8_t                   block_select_bits,
                        off_t                     offset,
                        FAR const void            *data,
                        size_t                    len)
{
  uint8_t addr_cntl[3];

  addr_cntl[0] = (uint8_t)(offset >> 8);
  addr_cntl[1] = (uint8_t)offset;
  addr_cntl[2] = block_select_bits | W5500_RWB_WRITE | W5500_OM_DATA_LEN_VAR;

  w5500_lock(self);
  SPI_SNDBLOCK(self->spi_dev, addr_cntl, sizeof(addr_cntl));
  SPI_SNDBLOCK(self->spi_dev, data, len);
  w5500_unlock(self);
}

/****************************************************************************
 * Name: w5500_read8
 *
 * Description:
 *   Read a single byte from one of the W5500's register or buffer blocks.
 *
 * Input Parameters:
 *   self              - The respective w5500 device
 *   block_select_bits - Register or buffer block to read from
 *   offset            - The offset address within the block
 *
 * Returned Value:
 *   The byte read
 *
 ****************************************************************************/

static uint8_t w5500_read8(FAR struct w5500_driver_s *self,
                           uint8_t                   block_select_bits,
                           uint16_t                  offset)
{
  uint8_t value;

  w5500_read(self,
             block_select_bits,
             offset,
             &value,
             sizeof(value));

  return value;
}

/****************************************************************************
 * Name: w5500_write8
 *
 * Description:
 *   Write a single byte to one of the W5500's register or buffer blocks.
 *
 * Input Parameters:
 *   self              - The respective w5500 device
 *   block_select_bits - Register or buffer block to read from
 *   offset            - The offset address within the block
 *   value             - The byte value to write
 *
 ****************************************************************************/

static void w5500_write8(FAR struct w5500_driver_s *self,
                         uint8_t                   block_select_bits,
                         uint16_t                  offset,
                         uint8_t                   value)
{
  w5500_write(self,
              block_select_bits,
              offset,
              &value,
              sizeof(value));
}

/****************************************************************************
 * Name: w5500_read16
 *
 * Description:
 *   Read a two byte value from one of the W5500's register or buffer blocks.
 *
 * Input Parameters:
 *   self              - The respective w5500 device
 *   block_select_bits - Register or buffer block to read from
 *   offset            - The offset address within the block
 *
 * Returned Value:
 *   The two byte value read in host byte order.
 *
 ****************************************************************************/

static uint16_t w5500_read16(FAR struct w5500_driver_s *self,
                             uint8_t                   block_select_bits,
                             uint16_t                  offset)
{
  uint16_t value;

  w5500_read(self,
             block_select_bits,
             offset,
             &value,
             sizeof(value));

  return NTOHS(value);
}

/****************************************************************************
 * Name: w5500_write16
 *
 * Description:
 *   Write a two byte value to one of the W5500's register or buffer blocks.
 *
 * Input Parameters:
 *   self              - The respective w5500 device
 *   block_select_bits - Register or buffer block to read from
 *   offset            - The offset address within the block
 *   value             - The two byte value to write in host byte order.
 *
 ****************************************************************************/

static void w5500_write16(FAR struct w5500_driver_s *self,
                          uint8_t                   block_select_bits,
                          uint16_t                  offset,
                          uint16_t                  value)
{
  value = HTONS(value);

  w5500_write(self,
              block_select_bits,
              offset,
              &value,
              sizeof(value));
}

/****************************************************************************
 * Name: w5500_read16_atomic
 *
 * Description:
 *   Read a two-byte value that is concurrently updated by the W5500 hardware
 *   in a safe fashion.  In [W5500] it is recommended to read the 16 bits
 *   multiple times until one gets the same value twice in a row.
 *
 * Input Parameters:
 *   self               - The respective w5500 device
 *   block_select_bits  - Register block to read from  (See [W5500], section
 *                          2.2.2 Control Phase)
 *   offset             - Offset address of the register to read.
 *   value              - The 16-bit value read in host byte-order.
 *
 * Returned Value:
 *   OK in case of success.  A negated errno value in case of failure.
 *
 ****************************************************************************/

static int w5500_read16_atomic(FAR struct w5500_driver_s *self,
                               uint8_t                   block_select_bits,
                               uint16_t                  offset,
                               FAR uint16_t              *value)
{
  int i;

  *value = w5500_read16(self, block_select_bits, offset);

  for (i = 0; i < 100; i++)
    {
      uint16_t temp;

      temp = w5500_read16(self, block_select_bits, offset);

      if (*value == temp)
        {
          return OK;
        }

      *value = temp;
    }

  nerr("Failed to get consistent value.\n");

  return -EIO;
}

/* Ethernet frame transmission buffer management ****************************/

/****************************************************************************
 * Name: w5500_txbuf_reset
 *
 * Description:
 *   Reset state that manages the storage of multiple outgoing Ethernet
 *   frames in W5500's 16KiB Tx RAM.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 ****************************************************************************/

static void w5500_txbuf_reset(FAR struct w5500_driver_s *self)
{
  memset(self->txbuf_offset, 0, sizeof(self->txbuf_offset));
  self->txbuf_rdptr = 0;
  self->txbuf_wrptr = 0;
}

/****************************************************************************
 * Name: w5500_txbuf_numfree
 *
 * Description:
 *   Return the number of Ethernet frames that can still be stored in W5500's
 *   16KiB Tx RAM.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 * Returned Value:
 *   The number of Ethernet frames that can still be stored.
 *
 ****************************************************************************/

static int w5500_txbuf_numfree(FAR struct w5500_driver_s *self)
{
  if (self->txbuf_wrptr >= self->txbuf_rdptr)
    {
      return NUM_TXBUFS - (self->txbuf_wrptr - self->txbuf_rdptr);
    }
  else
    {
      return self->txbuf_rdptr - self->txbuf_wrptr - 1;
    }
}

/****************************************************************************
 * Name: w5500_txbuf_numpending
 *
 * Description:
 *   Return the number of Ethernet frames that are still pending for trans-
 *   mission from W5500's 16KiB Tx RAM.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 * Returned Value:
 *   The number of Ethernet frames that are pending for transmission.
 *
 ****************************************************************************/

static int w5500_txbuf_numpending(FAR struct w5500_driver_s *self)
{
  return NUM_TXBUFS - w5500_txbuf_numfree(self);
}

/****************************************************************************
 * Name: w5500_txbuf_copy
 *
 * Description:
 *   Copy an Ethernet frame from the socket device's buffer to the W5500's
 *   16KiB Tx RAM.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 * Returned Value:
 *   The byte offset of the first byte after the frame that was copied into
 *   W5500's 16KiB Tx RAM.
 *
 ****************************************************************************/

static uint16_t w5500_txbuf_copy(FAR struct w5500_driver_s *self)
{
  uint16_t offset;

  DEBUGASSERT(w5500_txbuf_numfree(self) > 0);

  offset = self->txbuf_offset[self->txbuf_wrptr];

  w5500_write(self,
              W5500_BSB_SOCKET_TX_BUFFER(0),
              offset,
              self->w_dev.d_buf,
              self->w_dev.d_len);

  self->txbuf_wrptr = (self->txbuf_wrptr + 1) % (NUM_TXBUFS + 1);
  self->txbuf_offset[self->txbuf_wrptr] = offset + self->w_dev.d_len;

  return self->txbuf_offset[self->txbuf_wrptr];
}

/****************************************************************************
 * Name: w5500_txbuf_next
 *
 * Description:
 *   Release storage for an Ethernet frame that has been successfully
 *   transmitted.  Also triggers transmission of the next Ethernet frame,
 *   if applicable.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 * Returned Value:
 *   Whether transmission of another Ethernet frame was triggered.
 *
 ****************************************************************************/

static bool w5500_txbuf_next(FAR struct w5500_driver_s *self)
{
  uint16_t offset;

  DEBUGASSERT(w5500_txbuf_numpending(self));

  self->txbuf_rdptr = (self->txbuf_rdptr + 1) % (NUM_TXBUFS + 1);

  offset = w5500_read16(self,
                        W5500_BSB_SOCKET_REGS(0),
                        W5500_SN_TX_RD0);

  DEBUGASSERT(self->txbuf_offset[self->txbuf_rdptr] == offset);

  if (!w5500_txbuf_numpending(self))
    {
      return false;
    }

  offset = self->txbuf_offset[(self->txbuf_rdptr + 1) % (NUM_TXBUFS + 1)];

  w5500_write16(self,
                W5500_BSB_SOCKET_REGS(0),
                W5500_SN_TX_WR0,
                offset);

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_CR,
               SN_CR_SEND);

  /* (Re-)start the TX timeout watchdog timer */

  wd_start(&self->w_txtimeout,
           W5500_TXTIMEOUT,
           w5500_txtimeout_expiry,
           (wdparm_t)self);

  return true;
}

/****************************************************************************
 * Name: w5500_fence
 *
 * Description:
 *   Put the W5500 into reset and disable respective interrupt handling.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 ****************************************************************************/

static void w5500_fence(FAR struct w5500_driver_s *self)
{
  self->lower->enable(self->lower, false);
  w5500_reset(self, true);  /* Reset and keep reset asserted */
  self->w_bifup = false;
}

/****************************************************************************
 * Name: w5500_unfence
 *
 * Description:
 *   Release W5500 from reset, initialize it and wait up to ten seconds
 *   for link up.
 *
 * Input Parameters:
 *   self - The respective w5500 device
 *
 * Returned Value:
 *   OK in case of success.  A negated errno value in case of failure, in
 *   which case the W5500 is fenced again.
 *
 ****************************************************************************/

static int w5500_unfence(FAR struct w5500_driver_s *self)
{
  uint8_t value;
  int i;

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  w5500_reset(self, false);  /* Reset sequence and keep reset de-asserted */

  /* Set the Ethernet interface's MAC address */

  w5500_write(self,
              W5500_BSB_COMMON_REGS,
              W5500_SHAR0, /* Source Hardware Address Register */
              self->w_dev.d_mac.ether.ether_addr_octet,
              sizeof(self->w_dev.d_mac.ether.ether_addr_octet));

  /* Configure socket 0 for raw MAC access with MAC filtering enabled. */

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_MR,
               SM_MR_MACRAW | SN_MR_MFEN);

  /* Allocate all TX and RX buffer space to socket 0 ... */

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_RXBUF_SIZE,
               SN_RXBUF_16KB);

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_TXBUF_SIZE,
               SN_TXBUF_16KB);

  /* ... and none to sockets 1 to 7. */

  for (i = 1; i < 8; i++)
    {
      w5500_write8(self,
                   W5500_BSB_SOCKET_REGS(i),
                   W5500_SN_RXBUF_SIZE,
                   SN_RXBUF_0KB);

      w5500_write8(self,
                   W5500_BSB_SOCKET_REGS(i),
                   W5500_SN_TXBUF_SIZE,
                   SN_TXBUF_0KB);
    }

  /* Enable RECV interrupts on socket 0 (SEND_OK interrupts will only be
   * enabled as long as a transmission is in progress).
   */

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_IMR,
               SN_INT_RECV);

  /* Enable interrupts on socket 0 */

  w5500_write8(self,
               W5500_BSB_COMMON_REGS,
               W5500_SIMR, /* Socket Interrupt Mask Register */
               SIMR(0));

  /* Open socket 0 */

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_CR, /* Control Register */
               SN_CR_OPEN);

  /* Check whether socket 0 is open in MACRAW mode. */

  value = w5500_read8(self,
                      W5500_BSB_SOCKET_REGS(0),
                      W5500_SN_SR);

  if (value != SN_SR_SOCK_MACRAW)
    {
      nerr("Unexpected status: %02" PRIx8 "\n", value);
      goto error;
    }

  /* Reset Tx buffer management state. */

  w5500_txbuf_reset(self);

  /* Wait up to 10 seconds for link-up */

  value = w5500_read8(self,
                      W5500_BSB_COMMON_REGS,
                      W5500_PHYCFGR);

  for (i = 0; (i < 100) && !(value & PHYCFGR_LNK); i++)
    {
      value = w5500_read8(self,
                          W5500_BSB_COMMON_REGS,
                          W5500_PHYCFGR);

      nxsig_usleep(100000); /* 100 ms x 100 = 10 sec */
    }

  if (value & PHYCFGR_LNK)
    {
      ninfo("Link up (%d Mbps / %s duplex)\n",
            (value & PHYCFGR_SPD) ? 100 : 10,
            (value & PHYCFGR_DPX) ? "full" : "half");
    }
  else
    {
      nwarn("Link still down.  Cable plugged?\n");
      goto error;
    }

  return OK;

error:
  w5500_fence(self);
  return -EIO;
}

/****************************************************************************
 * Name: w5500_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void w5500_transmit(FAR struct w5500_driver_s *self)
{
  uint16_t offset;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  if (!w5500_txbuf_numfree(self))
    {
      ninfo("Dropping Tx packet due to no buffer available.\n");
      NETDEV_TXERRORS(self->w_dev);
      return;
    }

  /* Increment statistics */

  NETDEV_TXPACKETS(self->w_dev);

  /* Copy packet data to TX buffer */

  offset = w5500_txbuf_copy(self);

  /* If there have not been any Tx buffers in use this means we need to start
   * transmission.  Otherwise, this is done either in w5500_txdone or in
   * w5500_txtimeout_work.
   */

  if (w5500_txbuf_numpending(self) == 1)
    {
      /* Set TX Write Pointer to indicate packet length */

      w5500_write16(self,
                    W5500_BSB_SOCKET_REGS(0),
                    W5500_SN_TX_WR0,
                    offset);

      /* Enable Tx interrupts (Rx ones are always enabled). */

      w5500_write8(self,
                   W5500_BSB_SOCKET_REGS(0),
                   W5500_SN_IMR,
                   SN_INT_RECV | SN_INT_SEND_OK);

      /* Send the packet */

      w5500_write8(self,
                   W5500_BSB_SOCKET_REGS(0),
                   W5500_SN_CR, /* Control Register */
                   SN_CR_SEND);

      /* Setup the TX timeout watchdog (perhaps restarting the timer) */

      wd_start(&self->w_txtimeout, W5500_TXTIMEOUT,
               w5500_txtimeout_expiry, (wdparm_t)self);
    }

#ifdef CONFIG_DEBUG_NET_INFO
  ninfodumpbuffer("Transmitted:", self->w_dev.d_buf, self->w_dev.d_len);
#endif
}

/****************************************************************************
 * Name: w5500_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int w5500_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct w5500_driver_s *self =
    (FAR struct w5500_driver_s *)dev->d_private;

  /* Send the packet */

  w5500_transmit(self);

  /* Check if there is room in the device to hold another packet.
   * If not, return a non-zero value to terminate the poll.
   */

  return !w5500_txbuf_numfree(self);
}

/****************************************************************************
 * Name: w5500_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return with an outgoing packet.  This function checks for that case
 *   and performs the transmission if necessary.
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void w5500_reply(FAR struct w5500_driver_s *self)
{
  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (self->w_dev.d_len > 0)
    {
      /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv4(self->w_dev.d_flags))
        {
          arp_out(&self->w_dev);
        }
#endif

#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv6(self->w_dev.d_flags))
        {
          neighbor_out(&self->w_dev);
        }
#endif

      /* And send the packet */

      w5500_transmit(self);
    }
}

/****************************************************************************
 * Name: w5500_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void w5500_receive(FAR struct w5500_driver_s *self)
{
  do
    {
      uint16_t s0_rx_rd;
      uint16_t s0_rx_rsr;
      uint16_t pktlen;
      int ret;

      /* Check if the packet is a valid size for the network buffer
       * configuration.
       */

      ret = w5500_read16_atomic(self,
                                W5500_BSB_SOCKET_REGS(0),
                                W5500_SN_RX_RSR0,
                                &s0_rx_rsr);
      if (ret != OK)
        {
          goto error;
        }

      if (s0_rx_rsr == 0)
        {
          ninfo("No data left to read.  We are done.\n");
          break;
        }

      /* The W5500 prepends each packet with a 2-byte length field in
       * network byte order.  The length value includes the length field
       * itself.  At least this 2-byte packet length must be available.
       */

      if (s0_rx_rsr < sizeof(pktlen))
        {
          nerr("Received size too small. S0_RX_RSR %"PRIu16"\n", s0_rx_rsr);
          goto error;
        }

      /* Get the Socket 0 RX Read Pointer. */

      s0_rx_rd = w5500_read16(self,
                              W5500_BSB_SOCKET_REGS(0),
                              W5500_SN_RX_RD0);

      /* Read 16-bit length field. */

      pktlen = w5500_read16(self,
                            W5500_BSB_SOCKET_RX_BUFFER(0),
                            s0_rx_rd);

      if (pktlen > s0_rx_rsr)
        {
          nerr("Incomplete packet: pktlen %"PRIu16", S0_RX_RSR %"PRIu16"\n",
               pktlen,
               s0_rx_rd);

          goto error;
        }

      if (pktlen < s0_rx_rsr)
        {
          ninfo("More than one packet in RX buffer. "
                "pktlen %"PRIu16", S0_RX_RSR %"PRIu16"\n",
                pktlen,
                s0_rx_rsr);
        }

      self->w_dev.d_len = pktlen - sizeof(pktlen);

      /* Copy the data data from the hardware to priv->w_dev.d_buf.  Set
       * amount of data in priv->w_dev.d_len
       */

      if (self->w_dev.d_len <= CONFIG_NET_ETH_PKTSIZE)
        {
          w5500_read(self,
                     W5500_BSB_SOCKET_RX_BUFFER(0),
                     s0_rx_rd + sizeof(pktlen),
                     self->w_dev.d_buf,
                     self->w_dev.d_len);
        }

      /* Acknowledge data reception to W5500 */

      w5500_write16(self,
                    W5500_BSB_SOCKET_REGS(0),
                    W5500_SN_RX_RD0,
                    s0_rx_rd + pktlen);

      w5500_write8(self,
                   W5500_BSB_SOCKET_REGS(0),
                   W5500_SN_CR,
                   SN_CR_RECV);

      /* Check for errors and update statistics */

      if (self->w_dev.d_len > CONFIG_NET_ETH_PKTSIZE ||
          self->w_dev.d_len < ETH_HDRLEN)
        {
          nerr("Bad packet size dropped (%"PRIu16")\n", self->w_dev.d_len);
          self->w_dev.d_len = 0;
          NETDEV_RXERRORS(&priv->dev);
          continue;
        }

#ifdef CONFIG_DEBUG_NET_INFO
      ninfodumpbuffer("Received Packet:",
                      self->w_dev.d_buf,
                      self->w_dev.d_len);
#endif

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(&self->w_dev);
#endif

#ifdef CONFIG_NET_IPv4
      /* Check for an IPv4 packet */

      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&self->w_dev);

          /* Handle ARP on input, then dispatch IPv4 packet to the network
           * layer.
           */

          arp_ipin(&self->w_dev);
          ipv4_input(&self->w_dev);

          /* Check for a reply to the IPv4 packet */

          w5500_reply(self);
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      /* Check for an IPv6 packet */

      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(&self->w_dev);

          /* Dispatch IPv6 packet to the network layer */

          ipv6_input(&self->w_dev);

          /* Check for a reply to the IPv6 packet */

          w5500_reply(self);
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      /* Check for an ARP packet */

      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Dispatch ARP packet to the network layer */

          arp_arpin(&self->w_dev);
          NETDEV_RXARP(&self->w_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value
           * > 0.
           */

          if (self->w_dev.d_len > 0)
            {
              w5500_transmit(self);
            }
        }
      else
#endif
        {
          ninfo("Dropped frame\n");

          NETDEV_RXDROPPED(&self->w_dev);
        }
    }
  while (true); /* While there are more packets to be processed */

  return;

error:
  w5500_fence(self);
}

/****************************************************************************
 * Name: w5500_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void w5500_txdone(FAR struct w5500_driver_s *self)
{
  /* Check for errors and update statistics */

  NETDEV_TXDONE(self->w_dev);

  /* Check if there are pending transmissions. */

  if (!w5500_txbuf_next(self))
    {
      ninfo("No further transmissions pending.\n");

      /* If no further transmissions are pending, then cancel the TX timeout
       * and disable further Tx interrupts.
       */

      wd_cancel(&self->w_txtimeout);

      /* And disable further TX interrupts. */

      w5500_write8(self,
                   W5500_BSB_SOCKET_REGS(0),
                   W5500_SN_IMR, SN_INT_RECV);
    }

  /* In any event, poll the network for new TX data */

  devif_poll(&self->w_dev, w5500_txpoll);
}

/****************************************************************************
 * Name: w5500_interrupt_work
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
 *   Runs on a worker thread.
 *
 ****************************************************************************/

static void w5500_interrupt_work(FAR void *arg)
{
  FAR struct w5500_driver_s *self = (FAR struct w5500_driver_s *)arg;
  uint8_t ir[3];

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts.  Read IR, MIR and SIR in one shot
   * to optimize latency, although MIR is not actually used.
   */

  w5500_read(self,
             W5500_BSB_COMMON_REGS,
             W5500_IR,
             ir,
             sizeof(ir));

  /* We expect none of the common (integrated network stack related)
   * interrupts and only interrupts from socket 0.
   */

  if (ir[0] != 0)
    {
      nwarn("Ignoring unexpected interrupts. IR: 0x%02"PRIx8"\n", ir[0]);

      w5500_write8(self,
                   W5500_BSB_COMMON_REGS,
                   W5500_IR,
                   ir[0]);
    }

  if (ir[2] & ~SIR(0))
    {
      nwarn("Interrupt pending for unused socket. SIR: 0x%02"PRIx8"\n",
            ir[2]);

      goto error;
    }

  if (ir[2] == 0)
    {
      nwarn("Overinitiative interrupt work.\n");

      goto done;
    }

  /* Get and clear interrupt status bits */

  ir[0] = w5500_read8(self,
                      W5500_BSB_SOCKET_REGS(0),
                      W5500_SN_IR);

  if ((ir[0] == 0) || ir[0] & ~(SN_INT_RECV | SN_INT_SEND_OK))
    {
      nerr("Unsupported socket interrupts: %02"PRIx8"\n", ir[0]);

      goto error;
    }

  w5500_write8(self,
               W5500_BSB_SOCKET_REGS(0),
               W5500_SN_IR,
               ir[0]);

  /* Handle interrupts according to status bit settings */

  /* Check if a packet transmission just completed.  If so, call
   * w5500_txdone.  This may disable further Tx interrupts if there are no
   * pending transmissions.
   */

  if (ir[0] & SN_INT_SEND_OK)
    {
      w5500_txdone(self);
    }

  /* Check if we received an incoming packet, if so, call w5500_receive() */

  if (ir[0] & SN_INT_RECV)
    {
      w5500_receive(self);
    }

done:
  net_unlock();

  /* Re-enable Ethernet interrupts */

  self->lower->enable(self->lower, true);

  return;

error:
  w5500_fence(self);
  net_unlock();
}

/****************************************************************************
 * Name: w5500_interrupt
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
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static int w5500_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct w5500_driver_s *self = (FAR struct w5500_driver_s *)arg;

  DEBUGASSERT(self != NULL);

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  self->lower->enable(self->lower, false);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &self->w_irqwork, w5500_interrupt_work, self, 0);
  return OK;
}

/****************************************************************************
 * Name: w5500_txtimeout_work
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
 ****************************************************************************/

static void w5500_txtimeout_work(FAR void *arg)
{
  FAR struct w5500_driver_s *self = (FAR struct w5500_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(self->w_dev);

  /* Then reset the hardware */

  if (w5500_unfence(self) == OK)
    {
      self->lower->enable(self->lower, true);

      /* Then poll the network for new XMIT data */

      devif_poll(&self->w_dev, w5500_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: w5500_txtimeout_expiry
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
 *   Runs in the context of a the timer interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static void w5500_txtimeout_expiry(wdparm_t arg)
{
  FAR struct w5500_driver_s *self = (FAR struct w5500_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  w5500_fence(self);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &self->w_irqwork, w5500_txtimeout_work, self, 0);
}

/****************************************************************************
 * Name: w5500_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int w5500_ifup(FAR struct net_driver_s *dev)
{
  FAR struct w5500_driver_s *self =
    (FAR struct w5500_driver_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)dev->d_ipaddr & 0xff,
        (int)(dev->d_ipaddr >> 8) & 0xff,
        (int)(dev->d_ipaddr >> 16) & 0xff,
        (int)dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  ret = w5500_unfence(self);

  if (ret != OK)
    {
      return ret;
    }

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  w5500_ipv6multicast(self);
#endif

  /* Enable the Ethernet interrupt */

  self->w_bifup = true;
  self->lower->enable(self->lower, true);

  return OK;
}

/****************************************************************************
 * Name: w5500_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int w5500_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct w5500_driver_s *self =
    (FAR struct w5500_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  self->lower->enable(self->lower, false);

  /* Cancel the TX timeout timer */

  wd_cancel(&self->w_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the w5500_ifup() always
   * successfully brings the interface back up.
   */

  w5500_fence(self);

  /* Mark the device "down" */

  self->w_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: w5500_txavail_work
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
 *   Runs on a work queue thread.
 *
 ****************************************************************************/

static void w5500_txavail_work(FAR void *arg)
{
  FAR struct w5500_driver_s *priv = (FAR struct w5500_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->w_bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      /* If so, then poll the network for new XMIT data */

      devif_poll(&priv->w_dev, w5500_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: w5500_txavail
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
 *   The network is locked.
 *
 ****************************************************************************/

static int w5500_txavail(FAR struct net_driver_s *dev)
{
  FAR struct w5500_driver_s *priv =
    (FAR struct w5500_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->w_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->w_pollwork, w5500_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: w5500_addmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int w5500_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct w5500_driver_s *priv =
    (FAR struct w5500_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: w5500_rmmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int w5500_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct w5500_driver_s *priv =
    (FAR struct w5500_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: w5500_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void w5500_ipv6multicast(FAR struct w5500_driver_s *priv)
{
  FAR struct net_driver_s *dev;
  uint16_t tmp16;
  uint8_t mac[6];

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Ethernet MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Ethernet MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;

  dev    = &priv->dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  w5500_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  w5500_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  w5500_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: w5500_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int w5500_ioctl(FAR struct net_driver_s *dev, int cmd,
                       unsigned long arg)
{
  FAR struct w5500_driver_s *priv =
    (FAR struct w5500_driver_s *)dev->d_private;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      /* Add cases here to support the IOCTL commands */

      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
        return -ENOTTY;  /* Special return value for this case */
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w5500_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the W5500.
 *   lower - The lower half driver instance for this W5500 chip.
 *   devno - If more than one W5500 is supported, then this is the
 *           zero based number that identifies the W5500.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int w5500_initialize(FAR struct spi_dev_s *spi_dev,
                     FAR const struct w5500_lower_s *lower,
                     unsigned int devno)
{
  FAR struct w5500_driver_s *self;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(devno < CONFIG_NET_W5500_NINTERFACES);
  self = &g_w5500[devno];

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (lower->attach(lower, w5500_interrupt, self))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(self, 0, sizeof(struct w5500_driver_s));
  self->w_dev.d_buf     = (FAR uint8_t *)g_pktbuf[devno]; /* Single packet buffer */
  self->w_dev.d_ifup    = w5500_ifup;                     /* I/F up (new IP address) callback */
  self->w_dev.d_ifdown  = w5500_ifdown;                   /* I/F down callback */
  self->w_dev.d_txavail = w5500_txavail;                  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  self->w_dev.d_addmac  = w5500_addmac;                   /* Add multicast MAC address */
  self->w_dev.d_rmmac   = w5500_rmmac;                    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  self->w_dev.d_ioctl   = w5500_ioctl;                    /* Handle network IOCTL commands */
#endif
  self->w_dev.d_private = g_w5500;                        /* Used to recover private state from dev */
  self->spi_dev         = spi_dev;                        /* SPI hardware interconnect */
  self->lower           = lower;                          /* Low-level MCU specific support */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling w5500_ifdown().
   */

  w5500_ifdown(&self->w_dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&self->w_dev, NET_LL_ETHERNET);
  return OK;
}

#endif /* !defined(CONFIG_SCHED_WORKQUEUE) */

#endif /* CONFIG_NET_W5500 */
