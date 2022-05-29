/****************************************************************************
 * drivers/net/dm90x0.c
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

/* References: Davicom data sheets (DM9000-DS-F03-041906.pdf,
 *   DM9010-DS-F01-103006.pdf) and looking at lots of other DM90x0
 *   drivers.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_DM90x0)

/* Only one hardware interface supported at present (although there are
 * hooks throughout the design to that extending the support to multiple
 * interfaces should not be that difficult)
 */

#undef  CONFIG_DM9X_NINTERFACES
#define CONFIG_DM9X_NINTERFACES 1

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <net/ethernet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* DM90000 and DM9010 register offsets */

#define DM9X_NETC          0x00 /* Network control register */
#define DM9X_NETS          0x01 /* Network Status register */
#define DM9X_TXC           0x02 /* TX control register */
#define DM9X_TXS1          0x03 /* TX status register 1 */
#define DM9X_TXS2          0x03 /* TX status register 2 */
#define DM9X_RXC           0x05 /* RX control register */
#define DM9X_RXS           0x06 /* RX status register */
#define DM9X_RXOVF         0x07 /* Receive overflow counter register */
#define DM9X_BPTHRES       0x08 /* Back pressure threshold register */
#define DM9X_FCTHRES       0x09 /* Flow control threshold register */
#define DM9X_FC            0x0a /* RX/TX flow control register */
#define DM9X_EEPHYC        0x0b /* EEPROM & PHY control register */
#define DM9X_EEPHYA        0x0c /* EEPROM & PHY address register */
#define DM9X_EEPHYDL       0x0d /* EEPROM & PHY data register (lo) */
#define DM9X_EEPHYDH       0x0e /* EEPROM & PHY data register (hi) */
#define DM9X_WAKEUP        0x0f /* Wake-up control register */
#define DM9X_PAB0          0x10 /* Physical address register (byte 0) */
#define DM9X_PAB1          0x11 /* Physical address register (byte 1) */
#define DM9X_PAB2          0x12 /* Physical address register (byte 2) */
#define DM9X_PAB3          0x13 /* Physical address register (byte 3) */
#define DM9X_PAB4          0x14 /* Physical address register (byte 4) */
#define DM9X_PAB5          0x15 /* Physical address register (byte 5) */
#define DM9X_MAB0          0x16 /* Multicast address register (byte 0) */
#define DM9X_MAB1          0x17 /* Multicast address register (byte 1) */
#define DM9X_MAB2          0x18 /* Multicast address register (byte 2) */
#define DM9X_MAB3          0x19 /* Multicast address register (byte 3) */
#define DM9X_MAB4          0x1a /* Multicast address register (byte 4) */
#define DM9X_MAB5          0x1b /* Multicast address register (byte 5) */
#define DM9X_MAB6          0x1c /* Multicast address register (byte 6) */
#define DM9X_MAB7          0x1d /* Multicast address register (byte 7) */
#define DM9X_GPC           0x1e /* General purpose control register */
#define DM9X_GPD           0x1f /* General purpose register */

#define DM9X_TRPAL         0x22 /* TX read pointer address (lo) */
#define DM9X_TRPAH         0x23 /* TX read pointer address (hi) */
#define DM9X_RWPAL         0x24 /* RX write pointer address (lo) */
#define DM9X_RWPAH         0x25 /* RX write pointer address (hi) */

#define DM9X_VIDL          0x28 /* Vendor ID (lo) */
#define DM9X_VIDH          0x29 /* Vendor ID (hi) */
#define DM9X_PIDL          0x2a /* Product ID (lo) */
#define DM9X_PIDH          0x2b /* Product ID (hi) */
#define DM9X_CHIPR         0x2c /* Product ID (lo) */
#define DM9X_TXC2          0x2d /* Transmit control register 2 (dm9010) */
#define DM9X_OTC           0x2e /* Operation test control register (dm9010) */
#define DM9X_SMODEC        0x2f /* Special mode control register */
#define DM9X_ETXCSR        0x30 /* Early transmit control/status register (dm9010) */
#define DM9X_TCCR          0x31 /* Transmit checksum control register (dm9010) */
#define DM9X_RCSR          0x32 /* Receive checksum control/status register (dm9010) */
#define DM9X_EPHYA         0x33 /* External PHY address register (dm9010) */
#define DM9X_GPC2          0x34 /* General purpose control register 2 (dm9010) */
#define DM9X_GPD2          0x35 /* General purpose register 2 */
#define DM9X_GPC3          0x36 /* General purpose control register 3 (dm9010) */
#define DM9X_GPD3          0x37 /* General purpose register 3 */
#define DM9X_PBUSC         0x38 /* Processor bus control register (dm9010) */
#define DM9X_IPINC         0x39 /* INT pin control register (dm9010) */

#define DM9X_MON1          0x40 /* Monitor register 1 (dm9010) */
#define DM9X_MON2          0x41 /* Monitor register 2 (dm9010) */

#define DM9X_SCLKC         0x50 /* System clock turn ON control register (dm9010) */
#define DM9X_SCLKR         0x51 /* Resume system clock control register (dm9010) */

#define DM9X_MRCMDX        0xf0 /* Memory data pre-fetch read command without address increment */
#define DM9X_MRCMDX1       0xf1 /* memory data read command without address increment (dm9010) */
#define DM9X_MRCMD         0xf2 /* Memory data read command with address increment */
#define DM9X_MDRAL         0xf4 /* Memory data read address register (lo) */
#define DM9X_MDRAH         0xf5 /* Memory data read address register (hi) */
#define DM9X_MWCMDX        0xf6 /* Memory data write command without address increment */
#define DM9X_MWCMD         0xf8 /* Memory data write command with address increment */
#define DM9X_MDWAL         0xfa /* Memory data write address register (lo) */
#define DM9X_MDWAH         0xfb /* Memory data write address register (lo) */
#define DM9X_TXPLL         0xfc /* Memory data write address register (lo) */
#define DM9X_TXPLH         0xfd /* Memory data write address register (hi) */
#define DM9X_ISR           0xfe /* Interrupt status register */
#define DM9X_IMR           0xff /* Interrupt mask register */

/* Network control register bit definitions */

#define DM9X_NETC_RST      (1 << 0) /* Software reset */
#define DM9X_NETC_LBKM     (3 << 1) /* Loopback mode mask */
#define DM9X_NETC_LBK0     (0 << 1) /*   0: Normal */
#define DM9X_NETC_LBK1     (1 << 1) /*   1: MAC internal loopback */
#define DM9X_NETC_LBK2     (2 << 1) /*   2: Internal PHY 100M mode loopback */
#define DM9X_NETC_FDX      (1 << 3) /* Full dupliex mode */
#define DM9X_NETC_FCOL     (1 << 4) /* Force collision mode */
#define DM9X_NETC_WAKEEN   (1 << 6) /* Wakeup event enable */
#define DM9X_NETC_EXTPHY   (1 << 7) /* Select external PHY */

/* Network status bit definitions */

#define DM9X_NETS_RXOV     (1 << 1) /* RX Fifo overflow */
#define DM9X_NETS_TX1END   (1 << 2) /* TX packet 1 complete status */
#define DM9X_NETS_TX2END   (1 << 3) /* TX packet 2 complete status */
#define DM9X_NETS_WAKEST   (1 << 5) /* Wakeup event status */
#define DM9X_NETS_LINKST   (1 << 6) /* Link status */
#define DM9X_NETS_SPEED    (1 << 7) /* Media speed */

/* IMR/ISR bit definitions */

#define DM9X_INT_PR        (1 << 0) /* Packet received interrupt */
#define DM9X_INT_PT        (1 << 1) /* Packet transmitted interrupt */
#define DM9X_INT_RO        (1 << 2) /* Receive overflow interrupt */
#define DM9X_INT_ROO       (1 << 3) /* Receive overflow counter overflow int */
#define DM9X_INT_UDRUN     (1 << 4) /* Transmit underrun interrupt */
#define DM9X_INT_LNKCHG    (1 << 5) /* Link status change interrupt */
#define DM9X_INT_ALL       (0x3f)

#define DM9X_IMR_UNUSED    (1 << 6) /* (not used) */
#define DM9X_IMR_PAR       (1 << 7) /* Enable auto R/W pointer reset */

#define DM9X_ISR_IOMODEM   (3 << 6) /* IO mode mask */
#define DM9X_ISR_IOMODE8   (2 << 6) /*   IO mode = 8 bit */
#define DM9X_ISR_IOMODE16  (0 << 6) /*   IO mode = 16 bit */
#define DM9X_ISR_IOMODE32  (1 << 6) /*   IO mode = 32 bit */

#define DM9X_IMRENABLE     (DM9X_INT_PR | DM9X_INT_PT | DM9X_INT_LNKCHG | DM9X_IMR_PAR)
#define DM9X_IMRRXDISABLE  (DM9X_INT_PT | DM9X_INT_LNKCHG | DM9X_IMR_PAR)
#define DM9X_IMRDISABLE    (DM9X_IMR_PAR)

/* EEPROM/PHY control register bits */

#define DM9X_EEPHYC_ERRE   (1 << 0) /* EEPROM (vs PHY) access status */
#define DM9X_EEPHYC_ERPRW  (1 << 1) /* EEPROM/PHY write access */
#define DM9X_EEPHYC_ERPRR  (1 << 2) /* EEPROM/PHY read access */
#define DM9X_EEPHYC_EPOS   (1 << 3) /* EEPROM/PHY operation select */
#define DM9X_EEPHYC_WEP    (1 << 4) /* Write EEPROM enable */
#define DM9X_EEPHYC_REEP   (1 << 5) /* Reload EEPROM */

/* Supported values from the vendor and product ID register */

#define DM9X_DAVICOMVID    0x0a46
#define DM9X_DM9000PID     0x9000
#define DM9X_DM9010PID     0x9010

/* RX control register bit settings */

#define DM9X_RXC_RXEN      (1 << 0) /* RX enable */
#define DM9X_RXC_PRMSC     (1 << 1) /* Promiscuous mode */
#define DM9X_RXC_RUNT      (1 << 2) /* Pass runt packet */
#define DM9X_RXC_ALL       (1 << 3) /* Pass all multicast */
#define DM9X_RXC_DISCRC    (1 << 4) /* Discard CRC error packets */
#define DM9X_RXC_DISLONG   (1 << 5) /* Discard long packets */
#define DM9X_RXC_WTDIS     (1 << 6) /* Disable watchdog timer */
#define DM9X_RXC_HASHALL   (1 << 7) /* Filter all addresses in hash table */

#define DM9X_RXCSETUP      (DM9X_RXC_DISCRC | DM9X_RXC_DISLONG)

/* EEPHY bit settings */

#define DM9X_EEPHYA_EROA   0x40 /* PHY register address 0x01 */

#define DM9X_PKTRDY        0x01 /* Packet ready to receive */

/* The RX interrupt will be disabled if more than the following RX
 * interrupts are received back-to-back.
 */

#define DM9X_CRXTHRES 10

/* All access is via an index register and a data register.  Select accecss
 * according to user supplied base address and bus width.
 */

#if defined(CONFIG_DM9X_BUSWIDTH8)
#  define DM9X_INDEX *(volatile uint8_t*)(CONFIG_DM9X_BASE)
#  define DM9X_DATA  *(volatile uint8_t*)(CONFIG_DM9X_BASE + 2)
#elif defined(CONFIG_DM9X_BUSWIDTH16)
#  define DM9X_INDEX *(volatile uint16_t*)(CONFIG_DM9X_BASE)
#  define DM9X_DATA  *(volatile uint16_t*)(CONFIG_DM9X_BASE + 2)
#elif defined(CONFIG_DM9X_BUSWIDTH32)
#  define DM9X_INDEX *(volatile uint32_t*)(CONFIG_DM9X_BASE)
#  define DM9X_DATA  *(volatile uint32_t*)(CONFIG_DM9X_BASE + 2)
#endif

/* Phy operating mode.  Default is AUTO, but this setting can be overridden
 * in the NuttX configuration file.
 */

#if !defined(CONFIG_DM9X_MODE_AUTO)   && !defined(CONFIG_DM9X_MODE_10MHD) && \
    !defined(CONFIG_DM9X_MODE_100MHD) && !defined(CONFIG_DM9X_MODE_10MFD) && \
    !defined(CONFIG_DM9X_MODE_100MFD)
#   define CONFIG_DM9X_MODE_AUTO 1
#endif

/* TX timeout = 1 minute */

#define DM6X_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->dm_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

union rx_desc_u
{
  uint8_t rx_buf[4];
  struct
  {
    uint8_t  rx_byte;
    uint8_t  rx_status;
    uint16_t rx_len;
  } desc;
};

/* The dm9x_driver_s encapsulates all DM90x0 state information for a single
 * DM90x0 hardware interface
 */

struct dm9x_driver_s
{
  bool dm_bifup;               /* true:ifup false:ifdown */
  bool dm_b100m;               /* true:speed == 100M; false:speed == 10M */
  uint8_t dm_ntxpending;       /* Count of packets pending transmission */
  uint8_t ncrxpackets;         /* Number of continuous rx packets  */
  struct wdog_s dm_txtimeout;  /* TX timeout timer */
  struct work_s dm_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s dm_pollwork;   /* For deferring poll work to the work queue */

  /* Mode-dependent function to move data in 8/16/32 I/O modes */

  void (*dm_read)(uint8_t *ptr, int len);
  void (*dm_write)(const uint8_t *ptr, int len);
  void (*dm_discard)(int len);

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dm_dev;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* At present, only a single DM90x0 device is supported. */

static struct dm9x_driver_s g_dm9x[CONFIG_DM9X_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static uint8_t getreg(int reg);
static void putreg(int reg, uint8_t value);
static void read8(uint8_t *ptr, int len);
static void read16(uint8_t *ptr, int len);
static void read32(uint8_t *ptr, int len);
static void discard8(int len);
static void discard16(int len);
static void discard32(int len);
static void write8(const uint8_t *ptr, int len);
static void write16(const uint8_t *ptr, int len);
static void write32(const uint8_t *ptr, int len);

#if 0 /* Not used */
static uint16_t dm9x_readsrom(FAR struct dm9x_driver_s *priv, int offset);
#endif
static uint16_t dm9x_phyread(FAR struct dm9x_driver_s *priv, int reg);
static void dm9x_phywrite(FAR struct dm9x_driver_s *priv, int reg,
              uint16_t value);

#if defined(CONFIG_DM9X_CHECKSUM)
static bool dm9x_rxchecksumready(uint8_t);
#else
#  define dm9x_rxchecksumready(a) ((a) == 0x01)
#endif

/* Common TX logic */

static int  dm9x_transmit(FAR struct dm9x_driver_s *priv);
static int  dm9x_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void dm9x_receive(FAR struct dm9x_driver_s *priv);
static void dm9x_txdone(FAR struct dm9x_driver_s *priv);

static void dm9x_interrupt_work(FAR void *arg);
static int  dm9x_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void dm9x_txtimeout_work(FAR void *arg);
static void dm9x_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int dm9x_ifup(FAR struct net_driver_s *dev);
static int dm9x_ifdown(FAR struct net_driver_s *dev);

static void dm9x_txavail_work(FAR void *arg);
static int dm9x_txavail(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int dm9x_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
static int dm9x_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/* Initialization functions */

static void dm9x_bringup(FAR struct dm9x_driver_s *priv);
static void dm9x_reset(FAR struct dm9x_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getreg and setreg
 *
 * Description:
 *   Access to memory-mapped DM90x0 8-bit registers
 *
 * Input Parameters:
 *   reg - Register number
 *   value - Value to write to the register (setreg only)
 *
 * Returned Value:
 *   Value read from the register (getreg only)
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint8_t getreg(int reg)
{
  DM9X_INDEX = reg;
  return DM9X_DATA & 0xff;
}

static void putreg(int reg, uint8_t value)
{
  DM9X_INDEX = reg;
  DM9X_DATA  = value & 0xff;
}

/****************************************************************************
 * Name: read8, read16, read32
 *
 * Description:
 *   Read packet data from the DM90x0 SRAM based on its current I/O mode
 *
 * Input Parameters:
 *   ptr - Location to write the packet data
 *   len - The number of bytes to read
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void read8(FAR uint8_t *ptr, int len)
{
  ninfo("Read %d bytes (8-bit mode)\n", len);
  for (; len > 0; len--)
    {
      *ptr++ = DM9X_DATA;
    }
}

static void read16(FAR uint8_t *ptr, int len)
{
  FAR uint16_t *ptr16 = (FAR uint16_t *)ptr;

  ninfo("Read %d bytes (16-bit mode)\n", len);
  for (; len > 0; len -= sizeof(uint16_t))
    {
      *ptr16++ = DM9X_DATA;
    }
}

static void read32(FAR uint8_t *ptr, int len)
{
  FAR uint32_t *ptr32 = (FAR uint32_t *)ptr;

  ninfo("Read %d bytes (32-bit mode)\n", len);
  for (; len > 0; len -= sizeof(uint32_t))
    {
      *ptr32++ = DM9X_DATA;
    }
}

/****************************************************************************
 * Name: discard8, discard16, discard32
 *
 * Description:
 *   Read and discard packet data in the DM90x0 SRAM based on its current
 *   I/O mode
 *
 * Input Parameters:
 *   len - The number of bytes to discard
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void discard8(int len)
{
  ninfo("Discard %d bytes (8-bit mode)\n", len);
  for (; len > 0; len--)
    {
      DM9X_DATA;
    }
}

static void discard16(int len)
{
  ninfo("Discard %d bytes (16-bit mode)\n", len);
  for (; len > 0; len -= sizeof(uint16_t))
    {
      DM9X_DATA;
    }
}

static void discard32(int len)
{
  ninfo("Discard %d bytes (32-bit mode)\n", len);
  for (; len > 0; len -= sizeof(uint32_t))
    {
      DM9X_DATA;
    }
}

/****************************************************************************
 * Name: write8, write16, write32
 *
 * Description:
 *   Write packet data into the DM90x0 SRAM based on its current I/O mode
 *
 * Input Parameters:
 *   ptr - Location to write the packet data
 *   len - The number of bytes to read
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void write8(FAR const uint8_t *ptr, int len)
{
  ninfo("Write %d bytes (8-bit mode)\n", len);

  for (; len > 0; len--)
    {
      DM9X_DATA = (*ptr++ & 0xff);
    }
}

static void write16(const uint8_t *ptr, int len)
{
  FAR uint16_t *ptr16 = (FAR uint16_t *)ptr;

  ninfo("Write %d bytes (16-bit mode)\n", len);

  for (; len > 0; len -= sizeof(uint16_t))
    {
       DM9X_DATA = *ptr16++;
    }
}

static void write32(FAR const uint8_t *ptr, int len)
{
  FAR uint32_t *ptr32 = (FAR uint32_t *)ptr;

  ninfo("Write %d bytes (32-bit mode)\n", len);

  for (; len > 0; len -= sizeof(uint32_t))
    {
      DM9X_DATA = *ptr32++;
    }
}

/****************************************************************************
 * Name: dm9x_readsrom
 *
 * Description:
 *   Read a word from SROM
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   offset - SROM offset to read from
 *
 * Returned Value:
 *   SROM content at that offset
 *
 * Assumptions:
 *
 ****************************************************************************/

#if 0 /* Not used */
static uint16_t dm9x_readsrom(FAR struct dm9x_driver_s *priv, int offset)
{
  putreg(DM9X_EEPHYA, offset);
  putreg(DM9X_EEPHYC, DM9X_EEPHYC_ERPRR);
  up_udelay(200);
  putreg(DM9X_EEPHYC, 0x00);
  return (getreg(DM9X_EEPHYDL) + (getreg(DM9X_EEPHYDH) << 8));
}
#endif

/****************************************************************************
 * Name: dm9x_phyread and dm9x_phywrite
 *
 * Description:
 *   Read/write data from/to the PHY
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   reg   - PHY register offset
 *   value - The value to write to the PHY register (dm9x_write only)
 *
 * Returned Value:
 *   The value read from the PHY (dm9x_read only)
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint16_t dm9x_phyread(FAR struct dm9x_driver_s *priv, int reg)
{
  /* Setup DM9X_EEPHYA, the EEPROM/PHY address register */

  putreg(DM9X_EEPHYA, DM9X_EEPHYA_EROA | reg);

  /* Issue PHY read command pulse in the EEPROM/PHY control register */

  putreg(DM9X_EEPHYC, (DM9X_EEPHYC_ERPRR | DM9X_EEPHYC_EPOS));
  up_udelay(100);
  putreg(DM9X_EEPHYC, 0x00);

  /* Return the data from the EEPROM/PHY data register pair */

  return (((uint16_t)getreg(DM9X_EEPHYDH)) << 8) |
           (uint16_t)getreg(DM9X_EEPHYDL);
}

static void dm9x_phywrite(FAR struct dm9x_driver_s *priv, int reg,
                          uint16_t value)
{
  /* Setup DM9X_EEPHYA, the EEPROM/PHY address register */

  putreg(DM9X_EEPHYA, DM9X_EEPHYA_EROA | reg);

  /* Put the data to write in the EEPROM/PHY data register pair */

  putreg(DM9X_EEPHYDL, (value & 0xff));
  putreg(DM9X_EEPHYDH, ((value >> 8) & 0xff));

  /* Issue PHY write command pulse in the EEPROM/PHY control register */

  putreg(DM9X_EEPHYC, (DM9X_EEPHYC_ERPRW | DM9X_EEPHYC_EPOS));
  up_udelay(500);
  putreg(DM9X_EEPHYC, 0x0);
}

/****************************************************************************
 * Name: dm9x_rxchecksumready
 *
 * Description:
 *   Return true if the RX checksum is available
 *
 * Input Parameters:
 *   rxbyte
 *
 * Returned Value:
 *   true: checksum is ready
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_DM9X_CHECKSUM)
static inline bool dm9x_rxchecksumready(uint8_t rxbyte)
{
  if ((rxbyte & 0x01) == 0)
    {
      return false;
    }

  return ((rxbyte >> 4) | 0x01) != 0;
}
#endif

/****************************************************************************
 * Name: dm9x_transmit
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

static int dm9x_transmit(FAR struct dm9x_driver_s *priv)
{
  /* Check if there is room in the DM90x0 to hold another packet.  In 100M
   * mode, that can be 2 packets, otherwise it is a single packet.
   */

  if (priv->dm_ntxpending < 1 || (priv->dm_b100m && priv->dm_ntxpending < 2))
    {
      /* Increment count of packets transmitted */

      priv->dm_ntxpending++;
      NETDEV_TXPACKETS(&dm9x0->dm_dev);

      /* Disable all DM90x0 interrupts */

      putreg(DM9X_IMR, DM9X_IMRDISABLE);

      /* Set the TX length */

      putreg(DM9X_TXPLL, (priv->dm_dev.d_len & 0xff));
      putreg(DM9X_TXPLH, (priv->dm_dev.d_len >> 8) & 0xff);

      /* Move the data to be sent into TX SRAM */

      DM9X_INDEX = DM9X_MWCMD;
      priv->dm_write(priv->dm_dev.d_buf, priv->dm_dev.d_len);

#if !defined(CONFIG_DM9X_ETRANS)
      /* Issue TX polling command */

      putreg(DM9X_TXC, 0x1); /* Cleared after TX complete */
#endif

      /* Clear count of back-to-back RX packet transfers */

      priv->ncrxpackets = 0;

      /* Re-enable DM90x0 interrupts */

      putreg(DM9X_IMR, DM9X_IMRENABLE);

      /* Setup the TX timeout watchdog (perhaps restarting the timer) */

      wd_start(&priv->dm_txtimeout, DM6X_TXTIMEOUT,
               dm9x_txtimeout_expiry, (wdparm_t)priv);
      return OK;
    }

  return -EBUSY;
}

/****************************************************************************
 * Name: dm9x_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the DM90x0 is reset
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

static int dm9x_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dm_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->dm_dev.d_flags))
#endif
        {
          arp_out(&priv->dm_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->dm_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->dm_dev))
        {
          /* Send the packet */

          dm9x_transmit(priv);

          /* Check if there is room in the DM90x0 to hold another packet.
           * In 100M mode, that can be 2 packets, otherwise it is a single
           * packet.
           */

          if (priv->dm_ntxpending > 1 || !priv->dm_b100m)
            {
              /* Returning a non-zero value terminate the poll operation */

              return 1;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: dm9x_receive
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

static void dm9x_receive(FAR struct dm9x_driver_s *priv)
{
  union rx_desc_u rx;
  bool bchecksumready;
  uint8_t rxbyte;

  ninfo("Packet received\n");

  do
    {
      /* Store the value of memory data read address register */

      getreg(DM9X_MDRAH);
      getreg(DM9X_MDRAL);

      getreg(DM9X_MRCMDX);         /* Dummy read */
      rxbyte = (uint8_t)DM9X_DATA; /* Get the most up-to-date data */

      /* Packet ready for receive check */

      bchecksumready = dm9x_rxchecksumready(rxbyte);
      if (!bchecksumready)
        {
          break;
        }

      /* A packet is ready now. Get status/length */

      DM9X_INDEX = DM9X_MRCMD; /* set read ptr ++ */

      /* Read packet status & length */

      priv->dm_read((FAR uint8_t *)&rx, 4);

      /* Check if any errors were reported by the hardware */

      if (rx.desc.rx_status & 0xbf)
        {
          /* Bad RX packet... update statistics */

          nerr("ERROR: Received packet with errors: %02x\n",
               rx.desc.rx_status);
          NETDEV_RXERRORS(&priv->dm_dev);

          /* Drop this packet and continue to check the next packet */

          priv->dm_discard(rx.desc.rx_len);
        }

      /* Also check if the packet is a valid size for the configuration */

      else if (rx.desc.rx_len < ETH_HDRLEN ||
               rx.desc.rx_len > (CONFIG_NET_ETH_PKTSIZE + 2))
        {
          nerr("ERROR: RX length error\n");
          NETDEV_RXERRORS(&priv->dm_dev);

          /* Drop this packet and continue to check the next packet */

          priv->dm_discard(rx.desc.rx_len);
        }
      else
        {
          /* Good packet...
           * Copy the packet data out of SRAM and pass it one to the network
           */

          priv->dm_dev.d_len = rx.desc.rx_len;
          priv->dm_read(priv->dm_dev.d_buf, rx.desc.rx_len);

#ifdef CONFIG_NET_PKT
          /* When packet sockets are enabled, feed the frame into the tap */

          pkt_input(&priv->dm_dev);
#endif

          /* We accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
          if (BUF->type == HTONS(ETHTYPE_IP))
            {
              ninfo("IPv4 frame\n");
              NETDEV_RXIPV4(&priv->dm_dev);

              /* Handle ARP on input then give the IPv4 packet to the network
               * layer
               */

              arp_ipin(&priv->dm_dev);
              ipv4_input(&priv->dm_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->dm_dev.d_len > 0)
                {
                  /* Update Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
                  if (IFF_IS_IPv4(priv->dm_dev.d_flags))
#endif
                    {
                      arp_out(&priv->dm_dev);
                    }
#ifdef CONFIG_NET_IPv6
                  else
                    {
                      neighbor_out(&priv->dm_dev);
                    }
#endif

                  /* And send the packet */

                  dm9x_transmit(priv);
                }
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
          if (BUF->type == HTONS(ETHTYPE_IP6))
            {
              ninfo("IPv6 frame\n");
              NETDEV_RXIPV6(&priv->dm_dev);

              /* Give the IPv6 packet to the network layer */

              ipv6_input(&priv->dm_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->dm_dev.d_len > 0)
                {
                  /* Update Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
                  if (IFF_IS_IPv4(priv->dm_dev.d_flags))
                    {
                      arp_out(&priv->dm_dev);
                    }
                  else
#endif
#ifdef CONFIG_NET_IPv6
                    {
                      neighbor_out(&priv->dm_dev);
                    }
#endif

                  /* And send the packet */

                      dm9x_transmit(priv);
                }
            }
          else
#endif
#ifdef CONFIG_NET_ARP
          if (BUF->type == HTONS(ETHTYPE_ARP))
            {
              arp_arpin(&priv->dm_dev);
              NETDEV_RXARP(&priv->dm_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will set
               * to a value > 0.
               */

              if (priv->dm_dev.d_len > 0)
                {
                  dm9x_transmit(priv);
                }
            }
#endif
          else
            {
              NETDEV_RXDROPPED(&priv->dm_dev);
            }
        }

      NETDEV_RXPACKETS(&priv->dm_dev);
      priv->ncrxpackets++;
    }
  while ((rxbyte & 0x01) == DM9X_PKTRDY &&
         priv->ncrxpackets < DM9X_CRXTHRES);
  ninfo("All RX packets processed\n");
}

/****************************************************************************
 * Name: dm9x_txdone
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

static void dm9x_txdone(FAR struct dm9x_driver_s *priv)
{
  int  nsr;

  ninfo("TX done\n");

  /* Another packet has completed transmission.  Decrement the count of
   * of pending TX transmissions.
   */

  nsr = getreg(DM9X_NETS);
  if (nsr & DM9X_NETS_TX1END)
    {
      if (priv->dm_ntxpending)
        {
          priv->dm_ntxpending--;
        }
      else
        {
          nerr("ERROR: Bad TX count (TX1END)\n");
        }
    }

  if (nsr & DM9X_NETS_TX2END)
    {
      if (priv->dm_ntxpending)
        {
          priv->dm_ntxpending--;
        }
      else
        {
          nerr("ERROR: Bad TX count (TX2END)\n");
        }
    }

  /* Cancel the TX timeout */

  if (priv->dm_ntxpending == 0)
    {
      wd_cancel(&priv->dm_txtimeout);
    }

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dm_dev, dm9x_txpoll);
}

/****************************************************************************
 * Name: dm9x_interrupt_work
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

static void dm9x_interrupt_work(FAR void *arg)
{
  FAR struct dm9x_driver_s *priv = (FAR struct dm9x_driver_s *)arg;
  uint8_t isr;
  uint8_t save;
  int i;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Save previous register address */

  save = (uint8_t)DM9X_INDEX;

  /* Disable all DM90x0 interrupts */

  putreg(DM9X_IMR, DM9X_IMRDISABLE);

  /* Get and clear the DM90x0 interrupt status bits */

  isr = getreg(DM9X_ISR);
  putreg(DM9X_ISR, isr);
  ninfo("Interrupt status: %02x\n", isr);

  /* Check for link status change */

  if (isr & DM9X_INT_LNKCHG)
    {
      /* Wait up to 0.5s for link OK */

      for (i = 0; i < 500; i++)
        {
          dm9x_phyread(priv, 0x1);
          if (dm9x_phyread(priv, 0x1) & 0x4) /* Link OK */
            {
              /* Wait to get detected speed */

              for (i = 0; i < 200; i++)
                {
                  up_mdelay(1);
                }

              /* Set the new network speed */

              if (dm9x_phyread(priv, 0) & 0x2000)
                {
                  priv->dm_b100m = true;
                }
              else
                {
                  priv->dm_b100m = false;
                }
              break;
            }

          up_mdelay(1);
        }

      nerr("ERROR: delay: %dmS speed: %s\n",
           i, priv->dm_b100m ? "100M" : "10M");
    }

  /* Check if we received an incoming packet */

  if (isr & DM9X_INT_PR)
    {
      dm9x_receive(priv);
    }

  /* Check if we are able to transmit a packet */

  if (isr & DM9X_INT_PT)
    {
      dm9x_txdone(priv);
    }

  /* If the number of consecutive receive packets exceeds a threshold,
   * then disable the RX interrupt.
   */

  if (priv->ncrxpackets >= DM9X_CRXTHRES)
    {
      /* Enable all DM90x0 interrupts EXCEPT for RX */

      putreg(DM9X_IMR, DM9X_IMRRXDISABLE);
    }
  else
    {
      /* Enable all DM90x0 interrupts */

      putreg(DM9X_IMR, DM9X_IMRENABLE);
    }

  /* Restore previous register address */

  DM9X_INDEX = save;
  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(CONFIG_DM9X_IRQ);
}

/****************************************************************************
 * Name: dm9x_interrupt
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

static int dm9x_interrupt(int irq, FAR void *context, FAR void *arg)
{
#if CONFIG_DM9X_NINTERFACES == 1
  FAR struct dm9x_driver_s *priv = &g_dm9x[0];
#else
# error "Additional logic needed to support multiple interfaces"
#endif
  uint8_t isr;

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(CONFIG_DM9X_IRQ);

  /* Determine if a TX transfer just completed */

  isr = getreg(DM9X_ISR);
  if ((isr & DM9X_INT_PT) != 0)
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be no race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(&priv->dm_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->dm_irqwork, dm9x_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: dm9x_txtimeout_work
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

static void dm9x_txtimeout_work(FAR void *arg)
{
  FAR struct dm9x_driver_s *priv = (FAR struct dm9x_driver_s *)arg;

  nerr("ERROR: TX timeout\n");

  /* Increment statistics and dump debug info */

  net_lock();
  NETDEV_TXTIMEOUTS(priv->dm_dev);

  ninfo("  TX packet count:           %d\n", priv->dm_ntxpending);
  ninfo("  TX read pointer address:   0x%02x:%02x\n",
        getreg(DM9X_TRPAH), getreg(DM9X_TRPAL));
  ninfo("  Memory data write address: 0x%02x:%02x (DM9010)\n",
        getreg(DM9X_MDWAH), getreg(DM9X_MDWAL));

  /* Then reset the DM90x0 */

  dm9x_reset(priv);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dm_dev, dm9x_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: dm9x_txtimeout_expiry
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

static void dm9x_txtimeout_expiry(wdparm_t arg)
{
  FAR struct dm9x_driver_s *priv = (FAR struct dm9x_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(CONFIG_DM9X_IRQ);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->dm_irqwork, dm9x_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: dm9x_phymode
 *
 * Description:
 *   Configure the PHY operating mode
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

static inline void dm9x_phymode(FAR struct dm9x_driver_s *priv)
{
  uint16_t phyreg0;
  uint16_t phyreg4;

#ifdef CONFIG_DM9X_MODE_AUTO
  phyreg0 = 0x1200;  /* Auto-negotiation & Restart Auto-negotiation */
  phyreg4 = 0x01e1;  /* Default flow control disable */
#elif defined(CONFIG_DM9X_MODE_10MHD)
  phyreg4 = 0x21;
  phyreg0 = 0x1000;
#elif defined(CONFIG_DM9X_MODE_10MFD)
  phyreg4 = 0x41;
  phyreg0 = 0x1100;
#elif defined(CONFIG_DM9X_MODE_100MHD)
  phyreg4 = 0x81;
  phyreg0 = 0x3000;
#elif defined(CONFIG_DM9X_MODE_100MFD)
  phyreg4 = 0x101;
  phyreg0 = 0x3100;
#else
# error "Recognized PHY mode"
#endif

  dm9x_phywrite(priv, 0, phyreg0);
  dm9x_phywrite(priv, 4, phyreg4);
}

/****************************************************************************
 * Name: dm9x_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the DM90x0 interface when an IP address is
 *   provided
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

static int dm9x_ifup(FAR struct net_driver_s *dev)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)dev->d_private;
  uint8_t netstatus;
  int i;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));

  /* Initialize DM90x0 chip */

  dm9x_bringup(priv);

  /* Check link state and media speed (waiting up to 3s for link OK) */

  priv->dm_b100m = false;
  for (i = 0; i < 3000; i++)
    {
      netstatus = getreg(DM9X_NETS);
      if (netstatus & DM9X_NETS_LINKST)
        {
          /* Link OK... Wait a bit before getting the detected speed */

          up_mdelay(200);
          netstatus = getreg(DM9X_NETS);
          if ((netstatus & DM9X_NETS_SPEED) == 0)
            {
              priv->dm_b100m = true;
            }

          break;
        }

      i++;
      up_mdelay(1);
    }

  ninfo("delay: %dmS speed: %s\n", i, priv->dm_b100m ? "100M" : "10M");

  /* Enable the DM9X interrupt */

  priv->dm_bifup = true;
  up_enable_irq(CONFIG_DM9X_IRQ);
  return OK;
}

/****************************************************************************
 * Name: dm9x_ifdown
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

static int dm9x_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Stopping\n");

  /* Disable the DM9X interrupt */

  flags = enter_critical_section();
  up_disable_irq(CONFIG_DM9X_IRQ);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->dm_txtimeout);

  /* Reset the device */

  dm9x_phywrite(priv, 0x00, 0x8000);  /* PHY reset */
  putreg(DM9X_GPD, 0x01);             /* Power-down PHY (GEPIO0=1) */
  putreg(DM9X_IMR, DM9X_IMRDISABLE);  /* Disable all interrupts */
  putreg(DM9X_RXC, 0x00);             /* Disable RX */
  putreg(DM9X_ISR, DM9X_INT_ALL);     /* Clear interrupt status */

  priv->dm_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: dm9x_txavail_work
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

static void dm9x_txavail_work(FAR void *arg)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)arg;

  ninfo("Polling\n");

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->dm_bifup)
    {
      /* Check if there is room in the DM90x0 to hold another packet. In 100M
       * mode, that can be 2 packets, otherwise it is a single packet.
       */

      if (priv->dm_ntxpending < 1 ||
          (priv->dm_b100m && priv->dm_ntxpending < 2))
        {
          /* If so, then poll the network for new XMIT data */

          devif_poll(&priv->dm_dev, dm9x_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: dm9x_txavail
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

static int dm9x_txavail(FAR struct net_driver_s *dev)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->dm_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->dm_pollwork, dm9x_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: dm9x_addmac
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
static int dm9x_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Name: dm9x_rmmac
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
static int dm9x_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct dm9x_driver_s *priv =
    (FAR struct dm9x_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Name: dm9x_bringup
 *
 * Description:
 *   Initialize the dm90x0 chip
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

static void dm9x_bringup(FAR struct dm9x_driver_s *priv)
{
  ninfo("Initializing\n");

  /* Set the internal PHY power-on, GPIOs normal, and wait 2ms */

  putreg(DM9X_GPD, 0x01);  /* Power-down the PHY (GEPIO0=1) */
  up_udelay(500);
  putreg(DM9X_GPD, 0x00);  /* Preactivate PHY (GPIO0=0 */
  up_udelay(20);           /* Wait 20us for PHY power-on ready */

  /* Do a software reset and wait 20us (twice).  The reset autoclears
   * in 10us; 20us guarantees completion of the reset
   */

  putreg(DM9X_NETC, (DM9X_NETC_RST | DM9X_NETC_LBK1));
  up_udelay(20);
  putreg(DM9X_NETC, (DM9X_NETC_RST | DM9X_NETC_LBK1));
  up_udelay(20);

  /* Configure I/O mode */

  switch (getreg(DM9X_ISR) & DM9X_ISR_IOMODEM)
    {
      case DM9X_ISR_IOMODE8:
        priv->dm_read    = read8;
        priv->dm_write   = write8;
        priv->dm_discard = discard8;
        break;

      case DM9X_ISR_IOMODE16:
        priv->dm_read    = read16;
        priv->dm_write   = write16;
        priv->dm_discard = discard16;
        break;

      case DM9X_ISR_IOMODE32:
        priv->dm_read    = read32;
        priv->dm_write   = write32;
        priv->dm_discard = discard32;
        break;

      default:
        break;
    }

  /* Program PHY operating mode */

  dm9x_phymode(priv);

  /* Program operating mode */

  putreg(DM9X_NETC, 0x00);        /* Network control */
  putreg(DM9X_TXC, 0x00);         /* Clear TX Polling */
  putreg(DM9X_BPTHRES, 0x3f);     /* Less 3kb, 600us */
  putreg(DM9X_SMODEC, 0x00);      /* Special mode */
                                  /* Clear TX status */
  putreg(DM9X_NETS, DM9X_NETS_WAKEST | DM9X_NETS_TX1END | DM9X_NETS_TX2END);
  putreg(DM9X_ISR, DM9X_INT_ALL); /* Clear interrupt status */

#if defined(CONFIG_DM9X_CHECKSUM)
  putreg(DM9X_TCCR, 0x07);        /* TX UDP/TCP/IP checksum enable */
  putreg(DM9X_RCSR, 0x02);        /* Receive checksum enable */
#endif

#if defined(CONFIG_DM9X_ETRANS)
  putreg(DM9X_ETXCSR, 0x83);
#endif

  /* Initialize statistics */

  priv->ncrxpackets   = 0; /* Number of continuous RX packets  */
  priv->dm_ntxpending = 0; /* Number of pending TX packets */
  NETDEV_RESET_STATISTICS(&priv->dm_dev);

  /* Activate DM9000A/DM9010 */

  putreg(DM9X_RXC, DM9X_RXCSETUP | 1); /* RX enable */
  putreg(DM9X_IMR, DM9X_IMRENABLE);    /* Enable TX/RX interrupts */
}

/****************************************************************************
 * Name: dm9x_reset
 *
 * Description:
 *   Stop, reset, re-initialize, and restart the DM90x0 chip and driver.  At
 *   present, the chip is only reset after a TX timeout.
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

static void dm9x_reset(FAR struct dm9x_driver_s *priv)
{
  uint8_t save;
  int i;

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->dm_txtimeout);

  /* Save previous register address */

  save = (uint8_t)DM9X_INDEX;
  dm9x_bringup(priv);

  /* Wait up to 1 second for the link to be OK */

  priv->dm_b100m = false;
  for (i = 0; i < 1000; i++)
    {
      if (dm9x_phyread(priv, 0x1) & 0x4)
        {
          if (dm9x_phyread(priv, 0) & 0x2000)
            {
              priv->dm_b100m = true;
            }

          break;
        }

      up_mdelay(1);
    }

  /* Restore previous register address */

  DM9X_INDEX = save;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dm9x_initialize
 *
 * Description:
 *   Initialize the DM90x0 driver
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

int dm9x_initialize(void)
{
  uint8_t *mptr;
  uint16_t vid;
  uint16_t pid;
  int i;
  int j;

  /* Get the chip vendor ID and product ID */

  vid = (((uint16_t)getreg(DM9X_VIDH)) << 8) | (uint16_t)getreg(DM9X_VIDL);
  pid = (((uint16_t)getreg(DM9X_PIDH)) << 8) | (uint16_t)getreg(DM9X_PIDL);

  ninfo("I/O base: %08x VID: %04x PID: %04x\n", CONFIG_DM9X_BASE, vid, pid);

  /* Check if a DM90x0 chip is recognized at this I/O base */

  if (vid != DM9X_DAVICOMVID ||
      (pid != DM9X_DM9000PID && pid != DM9X_DM9010PID))
    {
      nerr("ERROR: vendor/product ID not found at this base address\n");
      return -ENODEV;
    }

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_DM9X_IRQ, dm9x_interrupt, NULL))
    {
      /* We could not attach the ISR to the ISR */

      nerr("ERROR: irq_attach() failed\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(g_dm9x, 0, CONFIG_DM9X_NINTERFACES*sizeof(struct dm9x_driver_s));
  g_dm9x[0].dm_dev.d_buf     = g_pktbuf;      /* Single packet buffer */
  g_dm9x[0].dm_dev.d_ifup    = dm9x_ifup;     /* I/F down callback */
  g_dm9x[0].dm_dev.d_ifdown  = dm9x_ifdown;   /* I/F up (new IP address) callback */
  g_dm9x[0].dm_dev.d_txavail = dm9x_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  g_dm9x[0].dm_dev.d_addmac  = dm9x_addmac;   /* Add multicast MAC address */
  g_dm9x[0].dm_dev.d_rmmac   = dm9x_rmmac;    /* Remove multicast MAC address */
#endif
  g_dm9x[0].dm_dev.d_private = g_dm9x;        /* Used to recover private state from dev */

  /* Read the MAC address */

  mptr = g_dm9x[0].dm_dev.d_mac.ether.ether_addr_octet;
  for (i = 0, j = DM9X_PAB0; i < ETHER_ADDR_LEN; i++, j++)
    {
      mptr[i] = getreg(j);
    }

  ninfo("MAC: %0x:%0x:%0x:%0x:%0x:%0x\n",
       mptr[0], mptr[1], mptr[2], mptr[3], mptr[4], mptr[5]);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&g_dm9x[0].dm_dev, NET_LL_ETHERNET);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_DM90x0 */
