/****************************************************************************
 * drivers/net/enc28j60.c
 *
 *   Copyright (C) 2010-2012, 2014-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * - ENC28J60 Data Sheet, Stand-Alone Ethernet Controller with SPI Interface,
 *   DS39662C, 2008 Microchip Technology Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_NET) && defined(CONFIG_ENC28J60)

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/net/enc28j60.h>
#include <nuttx/net/net.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "enc28j60.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* ENC28J60 Configuration Settings:
 *
 * CONFIG_ENC28J60 - Enabled ENC28J60 support
 * CONFIG_ENC28J60_SPIMODE - Controls the SPI mode
 * CONFIG_ENC28J60_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ENC28J60_NINTERFACES - Specifies the number of physical ENC28J60
 *   devices that will be supported.
 * CONFIG_ENC28J60_HALFDUPPLEX - Default is full duplex
 */

/* The ENC28J60 spec says that it supports SPI mode 0,0 only: "The
 * implementation used on this device supports SPI mode 0,0 only. In
 * addition, the SPI port requires that SCK be at Idle in a low state;
 * selectable clock polarity is not supported."  However, sometimes you
 * need to tinker with these things.
 */

#ifndef CONFIG_ENC28J60_SPIMODE
#  define CONFIG_ENC28J60_SPIMODE SPIDEV_MODE0
#endif

/* CONFIG_ENC28J60_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_ENC28J60_NINTERFACES
#  define CONFIG_ENC28J60_NINTERFACES 1
#endif

/* CONFIG_NET_ETH_PKTSIZE must always be defined */

#if !defined(CONFIG_NET_ETH_PKTSIZE) && (CONFIG_NET_ETH_PKTSIZE <= MAX_FRAMELEN)
#  error "CONFIG_NET_ETH_PKTSIZE is not valid for the ENC28J60"
#endif

/* We need to have the work queue to handle SPI interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define ENCWORK LPWORK

/* CONFIG_ENC28J60_DUMPPACKET will dump the contents of each packet. */

#ifdef CONFIG_ENC28J60_DUMPPACKET
#  define enc_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define enc_dumppacket(m,a,n)
#endif

/* Low-level register debug */

#if !defined(CONFIG_DEBUG_FEATURES) || !defined(CONFIG_DEBUG_NET)
#  undef CONFIG_ENC28J60_REGDEBUG
#endif

/* Timing *******************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of ticks per second */

#define ENC_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define ENC_TXTIMEOUT (60*CLK_TCK)

/* Poll timeout */

#define ENC_POLLTIMEOUT MSEC2TICK(50)

/* Packet Memory ************************************************************/

/* Packet memory layout */

#define ALIGNED_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 255) & ~255)

/* Work around Errata #5 (spurious reset of ERXWRPT to 0) by placing the RX
 * FIFO at the beginning of packet memory.
 */

#define ERRATA5 1
#if ERRATA5
#  define PKTMEM_RX_START 0x0000                            /* RX buffer must be at addr 0 for errata 5 */
#  define PKTMEM_RX_END   (PKTMEM_END-ALIGNED_BUFSIZE)      /* RX buffer length is total SRAM minus TX buffer */
#  define PKTMEM_TX_START (PKTMEM_RX_END+1)                 /* Start TX buffer after */
#  define PKTMEM_TX_ENDP1 (PKTMEM_TX_START+ALIGNED_BUFSIZE) /* Allow TX buffer for one frame */
#else
#  define PKTMEM_TX_START 0x0000                            /* Start TX buffer at 0 */
#  define PKTMEM_TX_ENDP1 ALIGNED_BUFSIZE                   /* Allow TX buffer for one frame */
#  define PKTMEM_RX_START PKTMEM_TX_ENDP1                   /* Followed by RX buffer */
#  define PKTMEM_RX_END   PKTMEM_END                        /* RX buffer goes to the end of SRAM */
#endif

/* Misc. Helper Macros ******************************************************/

#define enc_rdgreg(priv,ctrlreg) \
  enc_rdgreg2(priv, ENC_RCR | GETADDR(ctrlreg))
#define enc_wrgreg(priv,ctrlreg,wrdata) \
  enc_wrgreg2(priv, ENC_WCR | GETADDR(ctrlreg), wrdata)
#define enc_bfcgreg(priv,ctrlreg,clrbits) \
  enc_wrgreg2(priv, ENC_BFC | GETADDR(ctrlreg), clrbits)
#define enc_bfsgreg(priv,ctrlreg,setbits) \
  enc_wrgreg2(priv, ENC_BFS | GETADDR(ctrlreg), setbits)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/* Debug ********************************************************************/

#ifdef CONFIG_ENC28J60_REGDEBUG
#  define enc_wrdump(a,v) \
   syslog(LOG_DEBUG, "ENC28J60: %02x<-%02x\n", a, v);
#  define enc_rddump(a,v) \
   syslog(LOG_DEBUG, "ENC28J60: %02x->%02x\n", a, v);
#  define enc_cmddump(c) \
   syslog(LOG_DEBUG, "ENC28J60: CMD: %02x\n", c);
#  define enc_bmdump(c,b,s) \
   syslog(LOG_DEBUG, "ENC28J60: CMD: %02x buffer: %p length: %d\n", c, b, s);
#else
#  define enc_wrdump(a,v)
#  define enc_rddump(a,v)
#  define enc_cmddump(c)
#  define enc_bmdump(c,b,s)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the interface */

enum enc_state_e
{
  ENCSTATE_UNINIT = 0,                /* The interface is in an uninitialized state */
  ENCSTATE_DOWN,                      /* The interface is down */
  ENCSTATE_UP                         /* The interface is up */
};

/* The enc_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct enc_driver_s
{
  /* Device control */

  uint8_t               ifstate;       /* Interface state:  See ENCSTATE_* */
  uint8_t               bank;          /* Currently selected bank */
  uint16_t              nextpkt;       /* Next packet address */
  FAR const struct enc_lower_s *lower; /* Low-level MCU-specific support */

  /* Timing */

  struct wdog_s         txpoll;        /* TX poll timer */
  struct wdog_s         txtimeout;     /* TX timeout timer */

  /* If we don't own the SPI bus, then we cannot do SPI accesses from the
   * interrupt handler.
   */

  struct work_s         irqwork;       /* Interrupt continuation work queue support */
  struct work_s         towork;        /* Tx timeout work queue support */
  struct work_s         pollwork;      /* Poll timeout work queue support */

  /* This is the contained SPI driver intstance */

  FAR struct spi_dev_s *spi;

  /* This holds the information visible to the NuttX network */

  struct net_driver_s   dev;          /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* Driver status structure */

static struct enc_driver_s g_enc28j60[CONFIG_ENC28J60_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

static inline void enc_configspi(FAR struct spi_dev_s *spi);
static void enc_lock(FAR struct enc_driver_s *priv);
static inline void enc_unlock(FAR struct enc_driver_s *priv);

/* SPI control register access */

static uint8_t enc_rdgreg2(FAR struct enc_driver_s *priv, uint8_t cmd);
static void enc_wrgreg2(FAR struct enc_driver_s *priv, uint8_t cmd,
         uint8_t wrdata);
static inline void enc_src(FAR struct enc_driver_s *priv);
static void enc_setbank(FAR struct enc_driver_s *priv, uint8_t bank);
static uint8_t enc_rdbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg);
static void enc_wrbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg,
         uint8_t wrdata);
static int enc_waitbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg,
         uint8_t bits, uint8_t value);

#if 0 /* Sometimes useful */
static void enc_rxdump(FAR struct enc_driver_s *priv);
static void enc_txdump(FAR struct enc_driver_s *priv);
#endif

/* SPI buffer transfers */

static void enc_rdbuffer(FAR struct enc_driver_s *priv, FAR uint8_t *buffer,
         size_t buflen);
static inline void enc_wrbuffer(FAR struct enc_driver_s *priv,
         FAR const uint8_t *buffer, size_t buflen);

/* PHY register access */

static uint16_t enc_rdphy(FAR struct enc_driver_s *priv, uint8_t phyaddr);
static void enc_wrphy(FAR struct enc_driver_s *priv, uint8_t phyaddr,
         uint16_t phydata);

/* Common TX logic */

static int  enc_transmit(FAR struct enc_driver_s *priv);
static int  enc_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void enc_linkstatus(FAR struct enc_driver_s *priv);
static void enc_txif(FAR struct enc_driver_s *priv);
static void enc_txerif(FAR struct enc_driver_s *priv);
static void enc_txerif(FAR struct enc_driver_s *priv);
static void enc_rxerif(FAR struct enc_driver_s *priv);
static void enc_rxdispatch(FAR struct enc_driver_s *priv);
static void enc_pktif(FAR struct enc_driver_s *priv);
static void enc_irqworker(FAR void *arg);
static int  enc_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void enc_toworker(FAR void *arg);
static void enc_txtimeout(wdparm_t arg);
static void enc_pollworker(FAR void *arg);
static void enc_polltimer(wdparm_t arg);

/* NuttX callback functions */

static int  enc_ifup(struct net_driver_s *dev);
static int  enc_ifdown(struct net_driver_s *dev);
static int  enc_txavail(struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int  enc_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
static int  enc_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/* Initialization */

static void enc_pwrsave(FAR struct enc_driver_s *priv);
static void enc_pwrfull(FAR struct enc_driver_s *priv);
static void enc_setmacaddr(FAR struct enc_driver_s *priv);
static int  enc_reset(FAR struct enc_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enc_configspi
 *
 * Description:
 *   Configure the SPI for use with the ENC28J60
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void enc_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ENC28J60. */

  SPI_SETMODE(spi, CONFIG_ENC28J60_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ENC28J60_FREQUENCY);
}

/****************************************************************************
 * Name: enc_lock
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_lock(FAR struct enc_driver_s *priv)
{
  /* Lock the SPI bus in case there are multiple devices competing for the
   * SPI bus.
   */

  SPI_LOCK(priv->spi, true);

  /* Now make sure that the SPI bus is configured for the ENC28J60 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(priv->spi, CONFIG_ENC28J60_SPIMODE);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, CONFIG_ENC28J60_FREQUENCY);
}

/****************************************************************************
 * Name: enc_unlock
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void enc_unlock(FAR struct enc_driver_s *priv)
{
  /* Relinquish the lock on the bus. */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: enc_rdgreg2
 *
 * Description:
 *   Read a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the global address register.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   cmd   - The full command to received (cmd | address)
 *
 * Returned Value:
 *   The value read from the register
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint8_t enc_rdgreg2(FAR struct enc_driver_s *priv, uint8_t cmd)
{
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);

  /* Select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the read command and collect the data.  The sequence requires
   * 16-clocks:  8 to clock out the cmd + 8 to clock in the data.
   */

  SPI_SEND(priv->spi, cmd);        /* Clock out the command */
  rddata = SPI_SEND(priv->spi, 0); /* Clock in the data */

  /* De-select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);

  enc_rddump(cmd, rddata);
  return rddata;
}

/****************************************************************************
 * Name: enc_wrgreg2
 *
 * Description:
 *   Write to a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the global address register.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   cmd    - The full command to received (cmd | address)
 *   wrdata - The data to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_wrgreg2(FAR struct enc_driver_s *priv, uint8_t cmd,
                        uint8_t wrdata)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the write command and data.  The sequence requires 16-clocks:
   * 8 to clock out the cmd + 8 to clock out the data.
   */

  SPI_SEND(priv->spi, cmd);    /* Clock out the command */
  SPI_SEND(priv->spi, wrdata); /* Clock out the data */

  /* De-select ENC28J60 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
  enc_wrdump(cmd, wrdata);
}

/****************************************************************************
 * Name: enc_src
 *
 * Description:
 *   Send the single byte system reset command (SRC).
 *
 *   "The System Reset Command (SRC) allows the host controller to issue a
 *    System Soft Reset command.  Unlike other SPI commands, the SRC is
 *    only a single byte command and does not operate on any register. The
 *    command is started by pulling the CS pin low. The SRC opcode is the
 *    sent, followed by a 5-bit Soft Reset command constant of 1Fh. The
 *    SRC operation is terminated by raising the CS pin."
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void enc_src(FAR struct enc_driver_s *priv)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the system reset command. */

  SPI_SEND(priv->spi, ENC_SRC);

  /* Check CLKRDY bit to see when the reset is complete.  There is an errata
   * that says the CLKRDY may be invalid.  We'll wait a couple of msec to
   * workaround this condition.
   *
   * Also, "After a System Reset, all PHY registers should not be read or
   * written to until at least 50 �s have passed since the Reset has ended.
   * All registers will revert to their Reset default values. The dual
   * port buffer memory will maintain state throughout the System Reset."
   */

  up_mdelay(2);
#if 0
  while ((enc_rdgreg(priv, ENC_ESTAT) & ESTAT_CLKRDY) != 0);
#endif

  /* De-select ENC28J60 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
  enc_cmddump(ENC_SRC);
}

/****************************************************************************
 * Name: enc_setbank
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 * Assumption:
 *   The caller has exclusive access to the SPI bus
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   bank   - The bank to select (0-3)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_setbank(FAR struct enc_driver_s *priv, uint8_t bank)
{
  /* Check if the bank setting has changed */

  if (bank != priv->bank)
    {
      /* Select bank 0 (just so that all of the bits are cleared) */

      enc_bfcgreg(priv, ENC_ECON1, ECON1_BSEL_MASK);

      /* Then OR in bits to get the correct bank */

      if (bank != 0)
        {
          enc_bfsgreg(priv, ENC_ECON1, (bank << ECON1_BSEL_SHIFT));
        }

      /* Then remember the bank setting */

      priv->bank = bank;
    }
}

/****************************************************************************
 * Name: enc_rdbreg
 *
 * Description:
 *   Read from a banked control register using the RCR command.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to read
 *
 * Returned Value:
 *   The byte read from the banked register
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint8_t enc_rdbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg)
{
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);

  /* Set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Re-select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the RCR command and collect the data.  How we collect the data
   * depends on if this is a PHY/CAN or not.  The normal sequence requires
   * 16-clocks:  8 to clock out the cmd and  8 to clock in the data.
   */

  SPI_SEND(priv->spi, ENC_RCR | GETADDR(ctrlreg)); /* Clock out the command */
  if (ISPHYMAC(ctrlreg))
    {
      /* The PHY/MAC sequence requires 24-clocks:  8 to clock out the cmd,
       * 8 dummy bits, and 8 to clock in the PHY/MAC data.
       */

      SPI_SEND(priv->spi, 0); /* Clock in the dummy byte */
    }

  rddata = SPI_SEND(priv->spi, 0);  /* Clock in the data */

  /* De-select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
  enc_rddump(ENC_RCR | GETADDR(ctrlreg), rddata);
  return rddata;
}

/****************************************************************************
 * Name: enc_wrbreg
 *
 * Description:
 *   Write to a banked control register using the WCR command.  Unlike
 *   reading, this same SPI sequence works for normal, MAC, and PHY
 *   registers.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to write
 *   wrdata  - The data to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_wrbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg,
                       uint8_t wrdata)
{
  DEBUGASSERT(priv && priv->spi);

  /* Set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Re-select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the WCR command and data.  The sequence requires 16-clocks:
   * 8 to clock out the cmd + 8 to clock out the data.
   */

  SPI_SEND(priv->spi, ENC_WCR | GETADDR(ctrlreg)); /* Clock out the command */
  SPI_SEND(priv->spi, wrdata);                     /* Clock out the data */

  /* De-select ENC28J60 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
  enc_wrdump(ENC_WCR | GETADDR(ctrlreg), wrdata);
}

/****************************************************************************
 * Name: enc_waitbreg
 *
 * Description:
 *   Wait until banked register bit(s) take a specific value (or a timeout
 *   occurs).
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to check
 *   bits    - The bits to check (a mask)
 *   value   - The value of the bits to return (value under mask)
 *
 * Returned Value:
 *   OK on success, negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int enc_waitbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg,
                        uint8_t bits, uint8_t value)
{
  clock_t start = clock_systime_ticks();
  clock_t elapsed;
  uint8_t rddata;

  /* Loop until the exit condition is met */

  do
    {
      /* Read the byte from the requested banked register */

      rddata  = enc_rdbreg(priv, ctrlreg);
      elapsed = clock_systime_ticks() - start;
    }
  while ((rddata & bits) != value && elapsed < ENC_POLLTIMEOUT);

  return (rddata & bits) == value ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: enc_txdump enc_rxdump
 *
 * Description:
 *   Dump registers associated with receiving or sending packets.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if 0 /* Sometimes useful */
static void enc_rxdump(FAR struct enc_driver_s *priv)
{
  syslog(LOG_DEBUG, "Rx Registers:\n");
  syslog(LOG_DEBUG, "  EIE:      %02x EIR:      %02x\n",
         enc_rdgreg(priv, ENC_EIE), enc_rdgreg(priv, ENC_EIR));
  syslog(LOG_DEBUG, "  ESTAT:    %02x ECON1:    %02x ECON2:    %02x\n",
         enc_rdgreg(priv, ENC_ESTAT), enc_rdgreg(priv, ENC_ECON1),
         enc_rdgreg(priv, ENC_ECON2));
  syslog(LOG_DEBUG, "  ERXST:    %02x %02x\n",
         enc_rdbreg(priv, ENC_ERXSTH), enc_rdbreg(priv, ENC_ERXSTL));
  syslog(LOG_DEBUG, "  ERXND:    %02x %02x\n",
         enc_rdbreg(priv, ENC_ERXNDH), enc_rdbreg(priv, ENC_ERXNDL));
  syslog(LOG_DEBUG, "  ERXRDPT:  %02x %02x\n",
         enc_rdbreg(priv, ENC_ERXRDPTH), enc_rdbreg(priv, ENC_ERXRDPTL));
  syslog(LOG_DEBUG, "  ERXFCON:  %02x EPKTCNT:  %02x\n",
         enc_rdbreg(priv, ENC_ERXFCON), enc_rdbreg(priv, ENC_EPKTCNT));
  syslog(LOG_DEBUG, "  MACON1:   %02x MACON3:   %02x\n",
         enc_rdbreg(priv, ENC_MACON1), enc_rdbreg(priv, ENC_MACON3));
  syslog(LOG_DEBUG, "  MAMXFL:   %02x %02x\n",
         enc_rdbreg(priv, ENC_MAMXFLH), enc_rdbreg(priv, ENC_MAMXFLL));
  syslog(LOG_DEBUG, "  MAADR:    %02x:%02x:%02x:%02x:%02x:%02x\n",
         enc_rdbreg(priv, ENC_MAADR1), enc_rdbreg(priv, ENC_MAADR2),
         enc_rdbreg(priv, ENC_MAADR3), enc_rdbreg(priv, ENC_MAADR4),
         enc_rdbreg(priv, ENC_MAADR5), enc_rdbreg(priv, ENC_MAADR6));
}
#endif

#if 0 /* Sometimes useful */
static void enc_txdump(FAR struct enc_driver_s *priv)
{
  syslog(LOG_DEBUG, "Tx Registers:\n");
  syslog(LOG_DEBUG, "  EIE:      %02x EIR:      %02x\n",
         enc_rdgreg(priv, ENC_EIE), enc_rdgreg(priv, ENC_EIR));
  syslog(LOG_DEBUG, "  ESTAT:    %02x ECON1:    %02x\n",
         enc_rdgreg(priv, ENC_ESTAT), enc_rdgreg(priv, ENC_ECON1));
  syslog(LOG_DEBUG, "  ETXST:    %02x %02x\n",
         enc_rdbreg(priv, ENC_ETXSTH), enc_rdbreg(priv, ENC_ETXSTL));
  syslog(LOG_DEBUG, "  ETXND:    %02x %02x\n",
         enc_rdbreg(priv, ENC_ETXNDH), enc_rdbreg(priv, ENC_ETXNDL));
  syslog(LOG_DEBUG, "  MACON1:   %02x MACON3:   %02x MACON4:   %02x\n",
         enc_rdbreg(priv, ENC_MACON1), enc_rdbreg(priv, ENC_MACON3),
         enc_rdbreg(priv, ENC_MACON4));
  syslog(LOG_DEBUG, "  MACON1:   %02x MACON3:   %02x MACON4:   %02x\n",
         enc_rdbreg(priv, ENC_MACON1), enc_rdbreg(priv, ENC_MACON3),
         enc_rdbreg(priv, ENC_MACON4));
  syslog(LOG_DEBUG, "  MABBIPG:  %02x MAIPG %02x %02x\n",
         enc_rdbreg(priv, ENC_MABBIPG), enc_rdbreg(priv, ENC_MAIPGH),
         enc_rdbreg(priv, ENC_MAIPGL));
  syslog(LOG_DEBUG, "  MACLCON1: %02x MACLCON2:   %02x\n",
         enc_rdbreg(priv, ENC_MACLCON1), enc_rdbreg(priv, ENC_MACLCON2));
  syslog(LOG_DEBUG, "  MAMXFL:   %02x %02x\n",
         enc_rdbreg(priv, ENC_MAMXFLH), enc_rdbreg(priv, ENC_MAMXFLL));
}
#endif

/****************************************************************************
 * Name: enc_rdbuffer
 *
 * Description:
 *   Read a buffer of data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to read into
 *   buflen  - The number of bytes to read
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Read pointer is set to the correct address
 *
 ****************************************************************************/

static void enc_rdbuffer(FAR struct enc_driver_s *priv, FAR uint8_t *buffer,
                         size_t buflen)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the read buffer memory command (ignoring the response) */

  SPI_SEND(priv->spi, ENC_RBM);

  /* Then read the buffer data */

  SPI_RECVBLOCK(priv->spi, buffer, buflen);

  /* De-select ENC28J60 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
  enc_bmdump(ENC_WBM, buffer, buflen);
}

/****************************************************************************
 * Name: enc_wrbuffer
 *
 * Description:
 *   Write a buffer of data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to write from
 *   buflen  - The number of bytes to write
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Read pointer is set to the correct address
 *
 ****************************************************************************/

static inline void enc_wrbuffer(FAR struct enc_driver_s *priv,
                                FAR const uint8_t *buffer, size_t buflen)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENC28J60 chip
   *
   * "The WBM command is started by lowering the CS pin. ..."
   */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);

  /* Send the write buffer memory command (ignoring the response)
   *
   * "...The [3-bit]WBM opcode should then be sent to the ENC28J60,
   *  followed by the 5-bit constant, 1Ah."
   */

  SPI_SEND(priv->spi, ENC_WBM);

  /* "...the ENC28J60 requires a single per packet control byte to
   * precede the packet for transmission."
   *
   * POVERRIDE: Per Packet Override bit (Not set):
   *   1 = The values of PCRCEN, PPADEN and PHUGEEN will override the
   *       configuration defined by MACON3.
   *   0 = The values in MACON3 will be used to determine how the packet
   *       will be transmitted
   * PCRCEN: Per Packet CRC Enable bit (Set, but won't be used because
   *   POVERRIDE is zero).
   * PPADEN: Per Packet Padding Enable bit (Set, but won't be used because
   *   POVERRIDE is zero).
   * PHUGEEN: Per Packet Huge Frame Enable bit (Set, but won't be used
   *   because POVERRIDE is zero).
   */

  SPI_SEND(priv->spi,
           (PKTCTRL_PCRCEN | PKTCTRL_PPADEN | PKTCTRL_PHUGEEN));

  /* Then send the buffer
   *
   * "... After the WBM command and constant are sent, the data to
   *  be stored in the memory pointed to by EWRPT should be shifted
   *  out MSb first to the ENC28J60. After 8 data bits are received,
   *  the Write Pointer will automatically increment if AUTOINC is
   *  set. The host controller can continue to provide clocks on the
   *  SCK pin and send data on the SI pin, without raising CS, to
   *  keep writing to the memory. In this manner, with AUTOINC
   *  enabled, it is possible to continuously write sequential bytes
   *  to the buffer memory without any extra SPI command
   *  overhead.
   */

  SPI_SNDBLOCK(priv->spi, buffer, buflen);

  /* De-select ENC28J60 chip
   *
   * "The WBM command is terminated by bringing up the CS pin. ..."
   */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
  enc_bmdump(ENC_WBM, buffer, buflen + 1);
}

/****************************************************************************
 * Name: enc_rdphy
 *
 * Description:
 *   Read 16-bits of PHY data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   phyaddr - The PHY register address
 *
 * Returned Value:
 *   16-bit value read from the PHY
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint16_t enc_rdphy(FAR struct enc_driver_s *priv, uint8_t phyaddr)
{
  uint16_t data = 0;

  /* "To read from a PHY register:
   *
   *   1. Write the address of the PHY register to read from into MIREGADR
   *      register.
   */

  enc_wrbreg(priv, ENC_MIREGADR, phyaddr);

  /*   2. Set the MICMD.MIIRD bit. The read operation begins and the
   *      MISTAT.BUSY bit is set.
   */

  enc_wrbreg(priv, ENC_MICMD, MICMD_MIIRD);

  /*   3. Wait 10.24 �s. Poll the MISTAT.BUSY bit to be certain that the
   *      operation is complete. While busy, the host controller should not
   *      start any MIISCAN operations or write to the MIWRH register.
   *
   *      When the MAC has obtained the register contents, the BUSY bit will
   *      clear itself.
   */

  up_udelay(12);
  if (enc_waitbreg(priv, ENC_MISTAT, MISTAT_BUSY, 0x00) == OK)
    {
      /* 4. Clear the MICMD.MIIRD bit. */

      enc_wrbreg(priv, ENC_MICMD, 0x00);

      /* 5. Read the desired data from the MIRDL and MIRDH registers. The
       *    order that these bytes are accessed is unimportant."
       */

      data  = (uint16_t)enc_rdbreg(priv, ENC_MIRDL);
      data |= (uint16_t)enc_rdbreg(priv, ENC_MIRDH) << 8;
    }

  return data;
}

/****************************************************************************
 * Name: enc_wrphy
 *
 * Description:
 *   write 16-bits of PHY data.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   phyaddr - The PHY register address
 *   phydata - 16-bit data to write to the PHY
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_wrphy(FAR struct enc_driver_s *priv, uint8_t phyaddr,
                      uint16_t phydata)
{
  /* "To write to a PHY register:
   *
   *    1. Write the address of the PHY register to write to into the
   *       MIREGADR register.
   */

  enc_wrbreg(priv, ENC_MIREGADR, phyaddr);

  /*    2. Write the lower 8 bits of data to write into the MIWRL register. */

  enc_wrbreg(priv, ENC_MIWRL, phydata);

  /*    3. Write the upper 8 bits of data to write into MIWRH register.
   *       Writing to this register automatically begins MIIM transaction,
   *       so it must be written to after MIWRL. The MISTAT.BUSY bit becomes
   *       set.
   */

  enc_wrbreg(priv, ENC_MIWRH, phydata >> 8);

  /*    The PHY register will be written after the MIIM operation completes,
   *    which takes 10.24 �s. When the write operation has completed, BUSY
   *    bit will clear itself.
   *
   *    The host controller should not start any MIISCAN or MIIRD operations
   *    while busy."
   */

  up_udelay(12);
  enc_waitbreg(priv, ENC_MISTAT, MISTAT_BUSY, 0x00);
}

/****************************************************************************
 * Name: enc_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from:
 *
 *   -  pkif interrupt when an application responds to the receipt of data
 *      by trying to send something, or
 *   -  From watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int enc_transmit(FAR struct enc_driver_s *priv)
{
  uint16_t txend;

  /* Increment statistics */

  ninfo("Sending packet, pktlen: %d\n", priv->dev.d_len);
  NETDEV_TXPACKETS(&priv->dev);

  /* Verify that the hardware is ready to send another packet.  The driver
   * starts a transmission process by setting ECON1.TXRTS. When the packet is
   * finished transmitting or is aborted due to an error/cancellation, the
   * ECON1.TXRTS bit will be cleared.
   *
   * NOTE: If we got here, then we have committed to sending a packet.
   * higher level logic must have assured that (1) there is no transmission
   * in progress, and that (2) TX-related interrupts are disabled.
   */

  DEBUGASSERT((enc_rdgreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0);

  /* Send the packet: address=priv->dev.d_buf, length=priv->dev.d_len */

  enc_dumppacket("Transmit Packet", priv->dev.d_buf, priv->dev.d_len);

  /* Set transmit buffer start (is this necessary?). */

  enc_wrbreg(priv, ENC_ETXSTL, PKTMEM_TX_START & 0xff);
  enc_wrbreg(priv, ENC_ETXSTH, PKTMEM_TX_START >> 8);

  /* Reset the write pointer to start of transmit buffer */

  enc_wrbreg(priv, ENC_EWRPTL, PKTMEM_TX_START & 0xff);
  enc_wrbreg(priv, ENC_EWRPTH, PKTMEM_TX_START >> 8);

  /* Set the TX End pointer based on the size of the packet to send. Note
   * that the offset accounts for the control byte at the beginning the
   * buffer plus the size of the packet data.
   */

  txend = PKTMEM_TX_START +  priv->dev.d_len;
  enc_wrbreg(priv, ENC_ETXNDL, txend & 0xff);
  enc_wrbreg(priv, ENC_ETXNDH, txend >> 8);

  /* Send the WBM command and copy the packet itself into the transmit
   * buffer at the position of the EWRPT register.
   */

  enc_wrbuffer(priv, priv->dev.d_buf, priv->dev.d_len);

  /* Set TXRTS to send the packet in the transmit buffer */

  enc_bfsgreg(priv, ENC_ECON1, ECON1_TXRTS);

  /* Setup the TX timeout watchdog (perhaps restarting the timer).  Note:
   * Is there a race condition.  Could the TXIF interrupt occur before
   * the timer is started?
   */

  wd_start(&priv->txtimeout, ENC_TXTIMEOUT,
           enc_txtimeout, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: enc_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is
 *      reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the network lock.
 *
 ****************************************************************************/

static int enc_txpoll(struct net_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  ninfo("Poll result: d_len=%d\n", priv->dev.d_len);
  if (priv->dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
        {
          arp_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          enc_transmit(priv);

          /* Stop the poll now because we can queue only one packet */

          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return OK;
}

/****************************************************************************
 * Name: enc_linkstatus
 *
 * Description:
 *   The current link status can be obtained from the PHSTAT1.LLSTAT or
 *   PHSTAT2.LSTAT.
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_linkstatus(FAR struct enc_driver_s *priv)
{
#if 0
  uint16_t regval = enc_rdphy(priv, ENC_PHSTAT2);
  priv->duplex    = ((regval & PHSTAT2_DPXSTAT) != 0);
  priv->carrier   = ((regval & PHSTAT2_LSTAT) != 0);
#endif
}

/****************************************************************************
 * Name: enc_txif
 *
 * Description:
 *   An TXIF interrupt was received indicating that the last TX packet(s) is
 *   done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the network lock.
 *
 ****************************************************************************/

static void enc_txif(FAR struct enc_driver_s *priv)
{
  /* Update statistics */

  NETDEV_TXDONE(&priv->dev);

  /* Clear the request to send bit */

  enc_bfcgreg(priv, ENC_ECON1, ECON1_TXRTS);

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(&priv->txtimeout);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, enc_txpoll);
}

/****************************************************************************
 * Name: enc_txerif
 *
 * Description:
 *   An TXERIF interrupt was received indicating that TX abort has occurred.
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

static void enc_txerif(FAR struct enc_driver_s *priv)
{
  /* Update statistics */

  NETDEV_TXERRORS(&priv->dev);

  /* Reset TX */

  enc_bfsgreg(priv, ENC_ECON1, ECON1_TXRST);
  enc_bfcgreg(priv, ENC_ECON1, ECON1_TXRST | ECON1_TXRTS);

  /* Here we really should re-transmit (I fact, if we want half duplex to
   * work right, then it is necessary to do this!):
   *
   * 1.  Read the TSV:
   *     - Read ETXNDL to get the end pointer
   *     - Read 7 bytes from that pointer + 1 using ENC_RMB.
   * 2. Determine if we need to retransmit.  Check the LATE COLLISION bit, if
   *    set, then we need to transmit.
   * 3. Retranmit by resetting ECON1_TXRTS.
   */

#ifdef CONFIG_ENC28J60_HALFDUPLEX
#  error "Missing logic for half duplex"
#endif
}

/****************************************************************************
 * Name: enc_rxerif
 *
 * Description:
 *   An RXERIF interrupt was received indicating that the last TX packet(s)
 *   is done
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

static void enc_rxerif(FAR struct enc_driver_s *priv)
{
  /* REVISIT: Update statistics */
}

/****************************************************************************
 * Name: enc_rxdispatch
 *
 * Description:
 *   Give the newly received packet to the network.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the network lock.
 *
 ****************************************************************************/

static void enc_rxdispatch(FAR struct enc_driver_s *priv)
{
#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->dev);
#endif

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->dev);

      /* Handle ARP on input then give the IPv4 packet to the network
       * layer
       */

      arp_ipin(&priv->dev);
      ipv4_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
          if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
            {
              arp_out(&priv->dev);
            }
#ifdef CONFIG_NET_IPv6
          else
            {
              neighbor_out(&priv->dev);
            }
#endif

          /* And send the packet */

          enc_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->dev);

      /* Give the IPv6 packet to the network layer */

      ipv6_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
          if (IFF_IS_IPv4(priv->dev.d_flags))
            {
              arp_out(&priv->dev);
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
            {
              neighbor_out(&priv->dev);
            }
#endif

          /* And send the packet */

          enc_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (BUF->type == htons(ETHTYPE_ARP))
    {
      ninfo("ARP packet received (%02x)\n", BUF->type);
      NETDEV_RXARP(&priv->dev);

      arp_arpin(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          enc_transmit(priv);
        }
    }
  else
#endif
    {
      nwarn("WARNING: Unsupported packet type dropped (%02x)\n",
            htons(BUF->type));
      NETDEV_RXDROPPED(&priv->dev);
    }
}

/****************************************************************************
 * Name: enc_pktif
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
 *   Interrupts are enabled but the caller holds the network lock.
 *
 ****************************************************************************/

static void enc_pktif(FAR struct enc_driver_s *priv)
{
  uint8_t  rsv[6];
  uint16_t pktlen;
  uint16_t rxstat;

  /* Update statistics */

  NETDEV_RXPACKETS(&priv->dev);

  /* Set the read pointer to the start of the received packet (ERDPT) */

  DEBUGASSERT(priv->nextpkt <= PKTMEM_RX_END);
  enc_wrbreg(priv, ENC_ERDPTL, (priv->nextpkt));
  enc_wrbreg(priv, ENC_ERDPTH, (priv->nextpkt) >> 8);

  /* Read the next packet pointer and the 4 byte read status vector (RSV)
   * at the beginning of the received packet. (ERDPT should auto-increment
   * and wrap to the beginning of the read buffer as necessary)
   */

  enc_rdbuffer(priv, rsv, 6);

  /* Decode the new next packet pointer, and the RSV.  The
   * RSV is encoded as:
   *
   *  Bits 0-15:  Indicates length of the received frame. This includes the
   *              destination address, source address, type/length, data,
   *              padding and CRC fields. This field is stored in little-
   *              endian format.
   *  Bits 16-31: Bit encoded RX status.
   */

  priv->nextpkt = (uint16_t)rsv[1] << 8 | (uint16_t)rsv[0];
  pktlen        = (uint16_t)rsv[3] << 8 | (uint16_t)rsv[2];
  rxstat        = (uint16_t)rsv[5] << 8 | (uint16_t)rsv[4];

  ninfo("Receiving packet, nextpkt: %04x pktlen: %d rxstat: %04x\n",
        priv->nextpkt, pktlen, rxstat);

  /* Check if the packet was received OK */

  if ((rxstat & RXSTAT_OK) == 0)
    {
      nerr("ERROR: RXSTAT: %04x\n", rxstat);
      NETDEV_RXERRORS(&priv->dev);
    }

  /* Check for a usable packet length (4 added for the CRC) */

  else if (pktlen > (CONFIG_NET_ETH_PKTSIZE + 4) ||
           pktlen <= (ETH_HDRLEN + 4))
    {
      nerr("ERROR: Bad packet size dropped (%d)\n", pktlen);
      NETDEV_RXERRORS(&priv->dev);
    }

  /* Otherwise, read and process the packet */

  else
    {
      /* Save the packet length (without the 4 byte CRC) in priv->dev.d_len */

      priv->dev.d_len = pktlen - 4;

      /* Copy the data data from the receive buffer to priv->dev.d_buf.
       * ERDPT should be correctly positioned from the last call to
       * end_rdbuffer (above).
       */

      enc_rdbuffer(priv, priv->dev.d_buf, priv->dev.d_len);
      enc_dumppacket("Received Packet", priv->dev.d_buf, priv->dev.d_len);

      /* Dispatch the packet to the network */

      enc_rxdispatch(priv);
    }

  /* Move the RX read pointer to the start of the next received packet.
   * This frees the memory we just read.
   */

  enc_wrbreg(priv, ENC_ERXRDPTL, (priv->nextpkt));
  enc_wrbreg(priv, ENC_ERXRDPTH, (priv->nextpkt) >> 8);

  /* Decrement the packet counter indicate we are done with this packet */

  enc_bfsgreg(priv, ENC_ECON2, ECON2_PKTDEC);
}

/****************************************************************************
 * Name: enc_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Input Parameters:
 *   arg     - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_irqworker(FAR void *arg)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  uint8_t eir;

  DEBUGASSERT(priv);

  /* Get exclusive access to both the network and the SPI bus. */

  net_lock();
  enc_lock(priv);

  /* Disable further interrupts by clearing the global interrupt enable bit.
   * "After an interrupt occurs, the host controller should clear the global
   * enable bit for the interrupt pin before servicing the interrupt.
   * Clearing the enable bit will cause the interrupt pin to return to the
   * non-asserted state (high). Doing so will prevent the host controller
   * from missing a falling edge should another interrupt occur while the
   * immediate interrupt is being serviced."
   */

  enc_bfcgreg(priv, ENC_EIE, EIE_INTIE);

  /* Loop until all interrupts have been processed (EIR==0).  Note that
   * there is no infinite loop check... if there are always pending
   * interrupts, we are just broken.
   */

  while ((eir = enc_rdgreg(priv, ENC_EIR) & EIR_ALLINTS) != 0)
    {
      /* Handle interrupts according to interrupt register register bit
       * settings.
       */

      ninfo("EIR: %02x\n", eir);

      /* DMAIF: The DMA interrupt indicates that the DMA module has completed
       * its memory copy or checksum calculation. Additionally, this
       * interrupt will be caused if the host controller cancels a DMA
       * operation by manually clearing the DMAST bit. Once set, DMAIF can
       * only be cleared by the host controller or by a Reset condition.
       */

      if ((eir & EIR_DMAIF) != 0) /* DMA interrupt */
        {
          /* Not used by this driver. Just clear the interrupt request. */

          enc_bfcgreg(priv, ENC_EIR, EIR_DMAIF);
        }

      /* LINKIF: The LINKIF indicates that the link status has changed.
       * The actual current link status can be obtained from the
       * PHSTAT1.LLSTAT or PHSTAT2.LSTAT. Unlike other interrupt sources, the
       * link status change interrupt is created in the integrated PHY
       * module.
       *
       * To receive it, the host controller must set the PHIE.PLNKIE and
       * PGEIE bits. After setting the two PHY interrupt enable bits, the
       * LINKIF bit will then shadow the contents of the PHIR.PGIF bit.
       *
       * Once LINKIF is set, it can only be cleared by the host controller or
       * by a Reset. The LINKIF bit is read-only. Performing an MII read on
       * the PHIR register will clear the LINKIF, PGIF and PLNKIF bits
       * automatically and allow for future link status change interrupts.
       */

      if ((eir & EIR_LINKIF) != 0) /* Link change interrupt */
        {
          enc_linkstatus(priv);       /* Get current link status */
          enc_rdphy(priv, ENC_PHIR);  /* Clear the LINKIF interrupt */
        }

      /* TXIF: The Transmit Interrupt Flag (TXIF) is used to indicate that
       * the requested packet transmission has ended. Upon transmission
       * completion, abort or transmission cancellation by the host
       * controller, the EIR.TXIF flag will be set to 1.
       *
       * Once TXIF is set, it can only be cleared by the host controller
       * or by a Reset condition. Once processed, the host controller should
       * use the BFC command to clear the EIR.TXIF bit.
       */

      if ((eir & EIR_TXIF) != 0) /* Transmit interrupt */
        {
          enc_txif(priv);                       /* Handle TX completion */
          enc_bfcgreg(priv, ENC_EIR, EIR_TXIF); /* Clear the TXIF interrupt */
        }

      /* TXERIF: The Transmit Error Interrupt Flag (TXERIF) is used to
       * indicate that a transmit abort has occurred. An abort can occur
       * because of any of the following:
       *
       * 1. Excessive collisions occurred as defined by the Retransmission
       *    Maximum (RETMAX) bits in the MACLCON1 register.
       * 2. A late collision occurred as defined by the Collision Window
       *   (COLWIN) bits in the MACLCON2 register.
       * 3. A collision after transmitting 64 bytes occurred (ESTAT.LATECOL
       *    set).
       * 4. The transmission was unable to gain an opportunity to transmit
       *    the packet because the medium was constantly occupied for too
       *    long.  The deferral limit (2.4287 ms) was reached and the
       *    MACON4.DEFER bit was clear.
       * 5. An attempt to transmit a packet larger than the maximum frame
       *    length defined by the MAMXFL registers was made without setting
       *    the MACON3.HFRMEN bit or per packet POVERRIDE and PHUGEEN bits.
       *
       * Upon any of these conditions, the EIR.TXERIF flag is set to 1. Once
       * set, it can only be cleared by the host controller or by a Reset
       * condition.
       *
       * After a transmit abort, the TXRTS bit will be cleared, the
       * ESTAT.TXABRT bit will be set and the transmit status vector will be
       * written at ETXND + 1. The MAC will not automatically attempt to
       * retransmit the packet. The host controller may wish to read the
       * transmit status vector and LATECOL bit to determine the cause of
       * the abort. After determining the problem and solution, the host
       * controller should clear the LATECOL (if set) and TXABRT bits so
       * that future aborts can be detected accurately.
       *
       * In Full-Duplex mode, condition 5 is the only one that should cause
       * this interrupt. Collisions and other problems related to sharing
       * the network are not possible on full-duplex networks. The conditions
       * which cause the transmit error interrupt meet the requirements of
       * the transmit interrupt. As a result, when this interrupt occurs,
       * TXIF will also be simultaneously set.
       */

      if ((eir & EIR_TXERIF) != 0) /* Transmit Error Interrupts */
        {
          enc_txerif(priv);                       /* Handle the TX error */
          enc_bfcgreg(priv, ENC_EIR, EIR_TXERIF); /* Clear the TXERIF interrupt */
        }

      /* PKTIF The Receive Packet Pending Interrupt Flag (PKTIF) is used to
       * indicate the presence of one or more data packets in the receive
       * buffer and to provide a notification means for the arrival of new
       * packets. When the receive buffer has at least one packet in it,
       * EIR.PKTIF will be set. In other words, this interrupt flag will be
       * set anytime the Ethernet Packet Count register (EPKTCNT) is
       * non-zero.
       *
       * The PKTIF bit can only be cleared by the host controller or by a
       * Reset condition. In order to clear PKTIF, the EPKTCNT register must
       * be decremented to 0. If the last data packet in the receive buffer
       * is processed, EPKTCNT will become zero and the PKTIF bit will
       * automatically be cleared.
       */

#if 0
      /* Ignore PKTIF because is unreliable. Use EPKTCNT instead */

      if ((eir & EIR_PKTIF) != 0)
#endif
        {
          uint8_t pktcnt = enc_rdbreg(priv, ENC_EPKTCNT);
          if (pktcnt > 0)
            {
              ninfo("EPKTCNT: %02x\n", pktcnt);

              /* Handle packet receipt */

              enc_pktif(priv);
            }
        }

      /* RXERIF: The Receive Error Interrupt Flag (RXERIF) is used to
       * indicate a receive buffer overflow condition. Alternately, this
       * interrupt may indicate that too many packets are in the receive
       * buffer and more cannot be stored without overflowing the EPKTCNT
       * register.  When a packet is being received and the receive buffer
       * runs completely out of space, or EPKTCNT is 255 and cannot be
       * incremented, the packet being received will be aborted (permanently
       * lost) and the EIR.RXERIF bit will be set to 1.
       *
       * Once set, RXERIF can only be cleared by the host controller or by a
       * Reset condition. Normally, upon the receive error condition, the
       * host controller would process any packets pending from the receive
       * buffer and then make additional room for future packets by
       * advancing the ERXRDPT registers (low byte first) and decrementing
       * the EPKTCNT register.
       *
       * Once processed, the host controller should use the BFC command to
       * clear the EIR.RXERIF bit.
       */

      if ((eir & EIR_RXERIF) != 0) /* Receive Error Interrupts */
        {
          enc_rxerif(priv);                       /* Handle the RX error */
          enc_bfcgreg(priv, ENC_EIR, EIR_RXERIF); /* Clear the RXERIF interrupt */
        }
    }

  /* Enable GPIO interrupts */

  priv->lower->enable(priv->lower);

  /* Enable Ethernet interrupts */

  enc_bfsgreg(priv, ENC_EIE, EIE_INTIE);

  /* Release lock on the SPI bus and the network */

  enc_unlock(priv);
  net_unlock();
}

/****************************************************************************
 * Name: enc_interrupt
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

static int enc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct enc_driver_s *priv;

  DEBUGASSERT(arg != NULL);
  priv = (FAR struct enc_driver_s *)arg;

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&priv->irqwork));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  priv->lower->disable(priv->lower);
  return work_queue(ENCWORK, &priv->irqwork, enc_irqworker,
                    (FAR void *)priv, 0);
}

/****************************************************************************
 * Name: enc_toworker
 *
 * Description:
 *   Our TX watchdog timed out.  This is the worker thread continuation of
 *   the watchdog timer interrupt.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg     - The reference to the driver structure (case to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_toworker(FAR void *arg)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  int ret;

  nerr("ERROR: Tx timeout\n");
  DEBUGASSERT(priv);

  /* Get exclusive access to the network */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Then reset the hardware: Take the interface down, then bring it
   * back up
   */

  ret = enc_ifdown(&priv->dev);
  DEBUGASSERT(ret == OK);
  ret = enc_ifup(&priv->dev);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, enc_txpoll);

  /* Release lock on the network */

  net_unlock();
}

/****************************************************************************
 * Name: enc_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Perform work on the worker thread.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_txtimeout(wdparm_t arg)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timeout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv && work_available(&priv->towork));

  /* Notice that Tx timeout watchdog is not active so further Tx timeouts
   * can occur until we restart the Tx timeout watchdog.
   */

  ret = work_queue(ENCWORK, &priv->towork, enc_toworker, priv, 0);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name: enc_pollworker
 *
 * Description:
 *   Periodic timer handler continuation.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_pollworker(FAR void *arg)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;

  DEBUGASSERT(priv);

  /* Get exclusive access to both the network and the SPI bus. */

  net_lock();
  enc_lock(priv);

  /* Verify that the hardware is ready to send another packet.  The driver
   * start a transmission process by setting ECON1.TXRTS. When the packet is
   * finished transmitting or is aborted due to an error/cancellation, the
   * ECON1.TXRTS bit will be cleared.
   */

  if ((enc_rdgreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0)
    {
      /* Yes.. update TCP timing states and poll the network for new XMIT
       * data.  Hmmm.. looks like a bug here to me.  Does this mean if there
       * is a transmit in progress, we will missing TCP time state updates?
       */

      devif_timer(&priv->dev, ENC_WDDELAY, enc_txpoll);
    }

  /* Release lock on the SPI bus and the network */

  enc_unlock(priv);
  net_unlock();

  /* Setup the watchdog poll timer again */

  wd_start(&priv->txpoll, ENC_WDDELAY,
           enc_polltimer, (wdparm_t)arg);
}

/****************************************************************************
 * Name: enc_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_polltimer(wdparm_t arg)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timeout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv && work_available(&priv->pollwork));

  /* Notice that poll watchdog is not active so further poll timeouts can
   * occur until we restart the poll timeout watchdog.
   */

  ret = work_queue(ENCWORK, &priv->pollwork, enc_pollworker,
                   (FAR void *)priv, 0);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name: enc_ifup
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
 *
 ****************************************************************************/

static int enc_ifup(struct net_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Initialize Ethernet interface, set the MAC address, and make sure that
   * the ENC28J80 is not in power save mode.
   */

  ret = enc_reset(priv);
  if (ret == OK)
    {
      enc_setmacaddr(priv);
      enc_pwrfull(priv);

      /* Enable interrupts at the ENC28J60.  Interrupts are still disabled
       * at the interrupt controller.
       */

      enc_wrphy(priv, ENC_PHIE, PHIE_PGEIE | PHIE_PLNKIE);
      enc_bfcgreg(priv, ENC_EIR, EIR_ALLINTS);
      enc_wrgreg(priv, ENC_EIE, EIE_INTIE  | EIE_PKTIE  | EIE_LINKIE |
                                EIE_TXIE   | EIE_TXERIE | EIE_RXERIE);

      /* Enable the receiver */

      enc_bfsgreg(priv, ENC_ECON1, ECON1_RXEN);

      /* Set and activate a timer process */

      wd_start(&priv->txpoll, ENC_WDDELAY,
               enc_polltimer, (wdparm_t)priv);

      /* Mark the interface up and enable the Ethernet interrupt at the
       * controller
       */

      priv->ifstate = ENCSTATE_UP;
      priv->lower->enable(priv->lower);
    }

  /* Un-lock the SPI bus */

  enc_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: enc_ifdown
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

static int enc_ifdown(struct net_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;
  irqstate_t flags;
  int ret;

  ninfo("Taking down: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  priv->lower->disable(priv->lower);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txpoll);
  wd_cancel(&priv->txtimeout);

  /* Reset the device and leave in the power save state */

  ret = enc_reset(priv);
  enc_pwrsave(priv);

  priv->ifstate = ENCSTATE_DOWN;
  leave_critical_section(flags);

  /* Un-lock the SPI bus */

  enc_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: enc_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int enc_txavail(struct net_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Ignore the notification if the interface is not yet up */

  flags = enter_critical_section();
  if (priv->ifstate == ENCSTATE_UP)
    {
      /* Check if the hardware is ready to send another packet.  The driver
       * starts a transmission process by setting ECON1.TXRTS. When the
       * packet is finished transmitting or is aborted due to an error/
       * cancellation, the ECON1.TXRTS bit will be cleared.
       */

      if ((enc_rdgreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0)
        {
          /* The interface is up and TX is idle;
           * poll the network for new XMIT data
           */

          devif_poll(&priv->dev, enc_txpoll);
        }
    }

  /* Un-lock the SPI bus */

  leave_critical_section(flags);
  enc_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: enc_addmac
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
static int enc_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"

  /* Un-lock the SPI bus */

  enc_unlock(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: enc_rmmac
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
static int enc_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"

  /* Un-lock the SPI bus */

  enc_unlock(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: enc_pwrsave
 *
 * Description:
 *   The ENC28J60 may be commanded to power-down via the SPI interface.
 *   When powered down, it will no longer be able to transmit and receive
 *   any packets. To maximize power savings:
 *
 *   1. Turn off packet reception by clearing ECON1.RXEN.
 *   2. Wait for any in-progress packets to finish being received by
 *      polling ESTAT.RXBUSY. This bit should be clear before proceeding.
 *   3. Wait for any current transmissions to end by confirming ECON1.TXRTS
 *      is clear.
 *   4. Set ECON2.VRPS (if not already set).
 *   5. Enter Sleep by setting ECON2.PWRSV. All MAC, MII and PHY registers
 *      become inaccessible as a result. Setting PWRSV also clears
 *      ESTAT.CLKRDY automatically.
 *
 *   In Sleep mode, all registers and buffer memory will maintain their
 *   states. The ETH registers and buffer memory will still be accessible
 *   by the host controller. Additionally, the clock driver will continue
 *   to operate. The CLKOUT function will be unaffected.
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

static void enc_pwrsave(FAR struct enc_driver_s *priv)
{
  ninfo("Set PWRSV\n");

  /* 1. Turn off packet reception by clearing ECON1.RXEN. */

  enc_bfcgreg(priv, ENC_ECON1, ECON1_RXEN);

  /* 2. Wait for any in-progress packets to finish being received by
   *    polling ESTAT.RXBUSY. This bit should be clear before proceeding.
   */

  if (enc_waitbreg(priv, ENC_ESTAT, ESTAT_RXBUSY, 0) == OK)
    {
      /* 3. Wait for any current transmissions to end by confirming
       * ECON1.TXRTS is clear.
       */

      enc_waitbreg(priv, ENC_ECON1, ECON1_TXRTS, 0);

      /* 4. Set ECON2.VRPS (if not already set).
       *    (Set in enc_reset()
       *
       * 5. Enter Sleep by setting ECON2.PWRSV.
       */

      enc_bfsgreg(priv, ENC_ECON2, ECON2_PWRSV);
    }
}

/****************************************************************************
 * Name: enc_pwrfull
 *
 * Description:
 *   When normal operation is desired, the host controller must perform
 *   a slightly modified procedure:
 *
 *   1. Wake-up by clearing ECON2.PWRSV.
 *   2. Wait at least 300 �s for the PHY to stabilize. To accomplish the
 *      delay, the host controller may poll ESTAT.CLKRDY and wait for it
 *      to become set.
 *   3. Restore receive capability by setting ECON1.RXEN.
 *
 *   After leaving Sleep mode, there is a delay of many milliseconds
 *   before a new link is established (assuming an appropriate link
 *   partner is present). The host controller may wish to wait until
 *   the link is established before attempting to transmit any packets.
 *   The link status can be determined by polling the PHSTAT2.LSTAT bit.
 *   Alternatively, the link change interrupt may be used if it is
 *   enabled.
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

static void enc_pwrfull(FAR struct enc_driver_s *priv)
{
  ninfo("Clear PWRSV\n");

  /* 1. Wake-up by clearing ECON2.PWRSV. */

  enc_bfcgreg(priv, ENC_ECON2, ECON2_PWRSV);

  /* 2. Wait at least 300 �s for the PHY to stabilize. To accomplish the
   * delay, the host controller may poll ESTAT.CLKRDY and wait for it to
   * become set.
   */

  enc_waitbreg(priv, ENC_ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY);

  /* 3. Restore receive capability by setting ECON1.RXEN.
   *
   * The caller will do this when it is read to receive packets
   */
}

/****************************************************************************
 * Name: enc_setmacaddr
 *
 * Description:
 *   Set the MAC address to the configured value.  This is done after ifup
 *   or after a TX timeout.  Note that this means that the interface must
 *   be down before configuring the MAC addr.
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

static void enc_setmacaddr(FAR struct enc_driver_s *priv)
{
  /* Program the hardware with it's MAC address (for filtering).
   *   MAADR1  MAC Address Byte 1 (MAADR<47:40>), OUI Byte 1
   *   MAADR2  MAC Address Byte 2 (MAADR<39:32>), OUI Byte 2
   *   MAADR3  MAC Address Byte 3 (MAADR<31:24>), OUI Byte 3
   *   MAADR4  MAC Address Byte 4 (MAADR<23:16>)
   *   MAADR5  MAC Address Byte 5 (MAADR<15:8>)
   *   MAADR6  MAC Address Byte 6 (MAADR<7:0>)
   */

  enc_wrbreg(priv, ENC_MAADR1, priv->dev.d_mac.ether.ether_addr_octet[0]);
  enc_wrbreg(priv, ENC_MAADR2, priv->dev.d_mac.ether.ether_addr_octet[1]);
  enc_wrbreg(priv, ENC_MAADR3, priv->dev.d_mac.ether.ether_addr_octet[2]);
  enc_wrbreg(priv, ENC_MAADR4, priv->dev.d_mac.ether.ether_addr_octet[3]);
  enc_wrbreg(priv, ENC_MAADR5, priv->dev.d_mac.ether.ether_addr_octet[4]);
  enc_wrbreg(priv, ENC_MAADR6, priv->dev.d_mac.ether.ether_addr_octet[5]);
}

/****************************************************************************
 * Name: enc_reset
 *
 * Description:
 *   Stop, reset, re-initialize, and restart the ENC28J60.  This is done
 *   initially, on ifup, and after a TX timeout.
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

static int enc_reset(FAR struct enc_driver_s *priv)
{
  uint8_t regval;

  nwarn("WARNING: Reset\n");

  /* Configure SPI for the ENC28J60 */

  enc_configspi(priv->spi);

  /* Reset the ENC28J60 */

  enc_src(priv);

  /* Initialize ECON1: Clear ECON1 */

  enc_wrgreg(priv, ENC_ECON1, 0x00);

  /* Initialize ECON2: Enable address auto increment and voltage
   * regulator powersave.
   */

  enc_wrgreg(priv, ENC_ECON2, ECON2_AUTOINC | ECON2_VRPS);

  /* Initialize receive buffer.
   * First, set the receive buffer start address.
   */

  priv->nextpkt = PKTMEM_RX_START;
  enc_wrbreg(priv, ENC_ERXSTL, PKTMEM_RX_START & 0xff);
  enc_wrbreg(priv, ENC_ERXSTH, PKTMEM_RX_START >> 8);

  /* Set the receive data pointer */

  enc_wrbreg(priv, ENC_ERXRDPTL, PKTMEM_RX_START & 0xff);
  enc_wrbreg(priv, ENC_ERXRDPTH, PKTMEM_RX_START >> 8);

  /* Set the receive buffer end. */

  enc_wrbreg(priv, ENC_ERXNDL, PKTMEM_RX_END & 0xff);
  enc_wrbreg(priv, ENC_ERXNDH, PKTMEM_RX_END >> 8);

  /* Set transmit buffer start. */

  enc_wrbreg(priv, ENC_ETXSTL, PKTMEM_TX_START & 0xff);
  enc_wrbreg(priv, ENC_ETXSTH, PKTMEM_TX_START >> 8);

  /* Check if we are actually communicating with the ENC28J60.  If its
   * 0x00 or 0xff, then we are probably not communicating correctly
   * via SPI.
   */

  regval = enc_rdbreg(priv, ENC_EREVID);
  if (regval == 0x00 || regval == 0xff)
    {
      nerr("ERROR: Bad Rev ID: %02x\n", regval);
      return -ENODEV;
    }

  ninfo("Rev ID: %02x\n", regval);

  /* Set filter mode: unicast OR broadcast AND crc valid */

  enc_wrbreg(priv, ENC_ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN |
                                ERXFCON_BCEN);

  /* Enable MAC receive */

  enc_wrbreg(priv, ENC_MACON1, MACON1_MARXEN | MACON1_TXPAUS |
                               MACON1_RXPAUS);

  /* Enable automatic padding and CRC operations */

#ifdef CONFIG_ENC28J60_HALFDUPLEX
  enc_wrbreg(priv, ENC_MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN |
                               MACON3_FRMLNEN);
  enc_wrbreg(priv, ENC_MACON4, MACON4_DEFER);        /* Defer transmission enable */

  /* Set Non-Back-to-Back Inter-Packet Gap */

  enc_wrbreg(priv, ENC_MAIPGL, 0x12);
  enc_wrbreg(priv, ENC_MAIPGH, 0x0c);

  /* Set Back-to-Back Inter-Packet Gap */

  enc_wrbreg(priv, ENC_MABBIPG, 0x12);
#else
  /* Set filter mode: unicast OR broadcast AND crc valid AND Full Duplex */

  enc_wrbreg(priv, ENC_MACON3,
             MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN |
             MACON3_FULDPX);

  /* Set Non-Back-to-Back Inter-Packet Gap */

  enc_wrbreg(priv, ENC_MAIPGL, 0x12);

  /* Set Back-to-Back Inter-Packet Gap */

  enc_wrbreg(priv, ENC_MABBIPG, 0x15);
#endif

  /* Set the maximum packet size which the controller will accept */

  enc_wrbreg(priv, ENC_MAMXFLL, CONFIG_NET_ETH_PKTSIZE & 0xff);
  enc_wrbreg(priv, ENC_MAMXFLH, CONFIG_NET_ETH_PKTSIZE >> 8);

  /* Configure LEDs (No, just use the defaults for now) */

  /* Setup up PHCON1 & 2 */

#ifdef CONFIG_ENC28J60_HALFDUPLEX
  enc_wrphy(priv, ENC_PHCON1, 0x00);
  enc_wrphy(priv, ENC_PHCON2, PHCON2_HDLDIS);
#else
  enc_wrphy(priv, ENC_PHCON1, PHCON1_PDPXMD);
  enc_wrphy(priv, ENC_PHCON2, 0x00);
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enc_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.  The ENC28J60 device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Input Parameters:
 *   spi   - A reference to the platform's SPI driver for the ENC28J60
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., ENC28J60 GPIO interrupts).
 *   devno - If more than one ENC28J60 is supported, then this is the
 *           zero based number that identifies the ENC28J60;
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int enc_initialize(FAR struct spi_dev_s *spi,
                   FAR const struct enc_lower_s *lower, unsigned int devno)
{
  FAR struct enc_driver_s *priv;

  DEBUGASSERT(devno < CONFIG_ENC28J60_NINTERFACES);
  priv = &g_enc28j60[devno];

  /* Initialize the driver structure */

  memset(g_enc28j60, 0,
         CONFIG_ENC28J60_NINTERFACES * sizeof(struct enc_driver_s));

  priv->dev.d_buf     = g_pktbuf;     /* Single packet buffer */
  priv->dev.d_ifup    = enc_ifup;     /* I/F down callback */
  priv->dev.d_ifdown  = enc_ifdown;   /* I/F up (new IP address) callback */
  priv->dev.d_txavail = enc_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = enc_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = enc_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = priv;         /* Used to recover private state from dev */
  priv->spi           = spi;          /* Save the SPI instance */
  priv->lower         = lower;        /* Save the low-level MCU interface */

  /* The interface should be in the down state.  However, this function is
   * called too early in initialization to perform the ENC28J60 reset in
   * enc_ifdown.  We are depending upon the fact that the application level
   * logic will call enc_ifdown later to reset the ENC28J60.  NOTE:  The MAC
   * address will not be set up until enc_ifup() is called. That gives the
   * app time to set the MAC address before bringing the interface up.
   */

  priv->ifstate = ENCSTATE_UNINIT;

  /* Attach the interrupt to the driver (but don't enable it yet) */

  if (lower->attach(lower, enc_interrupt, priv) < 0)
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  return netdev_register(&priv->dev, NET_LL_ETHERNET);
}

#endif /* CONFIG_NET && CONFIG_ENC28J60_NET */
