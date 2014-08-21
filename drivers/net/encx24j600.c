/****************************************************************************
 * drivers/net/encx24j600.c
 *
 *   Copyright (C) 2013-2014 UVC Ingenieure. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
 *
 * References:
 * - ENC424J600/624J600 Data Sheet, Stand-Alone 10/100 Ethernet Controller
 *   with SPI or Parallel Interface, DS39935C, 2010 Microchip Technology Inc.
 *
 * Derived from enc28j60 driver written by:
 *
 *   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#if defined(CONFIG_NET) && defined(CONFIG_ENCX24J600)

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <queue.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/encx24j600.h>

#include "encx24j600.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* ENCX24J600 Configuration Settings:
 *
 * CONFIG_ENCX24J600 - Enabled ENCX24J600 support
 * CONFIG_ENCX24J600_SPIMODE - Controls the SPI mode
 * CONFIG_ENCX24J600_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ENCX24J600_NINTERFACES - Specifies the number of physical ENCX24J600
 *   devices that will be supported.
 * CONFIG_ENCX24J600_STATS - Collect network statistics
 */

/* The ENCX24J600 spec says that it supports SPI mode 0,0 only: "The
 * implementation used on this device supports SPI mode 0,0 only. In
 * addition, the SPI port requires that SCK be at Idle in a low state;
 * selectable clock polarity is not supported."  However, sometimes you
 * need to tinker with these things.
 */

#ifndef CONFIG_ENCX24J600_SPIMODE
#  define CONFIG_ENCX24J600_SPIMODE SPIDEV_MODE0
#endif

/* CONFIG_ENCX24J600_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_ENCX24J600_NINTERFACES
#  define CONFIG_ENCX24J600_NINTERFACES 1
#endif

/* CONFIG_NET_BUFSIZE must always be defined */

#if !defined(CONFIG_NET_BUFSIZE) && (CONFIG_NET_BUFSIZE <= MAX_FRAMELEN)
#  error "CONFIG_NET_BUFSIZE is not valid for the ENCX24J600"
#endif

/* We need to have the work queue to handle SPI interrupts */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* CONFIG_ENCX24J600_DUMPPACKET will dump the contents of each packet to the console. */

#ifdef CONFIG_ENCX24J600_DUMPPACKET
#  define enc_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define enc_dumppacket(m,a,n)
#endif

/* The ENCX24J600 will not do interrupt level processing */

#ifndef CONFIG_NET_NOINTS
#  warning "CONFIG_NET_NOINTS should be set"
#endif

/* Low-level register debug */

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_NET)
#  undef CONFIG_ENCX24J600_REGDEBUG
#endif

/* Timing *******************************************************************/

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define ENC_WDDELAY   (1*CLK_TCK)
#define ENC_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define ENC_TXTIMEOUT (60*CLK_TCK)

/* RX timeout (Time packets are held in the RX queue until they are dropped) */

#define ENC_RXTIMEOUT MSEC2TICK(2000)

/* Poll timeout */

#define ENC_POLLTIMEOUT MSEC2TICK(50)

/* Register poll timeout */

#define ENC_REGPOLLTIMEOUT MSEC2TICK(5000)

/* Packet Memory ************************************************************/

/* Packet memory layout */

#define PKTMEM_ALIGNED_BUFSIZE ((CONFIG_NET_BUFSIZE + 1) & ~1)
#define PKTMEM_RX_START (PKTMEM_START + PKTMEM_SIZE / 2)   /* Followed by RX buffer */
#define PKTMEM_RX_SIZE  (PKTMEM_SIZE - PKTMEM_RX_START)
#define PKTMEM_RX_END   (PKTMEM_START + PKTMEM_SIZE)       /* RX buffer goes to the end of SRAM */

/* We use preinitialized TX descriptors */

#define ENC_NTXDESCR ((PKTMEM_RX_START - PKTMEM_START) / PKTMEM_ALIGNED_BUFSIZE)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/* Debug ********************************************************************/

#ifdef CONFIG_ENCX24J600_REGDEBUG
#  define enc_wrdump(a,v)   lowsyslog("ENCX24J600: %02x<-%04x\n", a, v);
#  define enc_rddump(a,v)   lowsyslog("ENCX24J600: %02x->%04x\n", a, v);
#  define enc_bfsdump(a,m)  lowsyslog("ENCX24J600: %02x|=%04x\n", a, m);
#  define enc_bfcdump(a,m)  lowsyslog("ENCX24J600: %02x&=~%04x\n", a, m);
#  define enc_cmddump(c)    lowsyslog("ENCX24J600: CMD: %02x\n", c);
#  define enc_bmdump(c,b,s) lowsyslog("ENCX24J600: CMD: %02x buffer: %p length: %d\n", c,b,s);
#else
#  define enc_wrdump(a,v)
#  define enc_rddump(a,v)
#  define enc_bfsdump(a,m)
#  define enc_bfcdump(a,m)
#  define enc_cmddump(c)
#  define enc_bmdump(c,b,s)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the interface */

enum enc_state_e
{
  ENCSTATE_UNINIT = 0,                 /* The interface is in an uninitialized state */
  ENCSTATE_DOWN,                       /* The interface is down */
  ENCSTATE_UP,                         /* The interface is up */
  ENCSTATE_RUNNING                     /* The interface is has a cable plugged in and is ready to use */
};

struct enc_descr_s
{
  struct enc_descr_next *flink;
  uint16_t addr;
  uint16_t len;
  uint32_t ts;                         /* Timestamp of reception for timeout */
};

/* The enc_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct enc_driver_s
{
  /* Device control */

  uint8_t               ifstate;       /* Interface state:  See ENCSTATE_* */
  uint8_t               bank;          /* Currently selected bank command */
  uint16_t              nextpkt;       /* Next packet address */
  FAR const struct enc_lower_s *lower; /* Low-level MCU-specific support */

  /* Timing */

  WDOG_ID               txpoll;        /* TX poll timer */
  WDOG_ID               txtimeout;     /* TX timeout timer */

  /* If we don't own the SPI bus, then we cannot do SPI accesses from the
   * interrupt handler.
   */

  struct work_s         irqwork;       /* Interrupt continuation work queue support */
  struct work_s         towork;        /* Tx timeout work queue support */
  struct work_s         pollwork;      /* Poll timeout work queue support */

  struct enc_descr_s    txdescralloc[ENC_NTXDESCR];
  struct enc_descr_s    rxdescralloc[CONFIG_ENCX24J600_NRXDESCR];

  sq_queue_t            txfreedescr;   /* Free inititialized TX descriptors */
  sq_queue_t            rxfreedescr;   /* Free RX descriptors */
  sq_queue_t            txqueue;       /* Enqueued descriptors waiting for transmition */
  sq_queue_t            rxqueue;       /* Unhandled incoming packets waiting for reception */

  /* This is the contained SPI driver intstance */

  FAR struct spi_dev_s *spi;

  /* This holds the information visible to uIP/NuttX */

  struct net_driver_s   dev;           /* Interface understood by uIP */

  /* Statistics */

#ifdef CONFIG_ENCX24J600_STATS
  struct enc_stats_s    stats;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct enc_driver_s g_encx24j600[CONFIG_ENCX24J600_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void enc_configspi(FAR struct spi_dev_s *spi);
#  define enc_lock(priv);
#  define enc_unlock(priv);
#else
#  define enc_configspi(spi)
static void enc_lock(FAR struct enc_driver_s *priv);
static inline void enc_unlock(FAR struct enc_driver_s *priv);
#endif

/* SPI control register access */

static inline void enc_setethrst(FAR struct enc_driver_s *priv);
static void enc_setbank(FAR struct enc_driver_s *priv, uint8_t bank);
static uint16_t enc_rdreg(FAR struct enc_driver_s *priv, uint16_t ctrlreg);
static void enc_wrreg(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                         uint16_t wrdata);
static int enc_waitreg(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                          uint16_t bits, uint16_t value);
static void enc_bfs(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                    uint16_t bits);
static void enc_bfc(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                    uint16_t bits);
static void enc_cmd(FAR struct enc_driver_s *priv, uint8_t cmd, uint16_t arg);

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

static int  enc_txenqueue(FAR struct enc_driver_s *priv);
static int  enc_transmit(FAR struct enc_driver_s *priv);
static int  enc_txpoll(struct net_driver_s *dev);

/* Common RX logic */

static struct enc_descr_s *enc_rxgetdescr(FAR struct enc_driver_s *priv);
static void enc_rxldpkt(FAR struct enc_driver_s *priv, struct enc_descr_s *descr);
static void enc_rxrmpkt(FAR struct enc_driver_s *priv, struct enc_descr_s *descr);
static void enc_rxdispatch(FAR struct enc_driver_s *priv);

/* Interrupt handling */

static void enc_linkstatus(FAR struct enc_driver_s *priv);
static void enc_txif(FAR struct enc_driver_s *priv);
static void enc_pktif(FAR struct enc_driver_s *priv);
static void enc_rxabtif(FAR struct enc_driver_s *priv);
static void enc_irqworker(FAR void *arg);
static int  enc_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void enc_toworker(FAR void *arg);
static void enc_txtimeout(int argc, uint32_t arg, ...);
static void enc_pollworker(FAR void *arg);
static void enc_polltimer(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int  enc_ifup(struct net_driver_s *dev);
static int  enc_ifdown(struct net_driver_s *dev);
static int  enc_txavail(struct net_driver_s *dev);
static int  enc_rxavail(struct net_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int  enc_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
static int  enc_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/* Initialization */

static void enc_pwrsave(FAR struct enc_driver_s *priv);
static void enc_setmacaddr(FAR struct enc_driver_s *priv);
static void enc_resetbuffers(FAR struct enc_driver_s *priv);
static int  enc_reset(FAR struct enc_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: enc_configspi
 *
 * Description:
 *   Configure the SPI for use with the ENCX24J600
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void enc_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ENCX24J600.  But only if we own the SPI bus.
   * Otherwise, don't bother because it might change.
   */

  SPI_SETMODE(spi, CONFIG_ENCX24J600_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_ENCX24J600_FREQUENCY);
}
#endif

/****************************************************************************
 * Function: enc_lock
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static void enc_lock(FAR struct enc_driver_s *priv)
{
  /* Lock the SPI bus in case there are multiple devices competing for the SPI
   * bus.
   */

  SPI_LOCK(priv->spi, true);

  /* Now make sure that the SPI bus is configured for the ENCX24J600 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(priv->spi, CONFIG_ENCX24J600_SPIMODE);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETFREQUENCY(priv->spi, CONFIG_ENCX24J600_FREQUENCY);
}
#endif

/****************************************************************************
 * Function: enc_unlock
 *
 * Description:
 *   De-select the SPI
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void enc_unlock(FAR struct enc_driver_s *priv)
{
  /* Relinquish the lock on the bus. */

  SPI_LOCK(priv->spi, false);
}
#endif

/****************************************************************************
 * Function: enc_cmd
 *
 * Description:
 *   Execute two byte command.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   cmd     - ENCX24J600 two-byte command
 *   arg     - Two byte argument to the command
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_cmd(FAR struct enc_driver_s *priv, uint8_t cmd, uint16_t arg)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENCX24J600 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);;

  (void)SPI_SEND(priv->spi, cmd);          /* Clock out the command */
  (void)SPI_SEND(priv->spi, arg & 0xff);   /* Clock out the low byte */
  (void)SPI_SEND(priv->spi, arg >> 8);     /* Clock out the high byte */

  /* De-select ENCX24J600 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_wrdump(cmd, arg);
}

/****************************************************************************
 * Function: enc_setethrst
 *
 * Description:
 *   Issues System Reset by setting ETHRST (ECON2<4>)
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void enc_setethrst(FAR struct enc_driver_s *priv)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENC28J60 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);;

  /* Send the system reset command. */

  (void)SPI_SEND(priv->spi, ENC_SETETHRST);

  up_udelay(25);

  /* De-select ENC28J60 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_cmddump(ENC_SETETHRST);
}

/****************************************************************************
 * Function: enc_setbank
 *
 * Description:
 *   Set the bank for the next control register access.
 *
 * Assumption:
 *   The caller has exclusive access to the SPI bus
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *   bank   - SPI command to select the bank with
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The chip is selected and SPI is ready for communication.
 *
 ****************************************************************************/

static void enc_setbank(FAR struct enc_driver_s *priv, uint8_t bank)
{

  /* Check if a bank has to be set and if the bank setting has changed.
   * For registers that are available on all banks, the bank command is set to 0.
   */

  if (bank != 0 && bank != priv->bank)
    {
      /* Select bank with supplied command */

      SPI_SEND(priv->spi, bank);

      /* Then remember the bank setting */

      priv->bank = bank;
    }
}

/****************************************************************************
 * Function: enc_rdreg
 *
 * Description:
 *   Read one word from a control register using the RCR command.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to read
 *
 * Returned Value:
 *   The byte read from the banked register
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint16_t enc_rdreg(FAR struct enc_driver_s *priv, uint16_t ctrlreg)
{
  uint16_t rddata;

  DEBUGASSERT(priv && priv->spi);
  DEBUGASSERT((ctrlreg & 0xe0) == 0); /* banked regeitsers only */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);

  enc_setbank(priv, GETBANK(ctrlreg));

  SPI_SEND(priv->spi, ENC_RCR | GETADDR(ctrlreg));

  rddata = SPI_SEND(priv->spi, 0);        /* Clock in the low byte */
  rddata |= SPI_SEND(priv->spi, 0) << 8;  /* Clock in the high byte */


  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_rddump(GETADDR(ctrlreg), rddata);

  return rddata;
}

/****************************************************************************
 * Function: enc_wrreg
 *
 * Description:
 *   Write one word to a control register using the WCR command.
 *
 * Parameters:
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

static void enc_wrreg(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                      uint16_t wrdata)
{
  DEBUGASSERT(priv && priv->spi);
  DEBUGASSERT((ctrlreg & 0xe0) == 0); /* banked regeitsers only */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);;

  enc_setbank(priv, GETBANK(ctrlreg));

  SPI_SEND(priv->spi, ENC_WCR | GETADDR(ctrlreg));
  SPI_SEND(priv->spi, wrdata & 0xff); /* Clock out the low byte */
  SPI_SEND(priv->spi, wrdata >> 8);   /* Clock out the high byte */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_wrdump(GETADDR(ctrlreg), wrdata);
}

/****************************************************************************
 * Function: enc_waitbreg
 *
 * Description:
 *   Wait until banked register bit(s) take a specific value (or a timeout
 *   occurs).
 *
 * Parameters:
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

static int enc_waitreg(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                          uint16_t bits, uint16_t value)
{
  uint32_t start = clock_systimer();
  uint32_t elapsed;
  uint16_t rddata;

  /* Loop until the exit condition is met */

  do
    {
      /* Read the byte from the requested banked register */

      rddata  = enc_rdreg(priv, ctrlreg);
      elapsed = clock_systimer() - start;
    }
  while ((rddata & bits) != value && elapsed < ENC_REGPOLLTIMEOUT);

  return (rddata & bits) == value ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Function: enc_bfs
 *
 * Description:
 *   Bit Field Set.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to set bits in
 *   bits    - The bits to set (a mask)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_bfs(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                    uint16_t bits)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENCX24J600 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);;

  /* Set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Send the BFS command and data.  The sequence requires 24-clocks:
   * 8 to clock out the cmd + 16 to clock out the data.
   */

  (void)SPI_SEND(priv->spi, ENC_BFS | GETADDR(ctrlreg)); /* Clock out the command */
  (void)SPI_SEND(priv->spi, bits & 0xff);                /* Clock out the low byte */
  (void)SPI_SEND(priv->spi, bits >> 8);                  /* Clock out the high byte */

  /* De-select ENCX24J600 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_bfsdump(GETADDR(ctrlreg), bits);
}

/****************************************************************************
 * Function: enc_bfc
 *
 * Description:
 *   Bit Field Clear.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   ctrlreg - Bit encoded address of banked register to clear bits in
 *   bits    - The bits to clear (a mask)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_bfc(FAR struct enc_driver_s *priv, uint16_t ctrlreg,
                    uint16_t bits)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENCX24J600 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);;

  /* Set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Send the BFC command and data. The sequence requires 24-clocks:
   * 8 to clock out the cmd + 16 to clock out the data.
   */

  (void)SPI_SEND(priv->spi, ENC_BFC | GETADDR(ctrlreg)); /* Clock out the command */
  (void)SPI_SEND(priv->spi, bits & 0xff);                /* Clock out the low byte */
  (void)SPI_SEND(priv->spi, bits >> 8);                  /* Clock out the high byte */

  /* De-select ENCX24J600 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_bfcdump(GETADDR(ctrlreg), bits);
}

/****************************************************************************
 * Function: enc_txdump enc_rxdump
 *
 * Description:
 *   Dump registers associated with receiving or sending packets.
 *
 * Parameters:
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
  lowsyslog("Rx Registers:\n");
  lowsyslog("  EIE:      %02x EIR:      %02x\n",
            enc_rdgreg(priv, ENC_EIE), enc_rdgreg(priv, ENC_EIR));
  lowsyslog("  ESTAT:    %02x ECON1:    %02x ECON2:    %02x\n",
            enc_rdgreg(priv, ENC_ESTAT), enc_rdgreg(priv, ENC_ECON1),
            enc_rdgreg(priv, ENC_ECON2));
  lowsyslog("  ERXST:    %02x %02x\n",
            enc_rdbreg(priv, ENC_ERXSTH), enc_rdbreg(priv, ENC_ERXSTL));
  lowsyslog("  ERXND:    %02x %02x\n",
            enc_rdbreg(priv, ENC_ERXNDH), enc_rdbreg(priv, ENC_ERXNDL));
  lowsyslog("  ERXRDPT:  %02x %02x\n",
            enc_rdbreg(priv, ENC_ERXRDPTH), enc_rdbreg(priv, ENC_ERXRDPTL));
  lowsyslog("  ERXFCON:  %02x EPKTCNT:  %02x\n",
            enc_rdbreg(priv, ENC_ERXFCON), enc_rdbreg(priv, ENC_EPKTCNT));
  lowsyslog("  MACON1:   %02x MACON3:   %02x\n",
            enc_rdbreg(priv, ENC_MACON1), enc_rdbreg(priv, ENC_MACON3));
  lowsyslog("  MAMXFL:   %02x %02x\n",
            enc_rdbreg(priv, ENC_MAMXFLH), enc_rdbreg(priv, ENC_MAMXFLL));
  lowsyslog("  MAADR:    %02x:%02x:%02x:%02x:%02x:%02x\n",
            enc_rdbreg(priv, ENC_MAADR1), enc_rdbreg(priv, ENC_MAADR2),
            enc_rdbreg(priv, ENC_MAADR3), enc_rdbreg(priv, ENC_MAADR4),
            enc_rdbreg(priv, ENC_MAADR5), enc_rdbreg(priv, ENC_MAADR6));
}
#endif

#if 0 /* Sometimes useful */
static void enc_txdump(FAR struct enc_driver_s *priv)
{
  lowsyslog("Tx Registers:\n");
  lowsyslog("  EIE:      %02x EIR:      %02x ESTAT:    %02x\n",
            enc_rdgreg(priv, ENC_EIE), enc_rdgreg(priv, ENC_EIR),);
  lowsyslog("  ESTAT:    %02x ECON1:    %02x\n",
            enc_rdgreg(priv, ENC_ESTAT), enc_rdgreg(priv, ENC_ECON1));
  lowsyslog("  ETXST:    %02x %02x\n",
            enc_rdbreg(priv, ENC_ETXSTH), enc_rdbreg(priv, ENC_ETXSTL));
  lowsyslog("  ETXND:    %02x %02x\n",
            enc_rdbreg(priv, ENC_ETXNDH), enc_rdbreg(priv, ENC_ETXNDL));
  lowsyslog("  MACON1:   %02x MACON3:   %02x MACON4:   %02x\n",
            enc_rdbreg(priv, ENC_MACON1), enc_rdbreg(priv, ENC_MACON3),
            enc_rdbreg(priv, ENC_MACON4));
  lowsyslog("  MACON1:   %02x MACON3:   %02x MACON4:   %02x\n",
            enc_rdbreg(priv, ENC_MACON1), enc_rdbreg(priv, ENC_MACON3),
            enc_rdbreg(priv, ENC_MACON4));
  lowsyslog("  MABBIPG:  %02x MAIPG %02x %02x\n",
            enc_rdbreg(priv, ENC_MABBIPG), enc_rdbreg(priv, ENC_MAIPGH),
            enc_rdbreg(priv, ENC_MAIPGL));
  lowsyslog("  MACLCON1: %02x MACLCON2:   %02x\n",
            enc_rdbreg(priv, ENC_MACLCON1), enc_rdbreg(priv, ENC_MACLCON2));
  lowsyslog("  MAMXFL:   %02x %02x\n",
            enc_rdbreg(priv, ENC_MAMXFLH), enc_rdbreg(priv, ENC_MAMXFLL));
}
#endif

/****************************************************************************
 * Function: enc_rdbuffer
 *
 * Description:
 *   Read a buffer of data from RX Data Buffer.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to read into
 *   buflen  - The number of bytes to read
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   RX Data pointer is set to the correct address
 *
 ****************************************************************************/

static void enc_rdbuffer(FAR struct enc_driver_s *priv, FAR uint8_t *buffer,
                         size_t buflen)
{
  DEBUGASSERT(priv && priv->spi);

  /* Select ENCX24J600 chip */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);

  /* Send the read buffer memory command (ignoring the response) */

  (void)SPI_SEND(priv->spi, ENC_RRXDATA);

  /* Then read the buffer data */

  SPI_RECVBLOCK(priv->spi, buffer, buflen);

  /* De-select ENCX24J600 chip. */

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_bmdump(ENC_RRXDATA, buffer, buflen);
}

/****************************************************************************
 * Function: enc_wrbuffer
 *
 * Description:
 *   Write a buffer of data.
 *
 * Parameters:
 *   priv    - Reference to the driver state structure
 *   buffer  - A pointer to the buffer to write from
 *   buflen  - The number of bytes to write
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   General Purpose Write pointer is set to the correct address
 *
 ****************************************************************************/

static inline void enc_wrbuffer(FAR struct enc_driver_s *priv,
                                FAR const uint8_t *buffer, size_t buflen)
{
  DEBUGASSERT(priv && priv->spi);

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, true);;

  SPI_SEND(priv->spi, ENC_WGPDATA);
  SPI_SNDBLOCK(priv->spi, buffer, buflen);

  SPI_SELECT(priv->spi, SPIDEV_ETHERNET, false);
  enc_bmdump(ENC_WGPDATA, buffer, buflen);
}

/****************************************************************************
 * Function: enc_rdphy
 *
 * Description:
 *   Read 16-bits of PHY data.
 *
 * Parameters:
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
   *   1. Write the address of the PHY register to read from into the MIREGADR
   *      register (Register 3-1). Make sure to also set reserved bit 8 of this
   *      register.
   */

  enc_wrreg(priv, ENC_MIREGADR, phyaddr);

  /*   2. Set the MIIRD bit (MICMD<0>, Register 3-2). The read operation begins
   *      and the BUSY bit (MISTAT<0>, Register 3-3) is automatically set by
   *      hardware.
   */

  enc_bfs(priv, ENC_MICMD, MICMD_MIIRD);

  /*   3. Wait 25.6 μs. Poll the BUSY (MISTAT<0>) bit to be certain that the
   *      operation is complete. While busy, the host controller should not
   *      start any MIISCAN operations or write to the MIWR register. When the
   *      MAC has obtained the register contents, the BUSY bit will clear
   *      itself.
   */

  up_udelay(26);
  if (enc_waitreg(priv, ENC_MISTAT, MISTAT_BUSY, 0x00) == OK)
    {
      /* 4. Clear the MIIRD (MICMD<0>) bit. */

      enc_bfc(priv, ENC_MICMD, MICMD_MIIRD);

      /* 5. Read the desired data from the MIRD register. For 8-bit interfaces,
       *    the order that these bytes are read is unimportant."
       */

      data = enc_rdreg(priv, ENC_MIRD);
    }

  return data;
}

/****************************************************************************
 * Function: enc_wrphy
 *
 * Description:
 *   write 16-bits of PHY data.
 *
 * Parameters:
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
   *    1. Write the address of the PHY register to write to into the MIREGADR
   *       register. Make sure to also set reserved bit 8 of this register.
   */

  enc_wrreg(priv, ENC_MIREGADR, 0x0100 | phyaddr);

  /*    2. Write the 16 bits of data into the MIWR register. The low byte must
   *       be written first, followed by the high byte.
   */

  enc_wrreg(priv, ENC_MIWR, phydata);

  /*    3. Writing to the high byte of MIWR begins the MIIM transaction and the
   *       BUSY (MISTAT<0>) bit is automatically set by hardware.
   *
   * The PHY register is written after the MIIM operation completes, which takes
   * 25.6 μs. When the write operation has completed, the BUSY bit clears
   * itself. The host controller should not start any MIISCAN, MIWR or MIIRD
   * operations while the BUSY bit is set.
   */

  up_udelay(26);
  enc_waitreg(priv, ENC_MISTAT, MISTAT_BUSY, 0);
}

/****************************************************************************
 * Function: enc_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from:
 *
 *   -  pkif interrupt when an application responds to the receipt of data
 *      by trying to send something, or
 *   -  From watchdog based polling.
 *
 * Parameters:
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
  struct enc_descr_s *descr;

  /* dequeue next packet to transmit */

  descr = (struct enc_descr_s*)sq_remfirst(&priv->txqueue);

  DEBUGASSERT(descr != NULL);

  /* Verify that the hardware is ready to send another packet.  The driver
   * starts a transmission process by setting ECON1.TXRTS. When the packet is
   * finished transmitting or is aborted due to an error/cancellation, the
   * ECON1.TXRTS bit will be cleared.
   *
   * NOTE: If we got here, then we have committed to sending a packet.
   * higher level logic must have assured that (1) there is no transmission
   * in progress, and that (2) TX-related interrupts are disabled.
   */

  DEBUGASSERT((enc_rdreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0);

  /* Set TXStart and TXLen registers. */

  enc_wrreg(priv, ENC_ETXST, descr->addr);
  enc_wrreg(priv, ENC_ETXLEN, descr->len);

  /* Set TXRTS to send the packet in the transmit buffer */

  enc_bfs(priv, ENC_ECON1, ECON1_TXRTS);

  /* Setup the TX timeout watchdog (perhaps restarting the timer).  Note:
   * Is there a race condition.  Could the TXIF interrupt occur before
   * the timer is started?
   */

  (void)wd_start(priv->txtimeout, ENC_TXTIMEOUT, enc_txtimeout, 1,
                (uint32_t)priv);

  /* free the descriptor */

  sq_addlast((sq_entry_t*)descr, &priv->txfreedescr);

  return OK;
}

/****************************************************************************
 * Function: enc_txenqueue
 *
 * Description:
 *   Write packet from d_buf to the enc's SRAM if a free descriptor is available.
 *   The filled descriptor is enqueued for transmission.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   A packet is available in d_buf.
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static int enc_txenqueue(FAR struct enc_driver_s *priv)
{
  int ret = OK;
  struct enc_descr_s *descr;

  DEBUGASSERT(priv->dev.d_len > 0);

  /* Increment statistics */

#ifdef CONFIG_ENCX24J600_STATS
  priv->stats.txrequests++;
#endif

  descr = (struct enc_descr_s*)sq_remfirst(&priv->txfreedescr);

  if (descr != NULL)
    {
      enc_dumppacket("Write packet to enc SRAM", priv->dev.d_buf,
                     priv->dev.d_len);

      /* Copy the packet into the transmit buffer described by the current
       * tx descriptor
       */

      enc_cmd(priv, ENC_WGPWRPT, descr->addr);
      enc_wrbuffer(priv, priv->dev.d_buf, priv->dev.d_len);

      /* store packet length */

      descr->len = priv->dev.d_len;

      /* enqueue packet */

      sq_addlast((sq_entry_t*)descr, &priv->txqueue);

      /* if currently no transmission is active, trigger the transmission */

      if ((enc_rdreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0)
        {
          enc_transmit(priv);
        }
    }
  else
    {
      nlldbg("no free descriptors\n");
      ret = -ENOMEM;
    }

  return ret;
}

/****************************************************************************
 * Function: enc_txpoll
 *
 * Description:
 *   Enqueues uIP packets if available.
 *   This is a callback from devif_poll().  devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timedout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static int enc_txpoll(struct net_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;
  int ret = OK;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  nllvdbg("Poll result: d_len=%d\n", priv->dev.d_len);

  if (priv->dev.d_len > 0)
    {
      arp_out(&priv->dev);

      ret = enc_txenqueue(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return ret;
}

/****************************************************************************
 * Function: enc_linkstatus
 *
 * Description:
 *   The current link status can be obtained from the PHSTAT1.LLSTAT or
 *   PHSTAT2.LSTAT.
 *
 * Parameters:
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
  uint16_t regval;

  /* Before transmitting the first packet after link establishment or
   * auto-negotiation, the MAC duplex configuration must be manually set to
   * match the duplex configuration of the PHY. To do this, configure
   * FULDPX (MACON2<0>) to match PHYDPX (ESTAT<10>).
   */

  regval = enc_rdreg(priv, ENC_ESTAT);

  if (regval & ESTAT_PHYLNK)
    {
      if (regval & ESTAT_PHYDPX)
        {
          /* Configure full-duplex */

          enc_wrreg(priv, ENC_MABBIPG, 0x15);
          enc_bfs(priv, ENC_MACON2, MACON2_FULDPX);
        }
      else
        {
          /* Configure half-duplex */

          enc_wrreg(priv, ENC_MABBIPG, 0x12);
          enc_bfc(priv, ENC_MACON2, MACON2_FULDPX);
        }

      netdev_carrier_on(&priv->dev);
      priv->ifstate = ENCSTATE_RUNNING;
    }
  else
    {
      netdev_carrier_off(&priv->dev);
      priv->ifstate = ENCSTATE_UP;
    }
}

/****************************************************************************
 * Function: enc_txif
 *
 * Description:
 *   An TXIF interrupt was received indicating that the last TX packet(s) is
 *   done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void enc_txif(FAR struct enc_driver_s *priv)
{
  if (sq_empty(&priv->txqueue))
    {
      /* If no further xmits are pending, then cancel the TX timeout */

      wd_cancel(priv->txtimeout);

      /* Poll for TX packets from the networking layer */

      devif_poll(&priv->dev, enc_txpoll);
    }
  else
    {
      /* process txqueue */

      enc_transmit(priv);
    }
}

/****************************************************************************
 * Function: enc_rxldpkt
 *
 * Description:
 *   Load packet from the enc's RX buffer to the driver d_buf.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   descr - Reference to the descriptor that should be loaded
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void enc_rxldpkt(FAR struct enc_driver_s *priv,
                        struct enc_descr_s *descr)
{
  DEBUGASSERT(priv != NULL && descr != NULL);

  nllvdbg("load packet @%04x len: %d\n", descr->addr, descr->len);

  /* Set the rx data pointer to the start of the received packet (ERXRDPT) */

  enc_cmd(priv, ENC_WRXRDPT, descr->addr);

  /* Save the packet length (without the 4 byte CRC) in priv->dev.d_len */

  priv->dev.d_len = descr->len - 4;

  /* Copy the data data from the receive buffer to priv->dev.d_buf  */

  enc_rdbuffer(priv, priv->dev.d_buf, priv->dev.d_len);

  enc_dumppacket("loaded RX packet", priv->dev.d_buf, priv->dev.d_len);
}

/****************************************************************************
 * Function: enc_rxgetdescr
 *
 * Description:
 *   Check for a free descriptor in the free list. If no free descriptor is
 *   available a pending descriptor will be freed and returned
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   A free rx descriptor
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static struct enc_descr_s *enc_rxgetdescr(FAR struct enc_driver_s *priv)
{
  if (sq_empty(&priv->rxfreedescr))
    {
      DEBUGASSERT(sq_peek(&priv->rxqueue) != NULL);

      /* Packets are held in the enc's SRAM until the space is needed */

      enc_rxrmpkt(priv, (struct enc_descr_s*)sq_peek(&priv->rxqueue));
    }

  return (struct enc_descr_s*)sq_remfirst(&priv->rxfreedescr);
}

/****************************************************************************
 * Function: enc_rxrmpkt
 *
 * Description:
 *   Remove packet from the RX queue and free the block of memory in the enc's
 *   SRAM.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   descr - Reference to the descriptor that should be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void enc_rxrmpkt(FAR struct enc_driver_s *priv, struct enc_descr_s *descr)
{
  uint16_t addr;

  nllvdbg("free descr: %p\n", descr);

  /* If it is the last descriptor in the queue, advance ERXTAIL.
   * This way it is possible that gaps occcur. Maybe pending packets
   * can be reordered th enc's DMA to free RX space?
   */

  if (descr != NULL)
    {
      if (descr == (struct enc_descr_s*)sq_peek(&priv->rxqueue))
        {
          /* Wrap address properly around */
          addr = (descr->addr - PKTMEM_RX_START + descr->len - 2 + PKTMEM_RX_SIZE)
            % PKTMEM_RX_SIZE + PKTMEM_RX_START;

          DEBUGASSERT(addr >= PKTMEM_RX_START &&  addr < PKTMEM_RX_END);

          nllvdbg("ERXTAIL %04x\n", addr);

          enc_wrreg(priv, ENC_ERXTAIL, addr);

          /* Remove packet from RX queue */

          sq_remfirst(&priv->rxqueue);
        }
      else
        {
          /* Remove packet from RX queue */

          sq_rem((sq_entry_t*)descr, &priv->rxqueue);
        }

      sq_addlast((sq_entry_t*)descr, &priv->rxfreedescr);
    }
}

/****************************************************************************
 * Function: enc_rxdispatch
 *
 * Description:
 *   Give the newly received packet to uIP.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void enc_rxdispatch(FAR struct enc_driver_s *priv)
{
  struct enc_descr_s *descr;
  struct enc_descr_s *next;

  int ret = ERROR;

  /* Process the RX queue */

  descr = (struct enc_descr_s*)sq_peek(&priv->rxqueue);

  while (descr != NULL)
    {
      /* Store the next pointer, because removing the item from list will set
       * flink to NULL
       */

      next = (struct enc_descr_s*)sq_next(descr);

      /* Load the packet from the enc's SRAM */

      enc_rxldpkt(priv, descr);

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(ETHTYPE_IP))
#endif
        {
          nllvdbg("Try to process IP packet (%02x)\n", BUF->type);

          arp_ipin(&priv->dev);
          ret = devif_input(&priv->dev);

          if (ret == OK || (clock_systimer() - descr->ts) > ENC_RXTIMEOUT)
            {
              /* If packet has been sucessfully processed or has timed out,
               * free it.
               */

              enc_rxrmpkt(priv, descr);
            }

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              arp_out(&priv->dev);
              enc_txenqueue(priv);
            }
        }
      else if (BUF->type == htons(ETHTYPE_ARP))
        {
          nllvdbg("ARP packet received (%02x)\n", BUF->type);
          arp_arpin(&priv->dev);

          /* ARP packets are freed immediately */

          enc_rxrmpkt(priv, descr);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              enc_txenqueue(priv);
            }
        }
      else
        {
          /* free unsupported packet */

          enc_rxrmpkt(priv, descr);

          nlldbg("Unsupported packet type dropped (%02x)\n", htons(BUF->type));
        }

      descr = next;
    }
}

/****************************************************************************
 * Function: enc_pktif
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void enc_pktif(FAR struct enc_driver_s *priv)
{
  struct enc_descr_s *descr;
  uint8_t  rsv[8];
  uint16_t pktlen;
  uint32_t rxstat;
  uint16_t curpkt;
  int pktcnt;

  DEBUGASSERT(priv->nextpkt >= PKTMEM_RX_START && priv->nextpkt < PKTMEM_RX_END);

  /* Enqueue all pending packets to the RX queue until PKTCNT == 0 or
   * no more descriptors are available.
   */

  pktcnt = (enc_rdreg(priv, ENC_ESTAT) & ESTAT_PKTCNT_MASK) >> ESTAT_PKTCNT_SHIFT;

  while (pktcnt > 0)
    {
      curpkt = priv->nextpkt;

      /* Set the rx data pointer to the start of the received packet (ERXRDPT) */

      enc_cmd(priv, ENC_WRXRDPT, curpkt);

      /* Read the next packet pointer and the 6 byte read status vector (RSV)
       * at the beginning of the received packet. (ERXRDPT should auto-increment
       * and wrap to the beginning of the read buffer as necessary)
       */

      enc_rdbuffer(priv, rsv, 8);

      /* Decode the new next packet pointer, and the RSV.  The
       * RSV is encoded as:
       *
       *  Bits 0-15:  Indicates length of the received frame. This includes the
       *              destination address, source address, type/length, data,
       *              padding and CRC fields. This field is stored in little-
       *              endian format.
       *  Bits 16-47: Bit encoded RX status.
       */

      priv->nextpkt = (uint16_t)rsv[1] << 8  | (uint16_t)rsv[0];
      pktlen        = (uint16_t)rsv[3] << 8  | (uint16_t)rsv[2];
      rxstat        = (uint32_t)rsv[7] << 24 | (uint32_t)rsv[6] << 16 |
                      (uint32_t)rsv[5] << 8  | (uint32_t)rsv[4];

      nllvdbg("Receiving packet, nextpkt: %04x pktlen: %d rxstat: %08x pktcnt: %d\n",
             priv->nextpkt, pktlen, rxstat, pktcnt);

      /* We enqueue the packet first and remove it later if its faulty.
       * This way we avoid freeing packets that are not processed yet.
       */

      descr = enc_rxgetdescr(priv);

      /* Set current timestamp */

      descr->ts = clock_systimer();

      /* Store the start address of the frame without the enc's header */

      descr->addr = curpkt + 8;
      descr->len = pktlen;
      sq_addlast((sq_entry_t*)descr, &priv->rxqueue);

      /* Check if the packet was received OK */

      if ((rxstat & RXSTAT_OK) == 0)
        {
          nlldbg("ERROR: RXSTAT: %08x\n", rxstat);

          /* Discard packet */

          enc_rxrmpkt(priv, descr);

#ifdef CONFIG_ENCX24J600_STATS
          priv->stats.rxnotok++;
#endif
        }

      /* Check for a usable packet length (4 added for the CRC) */

      else if (pktlen > (CONFIG_NET_BUFSIZE + 4) || pktlen <= (NET_LL_HDRLEN + 4))
        {
          nlldbg("Bad packet size dropped (%d)\n", pktlen);

          /* Discard packet */

          enc_rxrmpkt(priv, descr);

#ifdef CONFIG_ENCX24J600_STATS
          priv->stats.rxpktlen++;
#endif
        }

      /* Decrement PKTCNT */

      enc_bfs(priv, ENC_ECON1, ECON1_PKTDEC);

      /* Try to process the packet */

      enc_rxdispatch(priv);

      /* Read out again, maybe there has another packet arrived */

      pktcnt = (enc_rdreg(priv, ENC_ESTAT) & ESTAT_PKTCNT_MASK) >> ESTAT_PKTCNT_SHIFT;
    }
}

/****************************************************************************
 * Function: enc_rxabtif
 *
 * Description:
 *   An interrupt was received indicating the abortion of an RX packet
 *
 *   "The receive abort interrupt occurs when the reception of a frame has been
 *   aborted. A frame being received is aborted when the Head Pointer attempts
 *   to overrun the Tail Pointer, or when the packet counter has reached FFh.
 *   In either case, the receive buffer is full and cannot fit the incoming
 *   frame, so the packet has been dropped.
 *   This interrupt does not occur when packets are dropped due to the receive
 *   filters rejecting a packet. The interrupt should be cleared by software
 *   once it has been serviced."
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are enabled but the caller holds the uIP lock.
 *
 ****************************************************************************/

static void enc_rxabtif(FAR struct enc_driver_s *priv)
{
  struct enc_descr_s *descr;

#if 0
  /* Free the last received packet from the RX queue */

  nlldbg("rx abort\n");
  nlldbg("ESTAT:   %04x\n", enc_rdreg(priv, ENC_ESTAT));
  nlldbg("EIR:     %04x\n", enc_rdreg(priv, ENC_EIR));
  nlldbg("ERXTAIL: %04x\n", enc_rdreg(priv, ENC_ERXTAIL));
  nlldbg("ERXHAED: %04x\n", enc_rdreg(priv, ENC_ERXHEAD));

  descr = (struct enc_descr_s*)sq_peek(&priv->rxqueue);

  while (descr != NULL)
    {
      nlldbg("addr: %04x len: %d\n", descr->addr, descr->len);
      descr = (struct enc_descr_s*)sq_next(descr);
    }

  DEBUGASSERT(false);
#endif

  descr = (struct enc_descr_s*)sq_peek(&priv->rxqueue);

  if (descr != NULL)
    {
      enc_rxrmpkt(priv, descr);

      nlldbg("pending packet freed\n");
    }
  else
    {
      /* If no pending packet blocks the reception, reset all buffers */

      enc_resetbuffers(priv);
    }
}

/****************************************************************************
 * Function: enc_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters:
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
  net_lock_t lock;
  uint16_t eir;

  DEBUGASSERT(priv);

  /* Get exclusive access to both uIP and the SPI bus. */

  lock = net_lock();
  enc_lock(priv);

  /* A good practice is for the host controller to clear the Global Interrupt
   * Enable bit, INTIE (EIE<15>), immediately after an interrupt event. This
   * causes the interrupt pin to return to the non-asserted (high) state. Once
   * the interrupt has been serviced, the INTIE bit is set again to re-enable
   * interrupts. If a new interrupt occurs while servicing another, the act of
   * resetting the global enable bit will cause a new falling edge to occur on
   * the interrupt pin and ensure that the host does not miss any events
   */

  enc_bfc(priv, ENC_EIE, EIE_INTIE);

  /* Loop until all interrupts have been processed (EIR==0).  Note that
   * there is no infinite loop check... if there are always pending interrupts,
   * we are just broken.
   */

  while ((eir = enc_rdreg(priv, ENC_EIR) & EIR_ALLINTS) != 0)
    {
      /* Handle interrupts according to interrupt register register bit
       * settings.
       */

      nllvdbg("EIR: %04x\n", eir);

      if ((eir & EIR_DMAIF) != 0) /* DMA interrupt */
        {
          /* Not used by this driver. Just clear the interrupt request. */

          enc_bfc(priv, ENC_EIR, EIR_DMAIF);
        }

      /* LINKIF: The link change interrupt occurs when the PHY link status
       * changes. This flag is set by hardware when a link has either been
       * established or broken between the device and a remote Ethernet partner.
       * The current link status can be read from PHYLNK (ESTAT<8>). The
       * interrupt should be cleared by software once it has been serviced.
       *
       * To enable the link change interrupt, set LINKIE (EIE<11>).
       */

      if ((eir & EIR_LINKIF) != 0) /* PHY Link Status Change */
        {
          enc_linkstatus(priv);                /* Get current link status */
          enc_bfc(priv, ENC_EIR, EIR_LINKIF);  /* Clear the LINKIF interrupt */
        }

      /* The transmit complete interrupt occurs when the transmission of a
       * frame has ended (whether or not it was successful). This flag is set
       * when TXRTS (ECON1<1>) is cleared. The interrupt should be cleared by
       * software once it has been serviced.
       */

      if ((eir & EIR_TXIF) != 0) /* Transmit Done */
        {
          enc_txif(priv);
          enc_bfc(priv, ENC_EIR, EIR_TXIF);
        }

      /* The receive abort interrupt occurs when the reception of a frame has
       * been aborted. A frame being received is aborted when the Head Pointer
       * attempts to overrun the Tail Pointer, or when the packet counter has
       * reached FFh. In either case, the receive buffer is full and cannot fit
       * the incoming frame, so the packet has been dropped. This interrupt does
       * not occur when packets are dropped due to the receive filters rejecting
       * a packet. The interrupt should be cleared by software once it has been
       * serviced.
       *
       * To enable the receive abort interrupt, set RXABTIE (EIE<1>).
       * The corresponding interrupt flag is RXABTIF (EIR<1>).
       */

      if ((eir & EIR_RXABTIF) != 0) /* Receive Abort */
        {
#ifdef CONFIG_ENCX24J600_STATS
          priv->stats.rxerifs++;
#endif
          enc_rxabtif(priv);
          enc_bfc(priv, ENC_EIR, EIR_RXABTIF);  /* Clear the RXABTIF interrupt */
        }

      /* The received packet pending interrupt occurs when one or more frames
       * have been received and are ready for software processing. This flag is
       * set when the PKTCNT<7:0> (ESTAT<7:0>) bits are non-zero. This interrupt
       * flag is read-only and will automatically clear when the PKTCNT bits are
       * decremented to zero. For more details about receiving and processing
       * incoming frames, refer to Section 9.0 "Transmitting and Receiving
       * Packets".
       *
       * To enable the received packet pending interrupt, set PKTIE (EIE<6>).
       * The corresponding interrupt flag is PKTIF (EIR<6>).
       */

      if ((eir & EIR_PKTIF) != 0    /* RX Packet Pending */
          && (enc_rdreg(priv, ENC_ESTAT) & ESTAT_PKTCNT_MASK) != 0)
        {
          enc_pktif(priv);          /* Handle packet receipt */

          /* No clearing necessary, after PKTCNT == 0 the bit is automatically
           * cleared. This means we will loop until all packets are processed.
           */
        }

#ifdef CONFIG_ENCX24J600_STATS
      /* The transmit abort interrupt occurs when the transmission of a frame
       * has been aborted. An abort can occur for any of the following reasons:
       *
       * * Excessive collisions occurred as defined by the Retransmission
       *   Maximum, MAXRET<3:0> bits (MACLCON<3:0>), setting. If this occurs,
       *   the COLCNT bits (ETXSTAT<3:0>) will indicate the number of collisions
       *   that occurred.
       *
       * * A late collision occurred after 63 bytes were transmitted. If this
       *   occurs, LATECOL (ETXSTAT<10>) will be set.
       *
       * * The medium was busy and the packet was deferred. If this occurs,
       *   EXDEFER (ETXSTAT<8>) will be set.
       *
       * * The application aborted the transmission by clearing TXRTS
       *   (ECON1<1>).
       *
       * The interrupt should be cleared by software once it has  been serviced.
       * To enable the transmit abort interrupt, set TXABTIE (EIE<2>).
       */

      if ((eir & EIR_TXABTIF) != 0) /* Transmit Abort */
        {
          priv->stats.txerifs++;
          enc_bfc(priv, ENC_EIR, EIR_TXABTIF);  /* Clear the TXABTIF interrupt */
        }
#endif
    }

  /* Enable GPIO interrupts */

  priv->lower->enable(priv->lower);

  /* Enable Ethernet interrupts */

  enc_bfs(priv, ENC_EIE, EIE_INTIE);

  /* Release lock on the SPI bus and uIP */

  enc_unlock(priv);
  net_unlock(lock);
}

/****************************************************************************
 * Function: enc_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int enc_interrupt(int irq, FAR void *context)
{
  register FAR struct enc_driver_s *priv = &g_encx24j600[0];

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
  return work_queue(HPWORK, &priv->irqwork, enc_irqworker, (FAR void *)priv, 0);
}

/****************************************************************************
 * Function: enc_toworker
 *
 * Description:
 *   Our TX watchdog timed out.  This is the worker thread continuation of
 *   the watchdog timer interrupt.  Reset the hardware and start again.
 *
 * Parameters:
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
  net_lock_t lock;
  int ret;

  nlldbg("Tx timeout\n");
  DEBUGASSERT(priv);

  /* Get exclusive access to uIP. */

  lock = net_lock();

  /* Increment statistics and dump debug info */

#ifdef CONFIG_ENCX24J600_STATS
  priv->stats.txtimeouts++;
#endif

  /* Then reset the hardware: Take the interface down, then bring it
   * back up
   */

  ret = enc_ifdown(&priv->dev);
  DEBUGASSERT(ret == OK);
  ret = enc_ifup(&priv->dev);
  DEBUGASSERT(ret == OK);
  (void)ret;

  /* Then poll uIP for new XMIT data */

  (void)devif_poll(&priv->dev, enc_txpoll);

  /* Release uIP */

  net_unlock(lock);
}

/****************************************************************************
 * Function: enc_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Perform work on the worker thread.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv && work_available(&priv->towork));

  /* Notice that Tx timeout watchdog is not active so further Tx timeouts
   * can occur until we restart the Tx timeout watchdog.
   */

  ret = work_queue(HPWORK, &priv->towork, enc_toworker, (FAR void *)priv, 0);
  (void)ret;
  DEBUGASSERT(ret == OK);
}

/****************************************************************************
 * Function: enc_pollworker
 *
 * Description:
 *   Periodic timer handler continuation.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
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
  net_lock_t lock;

  DEBUGASSERT(priv);

  /* Get exclusive access to both uIP and the SPI bus. */

  lock = net_lock();
  enc_lock(priv);

  /* Verify that the hardware is ready to send another packet.  The driver
   * start a transmission process by setting ECON1.TXRTS. When the packet is
   * finished transmitting or is aborted due to an error/cancellation, the
   * ECON1.TXRTS bit will be cleared.
   */

  if ((enc_rdreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0)
    {
      /* Yes.. update TCP timing states and poll uIP for new XMIT data. Hmmm..
       * looks like a bug here to me.  Does this mean if there is a transmit
       * in progress, we will missing TCP time state updates?
       */

      (void)devif_timer(&priv->dev, enc_txpoll, ENC_POLLHSEC);
    }

  /* Release lock on the SPI bus and uIP */

  enc_unlock(priv);
  net_unlock(lock);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, ENC_WDDELAY, enc_polltimer, 1, arg);
}

/****************************************************************************
 * Function: enc_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  int ret;

  /* In complex environments, we cannot do SPI transfers from the timout
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(priv && work_available(&priv->pollwork));

  /* Notice that poll watchdog is not active so further poll timeouts can
   * occur until we restart the poll timeout watchdog.
   */

  ret = work_queue(HPWORK, &priv->pollwork, enc_pollworker, (FAR void *)priv, 0);
  (void)ret;
  DEBUGASSERT(ret == OK);
}

/****************************************************************************
 * Function: enc_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
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

  nlldbg("Bringing up: %d.%d.%d.%d\n",
         dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Initialize Ethernet interface, set the MAC address, and make sure that
   * the ENC28J80 is not in power save mode.
   */

  ret = enc_reset(priv);
  if (ret == OK)
    {
      enc_setmacaddr(priv);

      /* Enable interrupts at the ENCX24J600.  Interrupts are still disabled
       * at the interrupt controller.
       */

      enc_bfc(priv, ENC_EIR, EIR_ALLINTS);
      enc_bfs(priv, ENC_EIE, EIE_INTIE  | EIE_LINKIE  |
                             EIE_PKTIE  | EIE_RXABTIE |
                             EIE_TXIE   );

#ifdef CONFIG_ENCX24J600_STATS
      enc_bfs(priv, ENC_EIE, EIE_TXABTIE);
#endif

      /* Enable the receiver */

      enc_bfs(priv, ENC_ECON1, ECON1_RXEN);

      /* Set and activate a timer process */

      (void)wd_start(priv->txpoll, ENC_WDDELAY, enc_polltimer, 1, (uint32_t)priv);

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
 * Function: enc_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
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

  nlldbg("Taking down: %d.%d.%d.%d\n",
         dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
         (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  priv->lower->disable(priv->lower);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Reset the device and leave in the power save state */

  ret = enc_reset(priv);
  enc_pwrsave(priv);

  priv->ifstate = ENCSTATE_DOWN;
  irqrestore(flags);

  /* Un-lock the SPI bus */

  enc_unlock(priv);

  return ret;
}

/****************************************************************************
 * Function: enc_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
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

  flags = irqsave();
  if (priv->ifstate == ENCSTATE_RUNNING)
    {
      /* Check if the hardware is ready to send another packet.  The driver
       * starts a transmission process by setting ECON1.TXRTS. When the packet is
       * finished transmitting or is aborted due to an error/cancellation, the
       * ECON1.TXRTS bit will be cleared.
       */

      if ((enc_rdreg(priv, ENC_ECON1) & ECON1_TXRTS) == 0)
        {
          /* The interface is up and TX is idle; poll uIP for new XMIT data */

          (void)devif_poll(&priv->dev, enc_txpoll);
        }
    }

  /* Un-lock the SPI bus */

  irqrestore(flags);
  enc_unlock(priv);

  return OK;
}

/****************************************************************************
 * Function: enc_rxavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int enc_rxavail(struct net_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;

  if (!sq_empty(&priv->rxqueue))
    {
      nllvdbg("RX queue not empty, trying to dispatch\n");
      enc_rxdispatch(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: enc_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
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
 * Function: enc_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
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
 * Function: enc_pwrsave
 *
 * Description:
 *   The ENCX24J600 may be placed in Power-Down mode through the command
 *   interface. In this mode, the device will no longer be able to transmit or
 *   receive any packets or perform DMA operations. However, most registers, and
 *   all buffer memories, retain their states and remain accessible by the host
 *   controller. The clock driver also remains operational, leaving the CLKOUT
 *   function unaffected. However, the MAC/MII and PHY registers all become
 *   inaccessible, and the PHY registers lose their current states.
 *
 *   1. Turn off the Modular Exponentiation and AES engines by clearing
 *      CRYPTEN (EIR<15>).
 *   2. Turn off packet reception by clearing RXEN (ECON1<0>).
 *   3. Wait for any in-progress receptions to complete by polling
 *      RXBUSY (ESTAT<13>) until it is clear.
 *   4. Wait for any current transmission operation to complete by verifying
 *      that TXRTS (ECON1<1>) is clear.
 *   5. Power-down the PHY by setting the PSLEEP bit (PHCON1<11>).
 *   6. Power-down the Ethernet interface by clearing
 *      ETHEN and STRCH (ECON2<15,14>). Disabling the LED stretching behavior is
 *      necessary to ensure no LEDs get trapped in a perpetually illuminated
 *      state in the event they are being stretched on when ETHEN is cleared.
 *
 * Note:
 *   Instead of providing a powerup function, the job is done by enc_reset.
 *   enc_ifup calls it anyway.
 *
 * Parameters:
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
  uint16_t regval;

  nllvdbg("Set PWRSV\n");

  /* 1. Turn off AES */

  enc_bfc(priv, ENC_EIR, EIR_CRYPTEN);

  /* 2. Turn off packet reception */

  enc_bfc(priv, ENC_ECON1, ECON1_RXEN);

  /* 3. Wait for pending reception to complete */

  enc_waitreg(priv, ENC_ESTAT, ESTAT_RXBUSY, 0);

  /* 4. Wait for any current transmissions to complete */

  enc_waitreg(priv, ENC_ECON1, ECON1_TXRTS, 0);

  /* 5. Power down the PHY */

  regval = enc_rdphy(priv, ENC_PHCON1);
  regval |= PHCON1_PSLEEP;
  enc_wrphy(priv, ENC_PHCON1, regval);

  /* 6. Power down the Ethernet interface */

  enc_bfc(priv, ENC_ECON2, ECON2_ETHEN | ECON2_STRCH);
}

/****************************************************************************
 * Function: enc_ldmacaddr
 *
 * Description:
 *   Load the MAC address from the ENCX24j600 and write it to the device
 *   structure.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_ldmacaddr(FAR struct enc_driver_s *priv)
{
  uint16_t regval;
  uint8_t *mac = priv->dev.d_mac.ether_addr_octet;

  nvdbg("Using ENCX24J600's built in MAC address\n");

  regval = enc_rdreg(priv, ENC_MAADR1);
  mac[0] = regval & 0xff;
  mac[1] = regval >> 8;

  regval = enc_rdreg(priv, ENC_MAADR2);
  mac[2] = regval & 0xff;
  mac[3] = regval >> 8;

  regval = enc_rdreg(priv, ENC_MAADR3);
  mac[4] = regval & 0xff;
  mac[5] = regval >> 8;
}

/****************************************************************************
 * Function: enc_setmacaddr
 *
 * Description:
 *   Set the MAC address to the configured value.  This is done after ifup
 *   or after a TX timeout.  Note that this means that the interface must
 *   be down before configuring the MAC addr.
 *   If the MAC address is 0 in all digits, the ENCX24J600's MAC is read out.
 *
 * Parameters:
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
  uint8_t *mac = priv->dev.d_mac.ether_addr_octet;
  struct ether_addr zmac;

  memset(&zmac, 0, sizeof(zmac));

  if (memcmp(&priv->dev.d_mac, &zmac, sizeof(zmac)) == 0)
    {
      /* No user defined MAC address. Read it from the device. */

      enc_ldmacaddr(priv);
    }
  else
    {
      /* There is a user defined mac address. Write it to the ENCXJ600 */

      nvdbg("Using an user defined MAC address\n");

      enc_wrreg(priv, ENC_MAADR1, (uint16_t)mac[1] << 8 | (uint16_t)mac[0]);
      enc_wrreg(priv, ENC_MAADR2, (uint16_t)mac[3] << 8 | (uint16_t)mac[2]);
      enc_wrreg(priv, ENC_MAADR3, (uint16_t)mac[5] << 8 | (uint16_t)mac[4]);
    }
}

/****************************************************************************
 * Function: enc_resetbuffers
 *
 * Description:
 *   Initializes the RX/TX queues and configures the enc's RX/TX buffers.
 *   Called on general reset and on rxabt interrupt.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void enc_resetbuffers(FAR struct enc_driver_s *priv)
{
  int i;
  /* Initialize receive and transmit buffers  */

  priv->nextpkt = PKTMEM_RX_START;
  enc_wrreg(priv, ENC_ERXST, PKTMEM_RX_START);

  /* Program the Tail Pointer, ERXTAIL, to the last even address of the buffer */

  enc_wrreg(priv, ENC_ERXTAIL, PKTMEM_RX_END - 2);

  sq_init(&priv->txfreedescr);
  sq_init(&priv->rxfreedescr);
  sq_init(&priv->txqueue);
  sq_init(&priv->rxqueue);

  /* For transmition we preinitialize the descriptors to aligned NET_BUFFSIZE */

  for (i = 0; i < ENC_NTXDESCR; i++)
    {
      priv->txdescralloc[i].addr = PKTMEM_START + PKTMEM_ALIGNED_BUFSIZE * i;
      sq_addlast((sq_entry_t*)&priv->txdescralloc[i], &priv->txfreedescr);
    }

  /* Receive descriptors addresses are set on reception */

  for (i = 0; i < CONFIG_ENCX24J600_NRXDESCR; i++)
    {
      sq_addlast((sq_entry_t*)&priv->rxdescralloc[i], &priv->rxfreedescr);
    }
}

/****************************************************************************
 * Function: enc_reset
 *
 * Description:
 *   Stop, reset, re-initialize, and restart the ENCX24J600.  This is done
 *   initially, on ifup, and after a TX timeout.
 *
 * Parameters:
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
  int ret;
  uint16_t regval;

  nllvdbg("Reset\n");

  do
    {
      enc_wrreg(priv, ENC_EUDAST, 0x1234);
    }
  while (enc_rdreg(priv, ENC_EUDAST) != 0x1234);

  /* Wait for clock to become ready */

  ret = enc_waitreg(priv, ENC_ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY);

  if (ret != OK)
    {
      nlldbg("ERROR: encx24j600 clock failed to become ready\n");
      return -ENODEV;
    }

  /* Reset the ENCX24J600 */

  enc_setethrst(priv);

  /* Check if EUDAST has been reset to 0 */

  regval = enc_rdreg(priv, ENC_EUDAST);

  if (regval != 0x0000)
    {
      nlldbg("ERROR: encx24j600 seems not to be reset properly\n");
      return -ENODEV;
    }

  /**
   * Wait at least 256 μs for the PHY registers and PHY status bits to become
   * available.
   */
  up_udelay(256);

  /* Initialize RX/TX buffers */

  enc_resetbuffers(priv);

#if 0
  /* When restarting auto-negotiation, the ESTAT_PHYLINK gets set but the link
   * seems not to be ready. Because auto-negotiation is enabled by default
   * (but with different PHANA_* settings) I did not investigate that further.
   */

  /* "Typically, when using auto-negotiation, users should write 0x05E1 to PHANA
   * to advertise flow control capability."
   */

  enc_wrphy(priv, ENC_PHANA, PHANA_ADPAUS0 | PHANA_AD10FD | PHANA_AD10 |
            PHANA_AD100FD | PHANA_AD100 | PHANA_ADIEEE0);

  /* Restart auto-negotiation */

  enc_wrphy(priv, ENC_PHCON1, PHCON1_RENEG);

  do
    {
      regval = enc_rdphy(priv, ENC_PHSTAT1);
    }
  while ((regval & PHSTAT1_ANDONE) != 0);

  nllvdbg("Auto-negotation completed\n");

#endif

  enc_linkstatus(priv);

  /* Set the maximum packet size which the controller will accept */

  enc_wrreg(priv, ENC_MAMXFL, CONFIG_NET_BUFSIZE + 4);

  ret = enc_waitreg(priv, ENC_ESTAT, ESTAT_PHYLNK, ESTAT_PHYLNK);

  if (ret == OK)
    {
      enc_linkstatus(priv);
    }

#if 0
  if (ret != OK)
    {
      nlldbg("ERROR: encx24j600 failed to establish link\n");
      return -ENODEV;
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: enc_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.  The ENCX24J600 device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the ENCX24J600
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., ENCX24J600 GPIO interrupts).
 *   devno - If more than one ENCX24J600 is supported, then this is the
 *           zero based number that identifies the ENCX24J600;
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int enc_initialize(FAR struct spi_dev_s *spi,
                          FAR const struct enc_lower_s *lower,
                          unsigned int devno)
{
  FAR struct enc_driver_s *priv;

  DEBUGASSERT(devno < CONFIG_ENCX24J600_NINTERFACES);
  priv = &g_encx24j600[devno];

  /* Initialize the driver structure */

  memset(g_encx24j600, 0, CONFIG_ENCX24J600_NINTERFACES*sizeof(struct enc_driver_s));
  priv->dev.d_ifup    = enc_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = enc_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = enc_txavail;  /* New TX data callback */
  priv->dev.d_rxavail = enc_rxavail;  /* RX wating callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = enc_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = enc_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = priv;         /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll       = wd_create();   /* Create periodic poll timer */
  priv->txtimeout    = wd_create();   /* Create TX timeout timer */
  priv->spi          = spi;           /* Save the SPI instance */
  priv->lower        = lower;         /* Save the low-level MCU interface */

  /* The interface should be in the down state.  However, this function is called
   * too early in initalization to perform the ENCX24J600 reset in enc_ifdown.  We
   * are depending upon the fact that the application level logic will call enc_ifdown
   * later to reset the ENCX24J600.
   */

  priv->ifstate = ENCSTATE_UNINIT;

  /* Attach the interrupt to the driver (but don't enable it yet) */

  if (lower->attach(lower, enc_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Configure SPI for the ENCX24J600 */

  enc_configspi(priv->spi);

  /* Lock the SPI bus so that we have exclusive access */

  enc_lock(priv);

  /* Load the MAC address */

  enc_ldmacaddr(priv);

  /* Power down the device */

  enc_pwrsave(priv);

  /* Unlock the SPI bus */

  enc_unlock(priv);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  return netdev_register(&priv->dev);
}

/****************************************************************************
 * Function: enc_stats
 *
 * Description:
 *   Return accumulated ENCX24J600 statistics.  Statistics are cleared after
 *   being returned.
 *
 * Parameters:
 *   devno - If more than one ENCX24J600 is supported, then this is the
 *           zero based number that identifies the ENCX24J600;
 *   stats - The user-provided location to return the statistics.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_ENCX24J600_STATS
int enc_stats(unsigned int devno, struct enc_stats_s *stats)
{
  FAR struct enc_driver_s *priv ;
  irqstate_t flags;

  DEBUGASSERT(devno < CONFIG_ENCX24J600_NINTERFACES);
  priv = &g_encx24j600[devno];

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  memcpy(stats, &priv->stats, sizeof(struct enc_stats_s));
  memset(&priv->stats, 0, sizeof(struct enc_stats_s));
  irqrestore(flags);
  return OK;
}
#endif
#endif /* CONFIG_NET && CONFIG_ENCX24J600_NET */
