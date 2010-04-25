/****************************************************************************
 * drivers/net/enc28j60.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_ENC28J60)

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#include "enc28j60.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* All SPI settings can be specifed in the configuration.  If not, some
 * defaults will be provided.
 *
 * CONFIG_ENC28J60_OWNBUS - Set if the ENC28J60 is the only active device on
 *   the SPI bus.
 * CONFIG_ENC28J60_SPIMODE - Controls the SPI mode
 * CONFIG_ENC28J60_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ENC28J60_NINTERFACES - Specifies the number of physical ENC28J60
 *   devices that will be supported.
 */

#ifndef CONFIG_ENC28J60_SPIMODE
#  define CONFIG_ENC28J60_SPIMODE SPIDEV_MODE2
#endif

/* CONFIG_ENC28J60_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_ENC28J60_NINTERFACES
# define CONFIG_ENC28J60_NINTERFACES 1
#endif

/* Timing *******************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define ENC28J60_WDDELAY   (1*CLK_TCK)
#define ENC28J60_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define ENC28J60_TXTIMEOUT (60*CLK_TCK)

/* Misc. Helper Macros ******************************************************/

#define enc28j60_rdglobal(priv,ctrlref) \
  enc28j60_rdglobal2(priv, ENC28J60_RCR | GETADDR(ctrlreg))
#define enc28j60_wrglobal(priv,ctrlreg,wrdata) \
  enc28j60_wrglobal2(priv, ENC28J60_WCR | GETADDR(ctrlreg), wrdata)
#define enc28j60_clrglobal(priv,ctrlreg,clrbits) \
  enc28j60_wrglobal2(priv, ENC28J60_BFC | GETADDR(ctrlreg), clrbits)
#define enc28j60_setglobal(priv,ctrlreg,setbits) \
  enc28j60_wrglobal2(priv, ENC28J60_BFS | GETADDR(ctrlreg), setbits)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The enc28j60_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct enc28j60_driver_s
{
  bool     bifup;            /* true:ifup false:ifdown */
  uint8_t  bank;             /* Currently selected bank */
  uint16_t nextpkt;          /* Next packet address */
  WDOG_ID  txpoll;           /* TX poll timer */
  WDOG_ID  txtimeout;        /* TX timeout timer */

  /* This is the contained SPI driver intstance */

  FAR struct spi_dev_s *spi;

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s dev;   /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct enc28j60_driver_s g_enc28j60[CONFIG_ENC28J60_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

static inline void enc28j60_configspi(FAR struct spi_dev_s *spi);
#ifdef CONFIG_ENC28J60_OWNBUS
static inline uint8_t enc28j60_select(FAR struct spi_dev_s *spi);
static inline uint8_t enc28j60_deselect(FAR struct spi_dev_s *spi);
#else
static uint8_t enc28j60_select(FAR struct spi_dev_s *spi);
static uint8_t enc28j60_deselect(FAR struct spi_dev_s *spi);
#endif
static uint8_t enc28j60_rdglobal2(FAR struct enc28j60_driver_s *priv,
         uint8_t cmd);
static void enc28j60_wrglobal2(FAR struct enc28j60_driver_s *priv,
         uint8_t cmd, uint8_t wrdata);
static void enc28j60_setbank(FAR struct enc28j60_driver_s *priv, uint8_t bank);
static uint8_t enc28j60_rdbank(FAR struct enc28j60_driver_s *priv,
         uint8_t ctrlreg);
static uint8_t enc28j60_rdphymac(FAR struct enc28j60_driver_s *priv,
         uint8_t ctrlreg);

/* Common TX logic */

static int  enc28j60_transmit(FAR struct enc28j60_driver_s *priv);
static int  enc28j60_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void enc28j60_receive(FAR struct enc28j60_driver_s *priv);
static void enc28j60_txdone(FAR struct enc28j60_driver_s *priv);
static int  enc28j60_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void enc28j60_polltimer(int argc, uint32_t arg, ...);
static void enc28j60_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int enc28j60_ifup(struct uip_driver_s *dev);
static int enc28j60_ifdown(struct uip_driver_s *dev);
static int enc28j60_txavail(struct uip_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: enc28j60_configspi
 *
 * Description:
 *   Configure the SPI for use with the ENC28J60
 *
 ****************************************************************************/
 
static inline void enc28j60_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ENC28J60.  But only if we own the SPI bus.
   * Otherwise, don't bother because it might change.
   */

#ifdef CONFIG_ENC28J60_OWNBUS
  SPI_SETMODE(spi, CONFIG_ENC28J60_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_ENC28J60_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_ENC28J60_FREQUENCY)
#endif
#endif
}

/****************************************************************************
 * Function: enc28j60_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 ****************************************************************************/

#ifdef CONFIG_ENC28J60_OWNBUS
static inline uint8_t enc28j60_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_ETHERNET, true);
}
#else
static uint8_t enc28j60_select(FAR struct spi_dev_s *spi)
{
  /* Select ENC2J60 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_ETHERNET, true);

  /* Now make sure that the SPI bus is configured for the ENC28J60 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_ENC28J60_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_ENC28J60_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_ENC28J60_FREQUENCY)
#endif
}
#endif

/****************************************************************************
 * Function: enc28j60_deselect
 *
 * Description:
 *   De-select the SPI
 *
 ****************************************************************************/

#ifdef CONFIG_ENC28J60_OWNBUS
static inline uint8_t enc28j60_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_ETHERNET, false);
}
#else
static uint8_t enc28j60_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ENC28J60 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_ETHERNET, false);
  SPI_LOCK(spi, false);
}
#endif

/****************************************************************************
 * Function: enc28j60_rdglobal2
 *
 * Description:
 *   Read a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the the global address register.
 *
 ****************************************************************************/

static uint8_t enc28j60_rdglobal2(FAR struct enc28j60_driver_s *priv,
                                 uint8_t cmd)
{
  FAR struct spi_dev_s *spi;
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC2J60 chip */

  enc28j60_select(spi);

  /* Send the read command and (maybe collect the return data) */

  rddata = SPI_SEND(spi, cmd);

  /* De-select ENC28J60 chip */

  enc28j60_deselect(spi);
  return rddata;
}

/****************************************************************************
 * Function: enc28j60_wrglobal2
 *
 * Description:
 *   Write to a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the the global address register.
 *
 ****************************************************************************/

static void enc28j60_wrglobal2(FAR struct enc28j60_driver_s *priv,
                               uint8_t cmd, uint8_t wrdata)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC2J60 chip */

  enc28j60_select(spi);

  /* Send the write command */

  (void)SPI_SEND(spi, cmd);

  /* Send the data byte */

  (void)SPI_SEND(spi, wrdata);

  /* De-select ENC28J60 chip. */

  enc28j60_deselect(spi);
}

/****************************************************************************
 * Function: enc28j60_setbank
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 * Assumption:
 *   The caller has exclusive access to the SPI bus
 *
 ****************************************************************************/

static void enc28j60_setbank(FAR struct enc28j60_driver_s *priv, uint8_t bank)
{
  /* Check if the bank setting has changed*/

  if (bank != priv->bank)
    {
      /* Select bank 0 (just so that all of the bits are cleared) */

      enc28j60_clrglobal(priv, ECON1, ECON1_BSEL_MASK);

      /* Then OR in bits to get the correct bank */

      if (bank != 0)
        {
          enc28j60_setglobal(priv, ECON1, (bank << ECON1_BSEL_SHIFT));
        }

      /* Then remember the bank setting */

      priv->bank = bank;
    }
}

/****************************************************************************
 * Function: enc28j60_rdbank
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 ****************************************************************************/
 
static uint8_t enc28j60_rdbank(FAR struct enc28j60_driver_s *priv,
                               uint8_t ctrlreg)
{
  FAR struct spi_dev_s *spi;
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC2J60 chip */

  enc28j60_select(spi);

  /* set the bank */

  enc28j60_setbank(priv, GETBANK(ctrlreg));

  /* Send the read command and collect the return data. */

  rddata = SPI_SEND(spi, ENC28J60_RCR | GETADDR(ctrlreg));

  /* De-select ENC28J60 chip */

  enc28j60_deselect(spi);
  return rddata;
}

/****************************************************************************
 * Function: enc28j60_rdphymac
 *
 * Description:
 *   Somewhat different timing is required to read from any PHY or MAC
 *   registers.  The PHY/MAC data is returned on the second byte after the
 *   command.
 *
 ****************************************************************************/

static uint8_t enc28j60_rdphymac(FAR struct enc28j60_driver_s *priv,
                                 uint8_t ctrlreg)
{
  FAR struct spi_dev_s *spi;
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC2J60 chip */

  enc28j60_select(spi);

  /* Set the bank */

  enc28j60_setbank(priv, GETBANK(ctrlreg));

  /* Send the read command (discarding the return data) */

  (void)SPI_SEND(spi, ENC28J60_RCR | GETADDR(ctrlreg));

  /* Do an extra transfer to get the data from the MAC or PHY */

  rddata = SPI_SEND(spi, 0);

  /* De-select ENC28J60 chip */

  enc28j60_deselect(spi);
  return rddata;
}

/****************************************************************************
 * Function: enc28j60_wrbank
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 ****************************************************************************/
 
static void enc28j60_wrbank(FAR struct enc28j60_driver_s *priv,
                            uint8_t ctrlreg, uint8_t wrdata)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC2J60 chip */

  enc28j60_select(spi);

  /* Set the bank */

  enc28j60_setbank(priv, GETBANK(ctrlreg));

  /* Send the write command */

  (void)SPI_SEND(spi, ENC28J60_WCR | GETADDR(ctrlreg));

  /* Send the data byte */

  (void)SPI_SEND(spi, wrdata);

  /* De-select ENC28J60 chip. */

  enc28j60_deselect(spi);
}

/****************************************************************************
 * Function: enc28j60_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
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

static int enc28j60_transmit(FAR struct enc28j60_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet */

  /* Increment statistics */

  /* Disable Ethernet interrupts */

  /* Send the packet: address=priv->dev.d_buf, length=priv->dev.d_len */

  /* Restore Ethernet interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, ENC28J60_TXTIMEOUT, enc28j60_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: enc28j60_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int enc28j60_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct enc28j60_driver_s *priv = (FAR struct enc28j60_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      uip_arp_out(&priv->dev);
      enc28j60_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: enc28j60_receive
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
 *
 ****************************************************************************/

static void enc28j60_receive(FAR struct enc28j60_driver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to priv->dev.d_buf.  Set
       * amount of data in priv->dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin();
          uip_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              uip_arp_out(&priv->dev);
              enc28j60_transmit(priv);
            }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

           if (priv->dev.d_len > 0)
             {
               enc28j60_transmit(priv);
             }
         }
     }
  while (false); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: enc28j60_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
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

static void enc28j60_txdone(FAR struct enc28j60_driver_s *priv)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(priv->txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->dev, enc28j60_uiptxpoll);
}

/****************************************************************************
 * Function: enc28j60_interrupt
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

static int enc28j60_interrupt(int irq, FAR void *context)
{
  register FAR struct enc28j60_driver_s *priv = &g_enc28j60[0];

  /* Disable Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call enc28j60_receive() */

  enc28j60_receive(priv);

  /* Check is a packet transmission just completed.  If so, call enc28j60_txdone */

  enc28j60_txdone(priv);

  /* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if 
   * there are no pending transmissions.
   */

  return OK;
}

/****************************************************************************
 * Function: enc28j60_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
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

static void enc28j60_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct enc28j60_driver_s *priv = (FAR struct enc28j60_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->dev, enc28j60_uiptxpoll);
}

/****************************************************************************
 * Function: enc28j60_polltimer
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

static void enc28j60_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct enc28j60_driver_s *priv = (FAR struct enc28j60_driver_s *)arg;

  /* Check if there is room in the send another TXr packet.  */

  /* If so, update TCP timing states and poll uIP for new XMIT data */

  (void)uip_timer(&priv->dev, enc28j60_uiptxpoll, ENC28J60_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, ENC28J60_WDDELAY, enc28j60_polltimer, 1, arg);
}

/****************************************************************************
 * Function: enc28j60_ifup
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

static int enc28j60_ifup(struct uip_driver_s *dev)
{
  FAR struct enc28j60_driver_s *priv = (FAR struct enc28j60_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initilize Ethernet interface */

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, ENC28J60_WDDELAY, enc28j60_polltimer, 1, (uint32_t)priv);

  /* Enable the Ethernet interrupt */

  priv->bifup = true;
  up_enable_irq(CONFIG_ENC28J60_IRQ);
  return OK;
}

/****************************************************************************
 * Function: enc28j60_ifdown
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

static int enc28j60_ifdown(struct uip_driver_s *dev)
{
  FAR struct enc28j60_driver_s *priv = (FAR struct enc28j60_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_ENC28J60_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Reset the device */

  priv->bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: enc28j60_txavail
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

static int enc28j60_txavail(struct uip_driver_s *dev)
{
  FAR struct enc28j60_driver_s *priv = (FAR struct enc28j60_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {

      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&priv->dev, enc28j60_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: enc28j60_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the Ethernet controller and driver */

int enc28j60_initialize(FAR struct spi_dev_s *spi)
{
  /* Initialize and configure the ENC28J60 */

  /* Initialize the driver structure */

  memset(g_enc28j60, 0, CONFIG_ENC28J60_NINTERFACES*sizeof(struct enc28j60_driver_s));
  g_enc28j60[0].dev.d_ifup    = enc28j60_ifup;     /* I/F down callback */
  g_enc28j60[0].dev.d_ifdown  = enc28j60_ifdown;   /* I/F up (new IP address) callback */
  g_enc28j60[0].dev.d_txavail = enc28j60_txavail;  /* New TX data callback */
  g_enc28j60[0].dev.d_private = (void*)g_enc28j60; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  g_enc28j60[0].txpoll       = wd_create();   /* Create periodic poll timer */
  g_enc28j60[0].txtimeout    = wd_create();   /* Create TX timeout timer */
  g_enc28j60[0].spi          = spi;           /* Save the SPI instance */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_ENC28J60_IRQ, enc28j60_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Read the MAC address from the hardware into g_enc28j60[0].dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_enc28j60[0].dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_ENC28J60_NET */

