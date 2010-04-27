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
#include <nuttx/wqueue.h>

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
 *   the SPI bus.  No locking or SPI configuration will be performed. All
 *   transfers will be performed from the ENC2J60 interrupt handler.
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

/* We need to have the work queue to handle SPI interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE) && !defined(CONFIG_ENC28J60_OWNBUS)
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Timing *******************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define ENC28J60_WDDELAY   (1*CLK_TCK)
#define ENC28J60_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define ENC28J60_TXTIMEOUT (60*CLK_TCK)

/* Misc. Helper Macros ******************************************************/

#define enc_rdgreg(priv,ctrlreg) \
  enc_rdgreg2(priv, ENC28J60_RCR | GETADDR(ctrlreg))
#define enc_wdgreg(priv,ctrlreg,wrdata) \
  enc_wdgreg2(priv, ENC28J60_WCR | GETADDR(ctrlreg), wrdata)
#define enc_clrglobal(priv,ctrlreg,clrbits) \
  enc_wdgreg2(priv, ENC28J60_BFC | GETADDR(ctrlreg), clrbits)
#define enc_setglobal(priv,ctrlreg,setbits) \
  enc_wdgreg2(priv, ENC28J60_BFS | GETADDR(ctrlreg), setbits)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The enc_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct enc_driver_s
{
  /* Device control */

  bool          bifup;        /* true:ifup false:ifdown */
  uint8_t       bank;         /* Currently selected bank */
  uint16_t      nextpkt;      /* Next packet address */
  int           irq;          /* GPIO IRQ configured for the ENC28J60 */

  /* Timing */

  WDOG_ID       txpoll;       /* TX poll timer */
  WDOG_ID       txtimeout;    /* TX timeout timer */

  /* We we don't own the SPI bus, then we cannot do SPI accesses from the
   * interrupt handler.
   */
 
#ifndef CONFIG_ENC28J60_OWNBUS
  struct work_s work;         /* Work queue support */
#endif

  /* This is the contained SPI driver intstance */

  FAR struct spi_dev_s *spi;

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s dev;    /* Interface understood by uIP */

#ifdef CONFIG_ENC28J60_STATS
  uint8_t  maxpktcnt;         /* Max. number of buffered RX packets */
  uint32_t txifs;             /* TXIF completion events */
  uint32_t txabrts;           /* TXIF completions with ESTAT.TXABRT */
  uint32_t txerifs;           /* TXERIF error events */
  uint32_t rxerifs;           /* RXERIF error evernts */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct enc_driver_s g_enc28j60[CONFIG_ENC28J60_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

static inline void enc_configspi(FAR struct spi_dev_s *spi);
#ifdef CONFIG_ENC28J60_OWNBUS
static inline void enc_select(FAR struct spi_dev_s *spi);
static inline void enc_deselect(FAR struct spi_dev_s *spi);
#else
static void enc_select(FAR struct spi_dev_s *spi);
static void enc_deselect(FAR struct spi_dev_s *spi);
#endif

/* SPI control register access */

static uint8_t enc_rdgreg2(FAR struct enc_driver_s *priv, uint8_t cmd);
static void enc_wdgreg2(FAR struct enc_driver_s *priv, uint8_t cmd,
         uint8_t wrdata);
static void enc_setbank(FAR struct enc_driver_s *priv, uint8_t bank);
static uint8_t enc_rdbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg);
static uint8_t enc_wrbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg,
         uint8_t wrdata);
static uint8_t enc_rdphymac(FAR struct enc_driver_s *priv, uint8_t ctrlreg);

/* SPI buffer transfers */

static void enc_rdbuffer(FAR struct enc_driver_s *priv, FAR uint8_t *buffer,
         size_t buflen);
static void enc_wrbuffer(FAR struct enc_driver_s *priv,
         FAR const uint8_t *buffer, size_t buflen);

/* PHY register access */

static uint16_t enc_rdphy(FAR struct enc_driver_s *priv, uint8_t phyaddr);
static void enc_wrphy(FAR struct enc_driver_s *priv, uint8_t phyaddr,
         uint16_t phydata);

/* Common TX logic */

static int  enc_transmit(FAR struct enc_driver_s *priv);
static int  enc_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void enc_linkstatus(FAR struct enc_driver_s *priv);
static void enc_txif(FAR struct enc_driver_s *priv);
static void enc_txerif(FAR struct enc_driver_s *priv);
static void enc_txerif(FAR struct enc_driver_s *priv);
static void enc_rxerif(FAR struct enc_driver_s *priv);
static void enc_pktif(FAR struct enc_driver_s *priv);
static void enc_worker(FAR void *arg);
static int  enc_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void enc_polltimer(int argc, uint32_t arg, ...);
static void enc_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int enc_ifup(struct uip_driver_s *dev);
static int enc_ifdown(struct uip_driver_s *dev);
static int enc_txavail(struct uip_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: enc_configspi
 *
 * Description:
 *   Configure the SPI for use with the ENC28J60
 *
 ****************************************************************************/
 
static inline void enc_configspi(FAR struct spi_dev_s *spi)
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
 * Function: enc_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 ****************************************************************************/

#ifdef CONFIG_ENC28J60_OWNBUS
static inline void enc_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_ETHERNET, true);
}
#else
static void enc_select(FAR struct spi_dev_s *spi)
{
  /* Select ENC28J60 chip (locking the SPI bus in case there are multiple
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
 * Function: enc_deselect
 *
 * Description:
 *   De-select the SPI
 *
 ****************************************************************************/

#ifdef CONFIG_ENC28J60_OWNBUS
static inline void enc_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_ETHERNET, false);
}
#else
static void enc_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ENC28J60 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_ETHERNET, false);
  SPI_LOCK(spi, false);
}
#endif

/****************************************************************************
 * Function: enc_rdgreg2
 *
 * Description:
 *   Read a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the the global address register.
 *
 ****************************************************************************/

static uint8_t enc_rdgreg2(FAR struct enc_driver_s *priv, uint8_t cmd)
{
  FAR struct spi_dev_s *spi;
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* Send the read command and (maybe collect the return data) */

  rddata = SPI_SEND(spi, cmd);

  /* De-select ENC28J60 chip */

  enc_deselect(spi);
  return rddata;
}

/****************************************************************************
 * Function: enc_wdgreg2
 *
 * Description:
 *   Write to a global register (EIE, EIR, ESTAT, ECON2, or ECON1).  The cmd
 *   include the CMD 'OR'd with the the global address register.
 *
 ****************************************************************************/

static void enc_wdgreg2(FAR struct enc_driver_s *priv, uint8_t cmd,
                        uint8_t wrdata)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* Send the write command */

  (void)SPI_SEND(spi, cmd);

  /* Send the data byte */

  (void)SPI_SEND(spi, wrdata);

  /* De-select ENC28J60 chip. */

  enc_deselect(spi);
}

/****************************************************************************
 * Function: enc_setbank
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 * Assumption:
 *   The caller has exclusive access to the SPI bus
 *
 ****************************************************************************/

static void enc_setbank(FAR struct enc_driver_s *priv, uint8_t bank)
{
  /* Check if the bank setting has changed*/

  if (bank != priv->bank)
    {
      /* Select bank 0 (just so that all of the bits are cleared) */

      enc_clrglobal(priv, ECON1, ECON1_BSEL_MASK);

      /* Then OR in bits to get the correct bank */

      if (bank != 0)
        {
          enc_setglobal(priv, ECON1, (bank << ECON1_BSEL_SHIFT));
        }

      /* Then remember the bank setting */

      priv->bank = bank;
    }
}

/****************************************************************************
 * Function: enc_rdbreg
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 ****************************************************************************/
 
static uint8_t enc_rdbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg)
{
  FAR struct spi_dev_s *spi;
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Send the read command and collect the return data. */

  rddata = SPI_SEND(spi, ENC28J60_RCR | GETADDR(ctrlreg));

  /* De-select ENC28J60 chip */

  enc_deselect(spi);
  return rddata;
}

/****************************************************************************
 * Function: enc_rdphymac
 *
 * Description:
 *   Somewhat different timing is required to read from any PHY or MAC
 *   registers.  The PHY/MAC data is returned on the second byte after the
 *   command.
 *
 ****************************************************************************/

static uint8_t enc_rdphymac(FAR struct enc_driver_s *priv, uint8_t ctrlreg)
{
  FAR struct spi_dev_s *spi;
  uint8_t rddata;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* Set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Send the read command (discarding the return data) */

  (void)SPI_SEND(spi, ENC28J60_RCR | GETADDR(ctrlreg));

  /* Do an extra transfer to get the data from the MAC or PHY */

  rddata = SPI_SEND(spi, 0);

  /* De-select ENC28J60 chip */

  enc_deselect(spi);
  return rddata;
}

/****************************************************************************
 * Function: enc_rwrbreg
 *
 * Description:
 *   Set the bank for these next control register access.
 *
 ****************************************************************************/
 
static void enc_rwrbreg(FAR struct enc_driver_s *priv, uint8_t ctrlreg,
                        uint8_t wrdata)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* Set the bank */

  enc_setbank(priv, GETBANK(ctrlreg));

  /* Send the write command */

  (void)SPI_SEND(spi, ENC28J60_WCR | GETADDR(ctrlreg));

  /* Send the data byte */

  (void)SPI_SEND(spi, wrdata);

  /* De-select ENC28J60 chip. */

  enc_deselect(spi);
}

/****************************************************************************
 * Function: enc_rdbuffer
 *
 * Description:
 *   Read a buffer of data.
 *
 ****************************************************************************/

static void enc_rdbuffer(FAR struct enc_driver_s *priv, FAR uint8_t *buffer,
                         size_t buflen)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* Send the read buffer memory command (ignoring the response) */

  (void)SPI_SEND(spi, ENC28J60_RBM);
 
  /* Then read the buffer data */

  SPI_RECVBLOCK(spi, buffer, buflen);

  /* De-select ENC28J60 chip. */

  enc_deselect(spi);
}

/****************************************************************************
 * Function: enc_wrbuffer
 *
 * Description:
 *   Write a buffer of data.
 *
 ****************************************************************************/

static void enc_wrbuffer(FAR struct enc_driver_s *priv,
                         FAR const uint8_t *buffer, size_t buflen)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(priv && priv->spi);
  spi = priv->spi;

  /* Select ENC28J60 chip */

  enc_select(spi);

  /* Send the write buffer memory command (ignoring the response) */

  (void)SPI_SEND(spi, ENC28J60_WBM);
 
  /* Then send the buffer */

  SPI_SNDBLOCK(spi, buffer, buflen);

  /* De-select ENC28J60 chip. */

  enc_deselect(spi);
}

/****************************************************************************
 * Function: enc_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txifs interrupt
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

static int enc_transmit(FAR struct enc_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet */

  /* Increment statistics */

  /* Disable Ethernet interrupts */

  /* Send the packet: address=priv->dev.d_buf, length=priv->dev.d_len */

  /* Restore Ethernet interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, ENC28J60_TXTIMEOUT, enc_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: enc_uiptxpoll
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

static int enc_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      uip_arp_out(&priv->dev);
      enc_transmit(priv);

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
 * Function: enc_linkstatus
 *
 * Description:
 *   The current link status can be obtained from the PHSTAT1.LLSTAT or
 *   PHSTAT2.LSTAT.
 *
 ****************************************************************************/

static void enc_linkstatus(FAR struct enc_driver_s *priv)
{
#warning "Missing logic"
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
 *
 ****************************************************************************/

static void enc_txif(FAR struct enc_driver_s *priv)
{
  /* Update statistics */

#ifdef CONFIG_ENC28J60_STATS
  priv->txifs++;
  if (enc_rdgreg(priv, ESTAT) & ESTAT_TXABRT)
    {
      priv->txabrts++;
    }
#endif

  /* Clear the request to send bit */

  enc_clrglobal(priv, ECON1, ECON1_TXRTS);

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(priv->txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->dev, enc_uiptxpoll);
}

/****************************************************************************
 * Function: enc_txerif
 *
 * Description:
 *   An TXERIF interrupt was received indicating that a TX abort has occurred.
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

static void enc_txerif(FAR struct enc_driver_s *priv)
{
  /* Update statistics */

#ifdef CONFIG_ENC28J60_STATS
  priv->txerifs++;
#endif

  /* Here we really should read the TSV and determine if we should re-transmit
   * the packet by resetting TXRTS... maybe someday.
   */

  enc_clrglobal(priv, ECON1, ECON1_TXRTS);

  /* Reset TX */

  enc_setglobal(priv, ECON1, ECON1_TXRST);
  enc_clrglobal(priv, ECON1, ECON1_TXRST);
}

/****************************************************************************
 * Function: enc_rxerif
 *
 * Description:
 *   An RXERIF interrupt was received indicating that the last TX packet(s) is
 *   done
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

static void enc_rxerif(FAR struct enc_driver_s *priv)
{
  /* Update statistics */

#ifdef CONFIG_ENC28J60_STATS
  priv->rxerifs++;
#endif
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
 *
 ****************************************************************************/

static void enc_pktif(FAR struct enc_driver_s *priv)
{
  /* Check for errors and update statistics */
#warning "Missing logic"

  /* Check if the packet is a valid size for the uIP buffer configuration */
#warning "Missing logic"

  /* Copy the data data from the hardware to priv->dev.d_buf.  Set
   * amount of data in priv->dev.d_len
   */
#warning "Missing logic"

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
          enc_transmit(priv);
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
           enc_transmit(priv);
         }
     }
}

/****************************************************************************
 * Function: enc_worker
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

static void enc_worker(FAR void *arg)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;
  uint8_t eir;

  DEBUGASSERT(priv);

  /* Disable further interrupts by clearing the global interrup enable bit */

  enc_clrglobal(priv, EIE, EIE_INTIE);

  /* Loop until all interrupts have been processed (EIR==0).  Note that
   * there is no infinite loop check... if there are always pending interrupts,
   * we are just broken.
   */

  while ((eir = enc_rdgreg(priv, EIR) & EIR_ALLINTS) != 0)
    {
      /* Handle interrupts according to interrupt register register bit
       * settings
       *
       * DMAIF: The DMA interrupt indicates that the DMA module has completed
       * its memory copy or checksum calculation. Additionally, this interrupt
       * will be caused if the host controller cancels a DMA operation by
       * manually clearing the DMAST bit. Once set, DMAIF can only be cleared
       * by the host controller or by a Reset condition.
       */

      if ((eir & EIR_DMAIF) != 0) /* DMA interrupt */
        {
          /* Not used by this driver. Just clear the interrupt request. */

          enc_clrglobal(priv, EIR, EIR_DMAIF);
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
          enc_linkstatus(priv);   /* Get current link status */
          enc_rdphy(priv, PHIR);  /* Clear the LINKIF interrupt */
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
          enc_txif(priv);                   /* Handle TX completion */
          enc_clrglobal(priv, EIR, EIR_TXIF); /* Clear the TXIF interrupt */
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
       *    the packet because the medium was constantly occupied for too long.
       *    The deferral limit (2.4287 ms) was reached and the MACON4.DEFER bit
       *    was clear.
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
       * which cause the transmit error interrupt meet the requirements of the
       * transmit interrupt. As a result, when this interrupt occurs, TXIF
       * will also be simultaneously set.
       */

      if ((eir & EIR_TXERIF) != 0) /* Transmit Error Interrupts */
        {
          enc_txerif(priv);                     /* Handle the TX error */
          enc_clrglobal(priv, EIR, EIR_TXERIF); /* Clear the TXERIF interrupt */
        }

      /* PKTIF The Receive Packet Pending Interrupt Flag (PKTIF) is used to
       * indicate the presence of one or more data packets in the receive
       * buffer and to provide a notification means for the arrival of new
       * packets. When the receive buffer has at least one packet in it,
       * EIR.PKTIF will be set. In other words, this interrupt flag will be
       * set anytime the Ethernet Packet Count register (EPKTCNT) is non-zero.
       *
       * The PKTIF bit can only be cleared by the host controller or by a Reset
       * condition. In order to clear PKTIF, the EPKTCNT register must be
       * decremented to 0. If the last data packet in the receive buffer is
       * processed, EPKTCNT will become zero and the PKTIF bit will automatically
       * be cleared.
       */

      /* Ignore PKTIF because is unreliable. Use EPKTCNT instead */
      /* if ((eir & EIR_PKTIF) != 0) */
        {
          uint8_t pktcnt = enc_rdbreg(priv, EPKTCNT);
          if (pktcnt > 0)
            {
#ifdef CONFIG_ENC28J60_STATS
              if (pkcnt > priv->maxpktcnt)
                {
                  priv->maxpktcnt = pktcnt;
                }
#endif
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

      if ((eir & EIR_RXERIF) != 0) /* Receive Errror Interrupts */
        {
          enc_rxerif(priv);                     /* Handle the RX error */
          enc_clrglobal(priv, EIR, EIR_RXERIF); /* Clear the RXERIF interrupt */
        }

    }

  /* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if 
   * there are no pending transmissions.
   */

  enc_setglobal(priv, EIE, EIE_INTIE);
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
  register FAR struct enc_driver_s *priv = &g_enc28j60[0];

  DEBUGASSERT(priv->irq == irq);

#ifdef CONFIG_ENC28J60_OWNBUS
  /* In very simple environments, we own the SPI and can do data transfers
   * from the interrupt handler.  That is actually a very bad idea in any
   * case because it keeps interrupts disabled for a long time.
   */

  enc_worker((FAR void*)priv);
  return OK;
#else
  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  return work_queue(&priv->work, enc_worker, (FAR void *)priv, 0);
#endif
}

/****************************************************************************
 * Function: enc_txtimeout
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

static void enc_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->dev, enc_uiptxpoll);
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

  /* Check if there is room in the send another TXr packet.  */

  /* If so, update TCP timing states and poll uIP for new XMIT data */

  (void)uip_timer(&priv->dev, enc_uiptxpoll, ENC28J60_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, ENC28J60_WDDELAY, enc_polltimer, 1, arg);
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

static int enc_ifup(struct uip_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initilize Ethernet interface */

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, ENC28J60_WDDELAY, enc_polltimer, 1, (uint32_t)priv);

  /* Enable the Ethernet interrupt */

  priv->bifup = true;
  up_enable_irq(priv->irq);
  return OK;
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

static int enc_ifdown(struct uip_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(priv->irq);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Reset the device */

  priv->bifup = false;
  irqrestore(flags);
  return OK;
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

static int enc_txavail(struct uip_driver_s *dev)
{
  FAR struct enc_driver_s *priv = (FAR struct enc_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {

      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&priv->dev, enc_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: enc_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.  The ENC28J60 device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Parameters:
 *   spi - A reference to the platform's SPI driver for the ENC28J60
 *   irq - The fully configured GPIO IRQ that ENC28J60 interrupts will be
 *         asserted on.  This driver will attach and entable this IRQ.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the Ethernet controller and driver */

int enc_initialize(FAR struct spi_dev_s *spi, int irq)
{
  /* Initialize and configure the ENC28J60 */

  /* Initialize the driver structure */

  memset(g_enc28j60, 0, CONFIG_ENC28J60_NINTERFACES*sizeof(struct enc_driver_s));
  g_enc28j60[0].dev.d_ifup    = enc_ifup;     /* I/F down callback */
  g_enc28j60[0].dev.d_ifdown  = enc_ifdown;   /* I/F up (new IP address) callback */
  g_enc28j60[0].dev.d_txavail = enc_txavail;  /* New TX data callback */
  g_enc28j60[0].dev.d_private = (void*)g_enc28j60; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  g_enc28j60[0].txpoll       = wd_create();   /* Create periodic poll timer */
  g_enc28j60[0].txtimeout    = wd_create();   /* Create TX timeout timer */
  g_enc28j60[0].spi          = spi;           /* Save the SPI instance */
  g_enc28j60[0].irq          = irq;           /* Save the IRQ number */

  /* Attach the IRQ to the driver */

  if (irq_attach(irq, enc_interrupt))
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

