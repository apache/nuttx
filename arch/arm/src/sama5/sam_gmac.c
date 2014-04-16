/****************************************************************************
 * arch/arm/src/sama5/sam_gmac.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <queue.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "up_arch.h"
#include "up_internal.h"
#include "cache.h"

#include "chip.h"
#include "chip/sam_pinmap.h"
#include "sam_pio.h"
#include "sam_periphclks.h"
#include "sam_memories.h"
#include "sam_ethernet.h"

#include <arch/board/board.h>

#if defined(CONFIG_NET) && defined(CONFIG_SAMA5_GMAC)

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Number of buffer for RX */

#ifndef CONFIG_SAMA5_GMAC_NRXBUFFERS
#  define CONFIG_SAMA5_GMAC_NRXBUFFERS  16
#endif

/* Number of buffer for TX */

#ifndef CONFIG_SAMA5_GMAC_NTXBUFFERS
#  define CONFIG_SAMA5_GMAC_NTXBUFFERS  8
#endif

#undef CONFIG_SAMA5_GMAC_NBC

#ifndef CONFIG_SAMA5_GMAC_PHYADDR
#  error "CONFIG_SAMA5_GMAC_PHYADDR must be defined in the NuttX configuration"
#endif

/* PHY definitions */

#ifdef SAMA5_GMAC_PHY_KSZ90x1
#  define GMII_OUI_MSB       0x0022
#  define GMII_OUI_LSB       GMII_PHYID2_OUI(5)
#else
#  error Unknown PHY
#endif

/* GMAC buffer sizes, number of buffers, and number of descriptors.
 *
 * REVISIT: The CONFIG_NET_MULTIBUFFER might be useful.  It might be possible
 * to use this option to send and receive messages directly into the DMA
 * buffers, saving a copy.  There might be complications on the receiving
 * side, however, where buffers may wrap and where the size of the received
 * frame will typically be smaller than a full packet.
 */

#ifdef CONFIG_NET_MULTIBUFFER
#  error CONFIG_NET_MULTIBUFFER must not be set
#endif

#define GMAC_RX_UNITSIZE 128                 /* Fixed size for RX buffer  */
#define GMAC_TX_UNITSIZE CONFIG_NET_BUFSIZE  /* MAX size for Ethernet packet */

/* The MAC can support frame lengths up to 1536 bytes */

#define GMAC_MAX_FRAMELEN       1536
#if CONFIG_NET_BUFSIZE >GMAC_MAX_FRAMELEN
#  error CONFIG_NET_BUFSIZE is too large
#endif

/* We need at least one more free buffer than transmit buffers */

#define SAM_GMAC_NFREEBUFFERS (CONFIG_SAMA5_GMAC_NTXBUFFERS+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAMA5_GMAC_REGDEBUG
#endif

#ifdef CONFIG_NET_DUMPPACKET
#  define sam_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define sam_dumppacket(m,a,n)
#endif

/* Timing *******************************************************************/
/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define SAM_WDDELAY     (1*CLK_TCK)
#define SAM_POLLHSEC    (1*2)

/* TX timeout = 1 minute */

#define SAM_TXTIMEOUT   (60*CLK_TCK)

/* PHY read/write delays in loop counts */

#define PHY_RETRY_MAX   300000

/* Helpers ******************************************************************/
/* This is a helper pointer for accessing the contents of the GMAC
 * header
 */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The sam_gmac_s encapsulates all state information for the GMAC peripheral */

struct sam_gmac_s
{
  uint8_t               ifup    : 1; /* true:ifup false:ifdown */
  WDOG_ID               txpoll;      /* TX poll timer */
  WDOG_ID               txtimeout;   /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s   dev;         /* Interface understood by uIP */

  /* Used to track transmit and receive descriptors */

  uint8_t               phyaddr;     /* PHY address (pre-defined by pins on reset) */
  uint16_t              txhead;      /* Circular buffer head index */
  uint16_t              txtail;      /* Circualr buffer tail index */
  uint16_t              rxndx;       /* RX index for current processing RX descriptor */

  uint8_t              *rxbuffer;    /* Allocated RX buffers */
  uint8_t              *txbuffer;    /* Allocated TX buffers */
  struct gmac_rxdesc_s *rxdesc;      /* Allocated RX descriptors */
  struct gmac_txdesc_s *txdesc;      /* Allocated TX descriptors */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_GMAC_REGDEBUG
   bool               wrlast;     /* Last was a write */
   uintptr_t          addrlast;   /* Last address */
   uint32_t           vallast;    /* Last value */
   int                ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The driver state singleton */

static struct sam_gmac_s g_gmac;

#ifdef CONFIG_SAMA5_GMAC_PREALLOCATE
/* Preallocated data */
/* TX descriptors list */

static struct gmac_txdesc_s g_txdesc[CONFIG_SAMA5_GMAC_NTXBUFFERS]
              __attribute__((aligned(8)));

/* RX descriptors list */

static struct gmac_rxdesc_s g_rxdesc[CONFIG_SAMA5_GMAC_NRXBUFFERS]
              __attribute__((aligned(8)));

/* Transmit Buffers
 *
 * Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
 * Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
 * shall be set to 0
 */

static uint8_t g_txbuffer[CONFIG_SAMA5_GMAC_NTXBUFFERS * GMAC_TX_UNITSIZE]
               __attribute__((aligned(8)));

/* Receive Buffers */

static uint8_t g_rxbuffer[CONFIG_SAMA5_GMAC_NRXBUFFERS * GMAC_RX_UNITSIZE]
               __attribute__((aligned(8)));
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAMA5_GMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool sam_checkreg(struct sam_gmac_s *priv, bool wr,
                         uint32_t regval, uintptr_t address);
static uint32_t sam_getreg(struct sam_gmac_s *priv, uintptr_t addr);
static void sam_putreg(struct sam_gmac_s *priv, uintptr_t addr, uint32_t val);
#else
# define sam_getreg(priv,addr)      getreg32(addr)
# define sam_putreg(priv,addr,val)  putreg32(val,addr)
#endif

/* Buffer management */

static uint16_t sam_txinuse(struct sam_gmac_s *priv);
static uint16_t sam_txfree(struct sam_gmac_s *priv);
static int  sam_buffer_initialize(struct sam_gmac_s *priv);
static void sam_buffer_free(struct sam_gmac_s *priv);

/* Common TX logic */

static int  sam_transmit(struct sam_gmac_s *priv);
static int  sam_uiptxpoll(struct uip_driver_s *dev);
static void sam_dopoll(struct sam_gmac_s *priv);

/* Interrupt handling */

static int  sam_recvframe(struct sam_gmac_s *priv);
static void sam_receive(struct sam_gmac_s *priv);
static void sam_txdone(struct sam_gmac_s *priv);
static int  sam_gmac_interrupt(int irq, void *context);

/* Watchdog timer expirations */

static void sam_polltimer(int argc, uint32_t arg, ...);
static void sam_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int  sam_ifup(struct uip_driver_s *dev);
static int  sam_ifdown(struct uip_driver_s *dev);
static int  sam_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int  sam_addmac(struct uip_driver_s *dev, const uint8_t *mac);
static int  sam_rmmac(struct uip_driver_s *dev, const uint8_t *mac);
#endif

/* PHY Initialization */

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_VERBOSE)
static void sam_phydump(struct sam_gmac_s *priv);
#else
#  define sam_phydump(priv)
#endif

static void sam_enablemdio(struct sam_gmac_s *priv);
static void sam_disablemdio(struct sam_gmac_s *priv);
static int  sam_phywait(struct sam_gmac_s *priv);
static int  sam_phyreset(struct sam_gmac_s *priv);
static int  sam_phyfind(struct sam_gmac_s *priv, uint8_t *phyaddr);
static int  sam_phyread(struct sam_gmac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t *phyval);
static int  sam_phywrite(struct sam_gmac_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t phyval);
#ifdef CONFIG_SAMA5_GMAC_AUTONEG
static int  sam_autonegotiate(struct sam_gmac_s *priv);
#else
static void sam_linkspeed(struct sam_gmac_s *priv);
#endif
static void sam_mdcclock(struct sam_gmac_s *priv);
static int  sam_phyinit(struct sam_gmac_s *priv);

/* GMAC Initialization */

static void sam_txreset(struct sam_gmac_s *priv);
static void sam_rxreset(struct sam_gmac_s *priv);
static void sam_gmac_reset(struct sam_gmac_s *priv);
static void sam_macaddress(struct sam_gmac_s *priv);
static int  sam_gmac_configure(struct sam_gmac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_GMAC_REGDEBUG
static bool sam_checkreg(struct sam_gmac_s *priv, bool wr, uint32_t regval,
                         uintptr_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_GMAC_REGDEBUG
static uint32_t sam_getreg(struct sam_gmac_s *priv, uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (sam_checkreg(priv, false, regval, address))
    {
      lldbg("%08x->%08x\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_GMAC_REGDEBUG
static void sam_putreg(struct sam_gmac_s *priv, uintptr_t address,
                       uint32_t regval)
{
  if (sam_checkreg(priv, true, regval, address))
    {
      lldbg("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

/****************************************************************************
 * Function: sam_txinuse
 *
 * Description:
 *   Return the number of TX buffers in-use
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   The number of TX buffers in-use
 *
 ****************************************************************************/

static uint16_t sam_txinuse(struct sam_gmac_s *priv)
{
  uint32_t txhead32 = (uint32_t)priv->txhead;
  if ((uint32_t)priv->txtail > txhead32)
    {
      return txhead32 += CONFIG_SAMA5_GMAC_NTXBUFFERS;
    }

  return (uint16_t)(txhead32 - (uint32_t)priv->txtail);
}

/****************************************************************************
 * Function: sam_txfree
 *
 * Description:
 *   Return the number of TX buffers available
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   The number of TX buffers available
 *
 ****************************************************************************/

static uint16_t sam_txfree(struct sam_gmac_s *priv)
{
  /* The number available is equal to the total number of buffers, minus the
   * number of buffers in use.  Notice that that actual number of buffers is
   * the configured size minus 1.
   */

  return (CONFIG_SAMA5_GMAC_NTXBUFFERS-1) - sam_txinuse(priv);
}

/****************************************************************************
 * Function: sam_buffer_initialize
 *
 * Description:
 *   Allocate aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function degenerates to a few assignements.
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

static int sam_buffer_initialize(struct sam_gmac_s *priv)
{
#ifdef CONFIG_SAMA5_GMAC_PREALLOCATE
  /* Use pre-allocated buffers */

  priv->txdesc   = g_txdesc;
  priv->rxdesc   = g_rxdesc;
  priv->txbuffer = g_txbuffer;
  priv->rxbuffer = g_rxbuffer;

#else
  size_t allocsize;

  /* Allocate buffers */

  allocsize = CONFIG_SAMA5_GMAC_NTXBUFFERS * sizeof(struct gmac_txdesc_s);
  priv->txdesc = (struct gmac_txdesc_s *)kmemalign(8, allocsize);
  if (!priv->txdesc)
    {
      nlldbg("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  memset(priv->txdesc, 0, allocsize);

  allocsize = CONFIG_SAMA5_GMAC_NRXBUFFERS * sizeof(struct gmac_rxdesc_s);
  priv->rxdesc = (struct gmac_rxdesc_s *)kmemalign(8, allocsize);
  if (!priv->rxdesc)
    {
      nlldbg("ERROR: Failed to allocate RX descriptors\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  memset(priv->rxdesc, 0, allocsize);

  allocsize = CONFIG_SAMA5_GMAC_NTXBUFFERS * GMAC_TX_UNITSIZE;
  priv->txbuffer = (uint8_t *)kmemalign(8, allocsize);
  if (!priv->txbuffer)
    {
      nlldbg("ERROR: Failed to allocate TX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  allocsize = CONFIG_SAMA5_GMAC_NRXBUFFERS * GMAC_RX_UNITSIZE;
  priv->rxbuffer = (uint8_t *)kmemalign(8, allocsize);
  if (!priv->rxbuffer)
    {
      nlldbg("ERROR: Failed to allocate RX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

#endif

  DEBUGASSERT(((uintptr_t)priv->rxdesc   & 7) == 0 &&
              ((uintptr_t)priv->rxbuffer & 7) == 0 &&
              ((uintptr_t)priv->txdesc   & 7) == 0 &&
              ((uintptr_t)priv->txbuffer & 7) == 0);
  return OK;
}

/****************************************************************************
 * Function: sam_buffer_free
 *
 * Description:
 *   Free aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function does nothing.
 *
 * Input Parameters:
 *   priv - The GMAC driver state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_buffer_free(struct sam_gmac_s *priv)
{
#ifndef CONFIG_SAMA5_GMAC_PREALLOCATE
  /* Free allocated buffers */

  if (priv->txdesc)
    {
      kfree(priv->txdesc);
      priv->txdesc = NULL;
    }

  if (priv->rxdesc)
    {
      kfree(priv->rxdesc);
      priv->rxdesc = NULL;
    }

  if (priv->txbuffer)
    {
      kfree(priv->txbuffer);
      priv->txbuffer = NULL;
    }

  if (priv->rxbuffer)
    {
      kfree(priv->rxbuffer);
      priv->rxbuffer = NULL;
    }
#endif
}

/****************************************************************************
 * Function: sam_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the TX done interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int sam_transmit(struct sam_gmac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;
  volatile struct gmac_txdesc_s *txdesc;
  uintptr_t virtaddr;
  uint32_t regval;
  uint32_t status;

  nllvdbg("d_len: %d txhead: %d txtail: %d\n",
          dev->d_len, priv->txhead, priv->txtail);
  sam_dumppacket("Transmit packet", dev->d_buf, dev->d_len);

  /* Check parameter */

  if (dev->d_len > GMAC_TX_UNITSIZE)
    {
      nlldbg("ERROR: Packet too big: %d\n", dev->d_len);
      return -EINVAL;
    }

  /* Pointer to the current TX descriptor */

  txdesc = &priv->txdesc[priv->txhead];

  /* If no free TX descriptor, buffer can't be sent */

  if (sam_txfree(priv) < 1)
    {
      nlldbg("ERROR: No free TX descriptors\n");
      return -EBUSY;
    }

  /* Setup/Copy data to transmition buffer */

  if (dev->d_len > 0)
    {
      /* Driver managed the ring buffer */

      virtaddr = sam_virtramaddr(txdesc->addr);
      memcpy((void *)virtaddr, dev->d_buf, dev->d_len);
      cp15_clean_dcache((uint32_t)virtaddr, (uint32_t)virtaddr + dev->d_len);
    }

  /* Update TX descriptor status. */

  status = dev->d_len | GMACTXD_STA_LAST;
  if (priv->txhead == CONFIG_SAMA5_GMAC_NTXBUFFERS-1)
    {
      status |= GMACTXD_STA_WRAP;
    }

  /* Update the descriptor status and flush the updated value to RAM */

  txdesc->status = status;
  cp15_clean_dcache((uint32_t)txdesc,
                    (uint32_t)txdesc + sizeof(struct gmac_txdesc_s));

  /* Increment the head index */

  if (++priv->txhead >= CONFIG_SAMA5_GMAC_NTXBUFFERS)
    {
      priv->txhead = 0;
    }

  /* Now start transmission (if it is not already done) */

  regval  = sam_getreg(priv, SAM_GMAC_NCR);
  regval |= GMAC_NCR_TSTART;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, SAM_TXTIMEOUT, sam_txtimeout, 1,
                 (uint32_t)priv);

  /* Set d_len to zero meaning that the d_buf[] packet buffer is again
   * available.
   */

  dev->d_len = 0;

  /* If we have no more available TX descriptors, then we must disable the
   * RCOMP interrupt to stop further RX processing.  Why?  Because EACH RX
   * packet that is dispatch is also an opportunity to replay with the a TX
   * packet.  So, if we cannot handle an RX packet replay, then we disable
   * all RX packet processing.
   */

  if (sam_txfree(priv) < 1)
    {
      nllvdbg("Disabling RX interrupts\n");
      sam_putreg(priv, SAM_GMAC_IDR, GMAC_INT_RCOMP);
    }

  return OK;
}

/****************************************************************************
 * Function: sam_uiptxpoll
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
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int sam_uiptxpoll(struct uip_driver_s *dev)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      uip_arp_out(&priv->dev);
      sam_transmit(priv);

      /* Check if the there are any free TX descriptors.  We cannot perform
       * the TX poll if we do not have buffering for another packet.
       */

      if (sam_txfree(priv) == 0)
        {
          /* We have to terminate the poll if we have no more descriptors
           * available for another transfer.
           */

          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: sam_dopoll
 *
 * Description:
 *   Perform the uIP poll.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void sam_dopoll(struct sam_gmac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;

  /* Check if the there are any free TX descriptors.  We cannot perform the
   * TX poll if we do not have buffering for another packet.
   */

  if (sam_txfree(priv) > 0)
    {
      /* If we have the descriptor, then poll uIP for new XMIT data. */

      (void)uip_poll(dev, sam_uiptxpoll);
    }
}

/****************************************************************************
 * Function: sam_recvframe
 *
 * Description:
 *   The function is called when a frame is received. It scans the RX
 *   descriptors of the received frame and assembles the full packet/
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   - Global interrupts are disabled by interrupt handling logic.
 *   - The RX descriptor D-cache list has been invalided to force fetching
 *     from RAM.
 *
 ****************************************************************************/

static int sam_recvframe(struct sam_gmac_s *priv)
{
  volatile struct gmac_rxdesc_s *rxdesc;
  struct uip_driver_s *dev;
  const uint8_t *src;
  uint8_t  *dest;
  uintptr_t physaddr;
  uint32_t rxndx;
  uint32_t pktlen;
  uint16_t copylen;
  bool isframe;

  /* Process received RX descriptor.  The ownership bit is set by the GMAC
   * once it has successfully written a frame to memory.
   */

  dev        = &priv->dev;
  dev->d_len = 0;

  rxndx      = priv->rxndx;
  rxdesc     = &priv->rxdesc[rxndx];
  isframe    = false;

  /* Invalidate the RX descriptor to force re-fetching from RAM */

  cp15_invalidate_dcache((uintptr_t)rxdesc,
                         (uintptr_t)rxdesc + sizeof(struct gmac_rxdesc_s));

  nllvdbg("rxndx: %d\n", rxndx);

  while ((rxdesc->addr & GMACRXD_ADDR_OWNER) != 0)
    {
      /* The start of frame bit indicates the beginning of a frame.  Discard
       * any previous fragments.
       */

      if ((rxdesc->status & GMACRXD_STA_SOF) != 0)
        {
          /* Skip previous fragments */

          while (rxndx != priv->rxndx)
            {
              /* Give ownership back to the GMAC */

              rxdesc = &priv->rxdesc[priv->rxndx];
              rxdesc->addr &= ~(GMACRXD_ADDR_OWNER);

              /* Flush the modified RX descriptor to RAM */

              cp15_clean_dcache((uintptr_t)rxdesc,
                                (uintptr_t)rxdesc +
                                sizeof(struct gmac_rxdesc_s));

              /* Increment the RX index */

              if (++priv->rxndx >= CONFIG_SAMA5_GMAC_NRXBUFFERS)
                {
                  priv->rxndx = 0;
                }
            }

          /* Reset the packet data pointer and packet length */

          dest   = dev->d_buf;
          pktlen = 0;

          /* Start to gather buffers into the packet buffer */

          isframe = true;
        }

      /* Increment the working index */

      if (++rxndx >= CONFIG_SAMA5_GMAC_NRXBUFFERS)
        {
          rxndx = 0;
        }

      /* Copy data into the packet buffer */

      if (isframe)
        {
          if (rxndx == priv->rxndx)
            {
              nllvdbg("ERROR: No EOF (Invalid of buffers too small)\n");
              do
                {
                  /* Give ownership back to the GMAC */

                  rxdesc = &priv->rxdesc[priv->rxndx];
                  rxdesc->addr &= ~(GMACRXD_ADDR_OWNER);

                  /* Flush the modified RX descriptor to RAM */

                  cp15_clean_dcache((uintptr_t)rxdesc,
                                    (uintptr_t)rxdesc +
                                    sizeof(struct gmac_rxdesc_s));

                  /* Increment the RX index */

                  if (++priv->rxndx >= CONFIG_SAMA5_GMAC_NRXBUFFERS)
                    {
                      priv->rxndx = 0;
                    }
                }
              while (rxndx != priv->rxndx);
              return -EIO;
            }

          /* Get the number of bytes to copy from the buffer */

          copylen = GMAC_RX_UNITSIZE;
          if ((pktlen + copylen) > CONFIG_NET_BUFSIZE)
            {
              copylen = CONFIG_NET_BUFSIZE - pktlen;
            }

          /* Get the data source.  Invalidate the source memory region to
           * force reload from RAM.
           */

          physaddr = (uintptr_t)(rxdesc->addr & GMACRXD_ADDR_MASK);
          src = (const uint8_t *)sam_virtramaddr(physaddr);

          cp15_invalidate_dcache((uintptr_t)src, (uintptr_t)src + copylen);

          /* And do the copy */

          memcpy(dest, src, copylen);
          dest   += copylen;
          pktlen += copylen;

          /* If the end of frame has been received, return the data */

          if ((rxdesc->status & GMACRXD_STA_EOF) != 0)
            {
              /* Frame size from the GMAC */

              dev->d_len = (rxdesc->status & GMACRXD_STA_FRLEN_MASK);
              nllvdbg("packet %d-%d (%d)\n", priv->rxndx, rxndx, dev->d_len);

              /* All data have been copied in the application frame buffer,
               * release the RX descriptor
               */

              while (priv->rxndx != rxndx)
                {
                  /* Give ownership back to the GMAC */

                  rxdesc = &priv->rxdesc[priv->rxndx];
                  rxdesc->addr &= ~(GMACRXD_ADDR_OWNER);

                  /* Flush the modified RX descriptor to RAM */

                  cp15_clean_dcache((uintptr_t)rxdesc,
                                    (uintptr_t)rxdesc +
                                    sizeof(struct gmac_rxdesc_s));

                  /* Increment the RX index */

                  if (++priv->rxndx >= CONFIG_SAMA5_GMAC_NRXBUFFERS)
                    {
                      priv->rxndx = 0;
                    }
                }

              /* Check if the device packet buffer was large enough to accept
               * all of the data.
               */

              nllvdbg("rxndx: %d d_len: %d\n", priv->rxndx, dev->d_len);

              if (pktlen < dev->d_len)
                {
                  nlldbg("ERROR: Buffer size %d; frame size %d\n", dev->d_len, pktlen);
                  return -E2BIG;
                }

              return OK;
            }
        }

      /* We have not encount the SOF yet... discard this fragment and keep looking */

      else
        {
          /* Give ownership back to the GMAC */

          rxdesc->addr &= ~(GMACRXD_ADDR_OWNER);

          /* Flush the modified RX descriptor to RAM */

          cp15_clean_dcache((uintptr_t)rxdesc,
                            (uintptr_t)rxdesc +
                            sizeof(struct gmac_rxdesc_s));

          priv->rxndx = rxndx;
        }

    /* Process the next buffer */

    rxdesc = &priv->rxdesc[rxndx];

    /* Invalidate the RX descriptor to force re-fetching from RAM */

    cp15_invalidate_dcache((uintptr_t)rxdesc,
                           (uintptr_t)rxdesc + sizeof(struct gmac_rxdesc_s));
  }

  /* No packet was found */

  priv->rxndx = rxndx;
  nllvdbg("rxndx: %d\n", priv->rxndx);
  return -EAGAIN;
}

/****************************************************************************
 * Function: sam_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *   in FIFO memory.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void sam_receive(struct sam_gmac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;

  /* Loop while while sam_recvframe() successfully retrieves valid
   * GMAC frames.
   */

  while (sam_recvframe(priv) == OK)
    {
      sam_dumppacket("Received packet", dev->d_buf, dev->d_len);

      /* Check if the packet is a valid size for the uIP buffer configuration
       * (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_BUFSIZE)
        {
          nlldbg("DROPPED: Too big: %d\n", dev->d_len);
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      else if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      else if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          nllvdbg("IP frame\n");

          /* Handle ARP on input then give the IP packet to uIP */

          uip_arp_ipin(&priv->dev);
          uip_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
           {
             uip_arp_out(&priv->dev);
             sam_transmit(priv);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          nllvdbg("ARP frame\n");

          /* Handle ARP packet */

          uip_arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              sam_transmit(priv);
            }
        }
      else
        {
          nlldbg("DROPPED: Unknown type: %04x\n", BUF->type);
        }
    }
}

/****************************************************************************
 * Function: sam_txdone
 *
 * Description:
 *   An interrupt was received indicating that a frame has completed
 *   transmission.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void sam_txdone(struct sam_gmac_s *priv)
{
  struct gmac_txdesc_s *txdesc;

  /* Are there any outstanding transmssions?  Loop until either (1) all of
   * the TX have been examined, or (2) until we encounter the first
   * descriptor that is still in use by the hardware.
   */

  while (priv->txhead != priv->txtail)
    {
      /* Yes.. check the next buffer at the tail of the list */

      txdesc = &priv->txdesc[priv->txtail];
      cp15_invalidate_dcache((uintptr_t)txdesc,
                             (uintptr_t)txdesc + sizeof(struct gmac_txdesc_s));

      /* Is this TX descriptor still in use? */

      if ((txdesc->status & GMACTXD_STA_USED) == 0)
        {
          /* Yes.. the descriptor is still in use.  However, I have seen a
           * case (only repeatable on start-up) where the USED bit is never
           * set.  Yikes!  If we have encountered the first still busy
           * descriptor, then we should also have TQBD equal to the descriptor
           * address.  If it is not, then treat is as used anyway.
           */

#warning REVISIT
          if (priv->txtail == 0 &&
              sam_physramaddr((uintptr_t)txdesc) != sam_getreg(priv, SAM_GMAC_TBQB))
            {
              txdesc->status = (uint32_t)GMACTXD_STA_USED;
              cp15_clean_dcache((uintptr_t)txdesc,
                                (uintptr_t)txdesc + sizeof(struct gmac_txdesc_s));
            }
          else
            {
              /* Otherwise, the descriptor is truly in use.  Break out of the
               * loop now.
               */

              break;
            }
        }

      /* Increment the tail index */

      if (++priv->txtail >= CONFIG_SAMA5_GMAC_NTXBUFFERS)
        {
          /* Wrap to the beginning of the TX descriptor list */

          priv->txtail = 0;
        }

      /* At least one TX descriptor is available.  Re-enable RX interrupts.
       * RX interrupts may previously have been disabled when we ran out of
       * TX desciptors (see commits in sam_transmit()).
       */

      sam_putreg(priv, SAM_GMAC_IER, GMAC_INT_RCOMP);
    }

  /* Then poll uIP for new XMIT data */

  sam_dopoll(priv);
}

/****************************************************************************
 * Function: sam_gmac_interrupt
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

static int sam_gmac_interrupt(int irq, void *context)
{
  struct sam_gmac_s *priv = &g_gmac;
  uint32_t isr;
  uint32_t rsr;
  uint32_t tsr;
  uint32_t imr;
  uint32_t regval;
  uint32_t pending;
  uint32_t clrbits;

  isr = sam_getreg(priv, SAM_GMAC_ISR);
  rsr = sam_getreg(priv, SAM_GMAC_RSR);
  tsr = sam_getreg(priv, SAM_GMAC_TSR);
  imr = sam_getreg(priv, SAM_GMAC_IMR);

  pending = isr & ~(imr | GMAC_INT_UNUSED);
  nllvdbg("isr: %08x pending: %08x\n", isr, pending);

  /* Check for the completion of a transmission.  This should be done before
   * checking for received data (because receiving can cause another transmission
   * before we had a chance to handle the last one).
   *
   * ISR:TCOMP is set when a frame has been transmitted. Cleared on read.
   * TSR:COMP is set when a frame has been transmitted. Cleared by writing a
   *   one to this bit.
   */

  if ((pending & GMAC_INT_TCOMP) != 0 || (tsr & GMAC_TSR_TXCOMP) != 0)
    {
      /* A frame has been transmitted */

      clrbits = GMAC_TSR_TXCOMP;

      /* Check for Retry Limit Exceeded (RLE) */

      if ((tsr & GMAC_TSR_RLE) != 0)
        {
          /* Status RLE & Number of discarded buffers */

          clrbits = GMAC_TSR_RLE | sam_txinuse(priv);
          sam_txreset(priv);

          nlldbg("ERROR: Retry Limit Exceeded TSR: %08x\n", tsr);

          regval = sam_getreg(priv, SAM_GMAC_NCR);
          regval |= GMAC_NCR_TXEN;
          sam_putreg(priv, SAM_GMAC_NCR, regval);
        }

      /* Check Collision Occurred (COL) */

      if ((tsr & GMAC_TSR_COL) != 0)
        {
          nlldbg("ERROR: Collision occurred TSR: %08x\n", tsr);
          clrbits |= GMAC_TSR_COL;
        }

      /* Check for Transmit Frame Corruption due to AHB error (TFC) */

      if ((tsr & GMAC_TSR_TFC) != 0)
        {
          nlldbg("ERROR: Buffers exhausted mid-frame TSR: %08x\n", tsr);
          clrbits |= GMAC_TSR_TFC;
        }

      /* Check for Transmit Underrun (UND)
       *
       * ISR:UND is set transmit DMA was not able to read data from memory,
       *   either because the bus was not granted in time, because a not
       *   OK hresp(bus error) was returned or because a used bit was read
       *   midway through frame transmission. If this occurs, the
       *   transmitter forces bad CRC. Cleared by writing a one to this bit.
       */

      if ((tsr & GMAC_TSR_UND) != 0)
        {
          nlldbg("ERROR: Transmit Underrun TSR: %08x\n", tsr);
          clrbits |= GMAC_TSR_UND;
        }

      /* Check for HRESP not OK */

      if ((tsr & GMAC_TSR_HRESP) != 0)
        {
          nlldbg("ERROR: HRESP not OK: %08x\n", tsr);
          clrbits |= GMAC_TSR_HRESP;
        }

      /* Check for Late Collitions (LCO) */

      if ((tsr & GMAC_TSR_LCO) != 0)
        {
          nlldbg("ERROR: Late collision: %08x\n", tsr);
          clrbits |= GMAC_TSR_LCO;
        }

      /* Clear status */

      sam_putreg(priv, SAM_GMAC_TSR, clrbits);

      /* And handle the TX done event */

      sam_txdone(priv);
    }

  /* Check for the receipt of an RX packet.
   *
   * RXCOMP indicates that a packet has been received and stored in memory.
   *   The RXCOMP bit is cleared whent he interrupt status register was read.
   * RSR:REC indicates that one or more frames have been received and placed
   *   in memory. This indication is cleared by writing a one to this bit.
   */

  if ((pending & GMAC_INT_RCOMP) != 0 || (rsr & GMAC_RSR_REC) != 0)
    {
      clrbits = GMAC_RSR_REC;

      /* Check for Receive Overrun.
       *
       * RSR:RXOVR will be set if the RX FIFO is not able to store the
       *   receive frame due to a FIFO overflow, or if the receive status
       *   was not taken at the end of the frame. This bit is also set in
       *   DMA packet buffer mode if the packet buffer overflows. For DMA
       *   operation, the buffer will be recovered if an overrun occurs. This
       *   bit is cleared when set to 1.
       */

      if ((rsr & GMAC_RSR_RXOVR) != 0)
        {
          nlldbg("ERROR: Receiver overrun RSR: %08x\n", rsr);
          clrbits |= GMAC_RSR_RXOVR;
        }

      /* Check for buffer not available (BNA)
       *
       * RSR:BNA means that an attempt was made to get a new buffer and the
       *   pointer indicated that it was owned by the processor. The DMA will
       *   reread the pointer each time an end of frame is received until a
       *   valid pointer is found. This bit is set following each descriptor
       *   read attempt that fails, even if consecutive pointers are
       *   unsuccessful and software has in the mean time cleared the status
       *   flag. Cleared by writing a one to this bit.
       */

      if ((rsr & GMAC_RSR_BNA) != 0)
        {
          nlldbg("ERROR: Buffer not available RSR: %08x\n", rsr);
          clrbits |= GMAC_RSR_BNA;
        }

      /* Check for HRESP not OK (HNO)*/

      if ((rsr & GMAC_RSR_HNO) != 0)
        {
          nlldbg("ERROR: HRESP not OK: %08x\n", rsr);
          clrbits |= GMAC_RSR_HNO;
        }

      /* Clear status */

      sam_putreg(priv, SAM_GMAC_RSR, clrbits);

      /* Handle the received packet */

       sam_receive(priv);
    }

#ifdef CONFIG_DEBUG_NET
  /* Check for PAUSE Frame recieved (PFRE).
   *
   * ISR:PFRE indicates that a pause frame has been received.  Cleared on a read.
   */

  if ((pending & GMAC_INT_PFNZ) != 0)
    {
      nlldbg("Pause frame received\n");
    }

  /* Check for Pause Time Zero (PTZ)
   *
   * ISR:PTZ is set Pause Time Zero
   */

  if ((pending & GMAC_INT_PTZ) != 0)
    {
      nlldbg("Pause TO!\n");
    }
#endif

  return OK;
}

/****************************************************************************
 * Function: sam_txtimeout
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
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void sam_txtimeout(int argc, uint32_t arg, ...)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)arg;

  nlldbg("Timeout!\n");

  /* Then reset the hardware.  Just take the interface down, then back
   * up again.
   */

  sam_ifdown(&priv->dev);
  sam_ifup(&priv->dev);

  /* Then poll uIP for new XMIT data */

  sam_dopoll(priv);
}

/****************************************************************************
 * Function: sam_polltimer
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
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void sam_polltimer(int argc, uint32_t arg, ...)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)arg;
  struct uip_driver_s   *dev  = &priv->dev;

  /* Check if the there are any free TX descriptors.  We cannot perform the
   * TX poll if we do not have buffering for another packet.
   */

  if (sam_txfree(priv) > 0)
    {
      /* Update TCP timing states and poll uIP for new XMIT data. */

      (void)uip_timer(dev, sam_uiptxpoll, SAM_POLLHSEC);
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, SAM_WDDELAY, sam_polltimer, 1, arg);
}

/****************************************************************************
 * Function: sam_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the GMAC interface when an IP address is
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

static int sam_ifup(struct uip_driver_s *dev)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)dev->d_private;
  int ret;

  nlldbg("Bringing up: %d.%d.%d.%d\n",
         dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
         (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Configure the GMAC interface for normal operation. */

  nllvdbg("Initialize the GMAC\n");
  sam_gmac_configure(priv);

  /* Set the MAC address (should have been configured while we were down) */

  sam_macaddress(priv);

  /* Initialize for PHY access */

  ret = sam_phyinit(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phyinit failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_SAMA5_GMAC_AUTONEG
  /* Auto Negotiate, working in RMII mode */

  ret = sam_autonegotiate(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_autonegotiate failed: %d\n", ret);
      return ret;
    }
#else
   /* Just force the configured link speed */

   sam_linkspeed(priv);
#endif

  /* Enable normal MAC operation */

  nllvdbg("Enable normal operation\n");

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, SAM_WDDELAY, sam_polltimer, 1, (uint32_t)priv);

  /* Enable the GMAC interrupt */

  priv->ifup = true;
  up_enable_irq(SAM_IRQ_GMAC);
  return OK;
}

/****************************************************************************
 * Function: sam_ifdown
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

static int sam_ifdown(struct uip_driver_s *dev)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)dev->d_private;
  irqstate_t flags;

  nlldbg("Taking the network down\n");

  /* Disable the GMAC interrupt */

  flags = irqsave();
  up_disable_irq(SAM_IRQ_GMAC);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the GMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the sam_ifup() always
   * successfully brings the interface back up.
   */

  sam_gmac_reset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: sam_txavail
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

static int sam_txavail(struct uip_driver_s *dev)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)dev->d_private;
  irqstate_t flags;

  nllvdbg("ifup: %d\n", priv->ifup);

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->ifup)
    {
      /* Poll uIP for new XMIT data */

      sam_dopoll(priv);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: sam_addmac
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
static int sam_addmac(struct uip_driver_s *dev, const uint8_t *mac)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)dev->d_private;

  nllvdbg("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast routing table */
  /* Add the MAC address to the hardware multicast routing table */
#error "Missing logic"

  return OK;
}
#endif

/****************************************************************************
 * Function: sam_rmmac
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
static int sam_rmmac(struct uip_driver_s *dev, const uint8_t *mac)
{
  struct sam_gmac_s *priv = (struct sam_gmac_s *)dev->d_private;

  nllvdbg("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast routing table */
#error "Missing logic"

  return OK;
}
#endif

/****************************************************************************
 * Function: sam_phydump
 *
 * Description:
 *   Dump the contents of PHY registers
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_VERBOSE)
static void sam_phydump(struct sam_gmac_s *priv)
{
  uint16_t phyval;

  /* Enable management port */

  sam_enablemdio(priv);

  nllvdbg("GMII Registers (Address %02x)\n", priv->phyaddr);
  sam_phyread(priv, priv->phyaddr, GMII_MCR, &phyval);
  nllvdbg("       MCR: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, GMII_MSR, &phyval);
  nllvdbg("       MSR: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, GMII_ADVERTISE, &phyval);
  nllvdbg(" ADVERTISE: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, GMII_LPA, &phyval);
  nllvdbg("       LPR: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, GMII_1000BTCR, &phyval);
  nllvdbg("  1000BTCR: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, GMII_1000BTSR, &phyval);
  nllvdbg("  1000BTSR: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, GMII_ESTATUS, &phyval);
  nllvdbg("   ESTATUS: %04x\n", phyval);

  /* Disable management port */

  sam_disablemdio(priv);
}
#endif

/****************************************************************************
 * Function: sam_enablemdio
 *
 * Description:
 *  Enable the management port
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_enablemdio(struct sam_gmac_s *priv)
{
  uint32_t regval;
  uint32_t enables;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_GMAC_NCR);
  enables = regval & (GMAC_NCR_RXEN | GMAC_NCR_TXEN);

  regval &= ~(GMAC_NCR_RXEN | GMAC_NCR_TXEN);
  regval |= GMAC_NCR_MPE;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  regval |= enables;
  sam_putreg(priv, SAM_GMAC_NCR, regval);
}

/****************************************************************************
 * Function: sam_disablemdio
 *
 * Description:
 *  Disable the management port
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_disablemdio(struct sam_gmac_s *priv)
{
  uint32_t regval;
  uint32_t enables;

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_GMAC_NCR);
  enables = regval & (GMAC_NCR_RXEN | GMAC_NCR_TXEN);

  regval &= ~(GMAC_NCR_RXEN | GMAC_NCR_TXEN);
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  regval &= ~GMAC_NCR_MPE;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  regval |= enables;
  sam_putreg(priv, SAM_GMAC_NCR, regval);
}

/****************************************************************************
 * Function: sam_phywait
 *
 * Description:
 *  Wait for the PHY to become IDLE
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno (-ETIMEDOUT) on failure.
 *
 ****************************************************************************/

static int sam_phywait(struct sam_gmac_s *priv)
{
  volatile unsigned int retries;

  /* Loop for the configured number of attempts */

  for (retries = 0; retries < PHY_RETRY_MAX; retries++)
    {
      /* Is the PHY IDLE */

      if ((sam_getreg(priv, SAM_GMAC_NSR) & GMAC_NSR_IDLE) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: sam_phyreset
 *
 * Description:
 *  Reset the PHY
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phyreset(struct sam_gmac_s *priv)
{
  uint16_t mcr;
  int timeout;
  int ret;

  nllvdbg(" sam_phyreset\n");

  /* Enable management port */

  sam_enablemdio(priv);

  /* Reset the PHY */

  ret = sam_phywrite(priv, priv->phyaddr, GMII_MCR, GMII_MCR_RESET);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywrite failed: %d\n", ret);
    }

  /* Wait for the PHY reset to complete */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < 10; timeout++)
    {
      mcr = GMII_MCR_RESET;
      int result = sam_phyread(priv, priv->phyaddr, GMII_MCR, &mcr);
      if (result < 0)
        {
          nlldbg("ERROR: Failed to read the MCR register: %d\n", ret);
          ret = result;
        }
      else if ((mcr & GMII_MCR_RESET) == 0)
        {
          ret = OK;
          break;
        }
    }

  /* Disable management port */

  sam_disablemdio(priv);
  return ret;
}

/****************************************************************************
 * Function: sam_phyfind
 *
 * Description:
 *  Verify the PHY address and, if it is bad, try to one that works.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phyfind(struct sam_gmac_s *priv, uint8_t *phyaddr)
{
  uint16_t phyval;
  uint8_t candidate;
  unsigned int offset;
  int ret = -ESRCH;

  nllvdbg("Find a valid PHY address\n");

  /* Enable management port */

  sam_enablemdio(priv);

  /* Check initial candidate address */

  candidate = *phyaddr;

  ret = sam_phyread(priv, candidate, GMII_PHYID1, &phyval);
  if (ret == OK && phyval == GMII_OUI_MSB)
    {
      *phyaddr = candidate;
      ret = OK;
    }

  /* The current address does not work... try another */

  else
    {
      nlldbg("ERROR: sam_phyread failed for PHY address %02x: %d\n",
             candidate, ret);

      for (offset = 0; offset < 32; offset++)
        {
          /* Get the next candidate PHY address */

          candidate = (candidate + 1) & 0x1f;

          /* Try reading the PHY ID from the candidate PHY address */

          ret = sam_phyread(priv, candidate, GMII_PHYID1, &phyval);
          if (ret == OK && phyval == GMII_OUI_MSB)
            {
              ret = OK;
              break;
            }
        }
    }

  if (ret == OK)
    {
      nllvdbg("  PHYID1: %04x PHY addr: %d\n", phyval, candidate);
      *phyaddr = candidate;
    }

  /* Disable management port */

  sam_disablemdio(priv);
  return ret;
}

/****************************************************************************
 * Function: sam_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *   phyaddr - The PHY device address
 *   regaddr - The PHY register address
 *   phyval - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phyread(struct sam_gmac_s *priv, uint8_t phyaddr,
                       uint8_t regaddr, uint16_t *phyval)
{
  uint32_t regval;
  int ret;

  /* Make sure that the PHY is idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Write the PHY Maintenance register */

  regval = GMAC_MAN_DATA(0) | GMAC_MAN_WTN | GMAC_MAN_REGA(regaddr) |
           GMAC_MAN_PHYA(phyaddr) | GMAC_MAN_READ | GMAC_MAN_CLTTO;
  sam_putreg(priv, SAM_GMAC_MAN, regval);

  /* Wait until the PHY is again idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Return the PHY data */

  *phyval = (uint16_t)(sam_getreg(priv, SAM_GMAC_MAN) & GMAC_MAN_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: sam_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *   phyaddr - The PHY device address
 *   regaddr - The PHY register address
 *   phyval - The 16-bit value to write to the PHY register.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phywrite(struct sam_gmac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t phyval)
{
  uint32_t regval;
  int ret;

  /* Make sure that the PHY is idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Write the PHY Maintenance register */

  regval = GMAC_MAN_DATA(phyval) | GMAC_MAN_WTN | GMAC_MAN_REGA(regaddr) |
           GMAC_MAN_PHYA(phyaddr) | GMAC_MAN_WRITE | GMAC_MAN_CLTTO;
  sam_putreg(priv, SAM_GMAC_MAN, regval);

  /* Wait until the PHY is again IDLE */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Function: sam_autonegotiate
 *
 * Description:
 *  Autonegotiate speed and duplex.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_GMAC_AUTONEG
static int sam_autonegotiate(struct sam_gmac_s *priv)
{
  uint32_t regval;
  uint32_t ncr;
  uint32_t linkmode;
  uint16_t phyval;
  uint16_t phyid1;
  uint16_t phyid2;
  uint16_t advertise;
  uint16_t lpa;
  uint16_t btcr;
  uint16_t btsr;
  int timeout;
  int ret;

  /* Enable management port */

  sam_enablemdio(priv);

  /* Read the MS bits of the OUI from Pthe PHYID1 register */

  ret = sam_phyread(priv, priv->phyaddr, GMII_PHYID1, &phyid1);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read PHYID1 register\n");
      goto errout;
    }

  nllvdbg("PHYID1: %04x PHY address: %02x\n", phyid1, priv->phyaddr);

  /* Read the LS bits of the OUI from Pthe PHYID2 register */

  ret = sam_phyread(priv, priv->phyaddr, GMII_PHYID2, &phyid2);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read PHYID2 register\n");
      goto errout;
    }

  nllvdbg("PHYID2: %04x PHY address: %02x\n", phyid2, priv->phyaddr);

  if (phyid1 == GMII_OUI_MSB &&
     (phyid2 & GMII_PHYID2_OUI_MASK) == GMII_OUI_LSB)
    {
      nllvdbg("  Vendor Model Number:   %04x\n",
             (phyid2 & GMII_PHYID2_MODEL_MASK) >> GMII_PHYID2_MODEL_SHIFT);
      nllvdbg("  Model Revision Number: %04x\n",
             (phyid2 & GMII_PHYID2_REV_MASK) >> GMII_PHYID2_REV_SHIFT);
    }
  else
    {
      nlldbg("ERROR: PHY not recognized: PHYID1=%04x PHYID2=%04x\n",
              phyid1, phyid2);
    }

#ifdef SAMA5_GMAC_PHY_KSZ90x1
  /* Set up the KSZ9020/31 PHY */

  phyval = GMII_KSZ90x1_RCCPSR | GMII_ERCR_WRITE;
  sam_phywrite(priv, priv->phyaddr, GMII_ERCR, phyval);
  sam_phywrite(priv, priv->phyaddr, GMII_ERDWR, 0xf2f4);

  phyval = GMII_KSZ90x1_RRDPSR | GMII_ERCR_WRITE;
  sam_phywrite(priv, priv->phyaddr, GMII_ERCR, phyval);
  sam_phywrite(priv, priv->phyaddr, GMII_ERDWR, 0x2222);

  ret = sam_phywrite(priv, priv->phyaddr, GMII_KSZ90x1_ICS, 0xff00);
#endif

  /* Set the Auto_negotiation Advertisement Register, MII advertising for
   * Next page 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3
   */

  advertise = GMII_ADVERTISE_100BASETXFULL | GMII_ADVERTISE_100BASETXHALF |
              GMII_ADVERTISE_10BASETXFULL | GMII_ADVERTISE_10BASETXHALF |
              GMII_ADVERTISE_8023;

  ret = sam_phywrite(priv, priv->phyaddr, GMII_ADVERTISE, advertise);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write ADVERTISE register\n");
      goto errout;
    }

  /* Modify the 1000Base-T control register to advertise 1000Base-T full
   * and half duplex support.
   */

  ret = sam_phyread(priv, priv->phyaddr, GMII_1000BTCR, &btcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read 1000BTCR register: %d\n", ret);
      goto errout;
    }

  btcr |= GMII_1000BTCR_1000BASETFULL | GMII_1000BTCR_1000BASETHALF;

  ret = sam_phywrite(priv, priv->phyaddr, GMII_1000BTCR, btcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write 1000BTCR register: %d\n", ret);
      goto errout;
    }

  /* Restart Auto_negotiation */

  ret  = sam_phyread(priv, priv->phyaddr, GMII_MCR, &phyval);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read MCR register: %d\n", ret);
      goto errout;
    }

  phyval |=  GMII_MCR_ANRESTART;

  ret = sam_phywrite(priv, priv->phyaddr, GMII_MCR, phyval);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write MCR register: %d\n", ret);
      goto errout;
    }

  nllvdbg(" MCR: 0x%X\n", phyval);

  /* Wait for autonegotion to complete */

  timeout = 0;
  for (;;)
    {
      ret  = sam_phyread(priv, priv->phyaddr, GMII_MSR, &phyval);
      if (ret < 0)
        {
          nlldbg("ERROR: Failed to read MSR register: %d\n", ret);
          goto errout;
        }

      /* Check for completion of autonegotiation */

      if ((phyval & GMII_MSR_ANEGCOMPLETE) != 0)
        {
          /* Yes.. break out of the loop */

          nllvdbg("AutoNegotiate complete\n");
          break;
        }

      /* No.. check for a timeout */

      if (++timeout >= PHY_RETRY_MAX)
        {
          nlldbg("ERROR: TimeOut\n");
          sam_phydump(priv);
          ret = -ETIMEDOUT;
          goto errout;
        }
    }

  /* Setup the GMAC local link speed */

  linkmode = 0;  /* 10Base-T Half-Duplex */
  timeout  = 0;

  for (;;)
    {
      ret  = sam_phyread(priv, priv->phyaddr, GMII_1000BTSR, &btsr);
      if (ret < 0)
        {
          nlldbg("ERROR: Failed to read 1000BTSR register: %d\n", ret);
          goto errout;
        }

      /* Setup the GMAC link speed */

      if ((btsr & GMII_1000BTSR_LP1000BASETFULL) != 0 &&
          (btcr & GMII_1000BTCR_1000BASETHALF) != 0)
        {
          /* Set RGMII for 1000BaseTX and Full Duplex */

          linkmode = (GMAC_NCFGR_FD | GMAC_NCFGR_GBE);
          break;
        }
      else if ((btsr & GMII_1000BTSR_LP1000BASETHALF) != 0 &&
               (btcr & GMII_1000BTCR_1000BASETFULL) != 0)
        {
          /* Set RGMII for 1000BaseT and Half Duplex */

          linkmode = GMAC_NCFGR_GBE;
          break;
        }

      /* Get the Autonegotiation Link partner base page */

      ret  = sam_phyread(priv, priv->phyaddr, GMII_LPA, &lpa);
      if (ret < 0)
        {
          nlldbg("ERROR: Failed to read LPA register: %d\n", ret);
          goto errout;
        }

      /* Setup the GMAC link speed */

      if ((advertise & GMII_ADVERTISE_100BASETXFULL) != 0 &&
          (lpa & GMII_LPA_100BASETXFULL) != 0)
        {
          /* Set RGMII for 100BaseTX and Full Duplex */

          linkmode = (GMAC_NCFGR_SPD | GMAC_NCFGR_FD);
          break;
        }
      else if ((advertise & GMII_ADVERTISE_10BASETXFULL) != 0 &&
               (lpa & GMII_LPA_10BASETXFULL) != 0)
        {
          /* Set RGMII for 10BaseT and Full Duplex */

          linkmode = GMAC_NCFGR_FD;
          break;
        }
      else if ((advertise & GMII_ADVERTISE_100BASETXHALF) != 0 &&
               (lpa & GMII_LPA_100BASETXHALF) != 0)
        {
          /* Set RGMII for 100BaseTX and half Duplex */

          linkmode = GMAC_NCFGR_SPD;
          break;
        }
      else if ((advertise & GMII_ADVERTISE_10BASETXHALF) != 0 &&
               (lpa & GMII_LPA_10BASETXHALF) != 0)
        {
          /* Set RGMII for 10BaseT and half Duplex */

          break;
        }

      /* Check for a timeout */

      if (++timeout >= PHY_RETRY_MAX)
        {
          nlldbg("ERROR: TimeOut\n");
          sam_phydump(priv);
          ret = -ETIMEDOUT;
          goto errout;
        }
    }

  /* Disable RX and TX momentarily */

  ncr = sam_getreg(priv, SAM_GMAC_NCR);
  sam_putreg(priv, SAM_GMAC_NCR, ncr & ~(GMAC_NCR_RXEN | GMAC_NCR_TXEN));

  /* Modify the NCFGR register based on the negotiated speed and duplex */

  regval  = sam_getreg(priv, SAM_GMAC_NCFGR);
  regval &= ~(GMAC_NCFGR_SPD | GMAC_NCFGR_FD | GMAC_NCFGR_GBE);
  regval |= linkmode;
  sam_putreg(priv, SAM_GMAC_NCFGR, regval);
  sam_putreg(priv, SAM_GMAC_NCR, ncr);

  /* Enable RGMII enable */

  regval  = sam_getreg(priv, SAM_GMAC_UR);
  regval |= GMAC_UR_RGMII;
  sam_putreg(priv, SAM_GMAC_UR, regval);

errout:
  /* Disable the management port */

  sam_disablemdio(priv);
  return ret;
}
#endif

/****************************************************************************
 * Function: sam_linkspeed
 *
 * Description:
 *  If autonegotiation is not configured, then just force the configuration
 *  mode
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_GMAC_AUTONEG
statoc void sam_linkspeed(struct sam_gmac_s *priv)
{
  uint32_t regval;
  uint32_t ncr;

  /* Disable RX and TX momentarily */

  ncr = sam_getreg(priv, SAM_GMAC_NCR);
  sam_putreg(priv, SAM_GMAC_NCR, ncr & ~(GMAC_NCR_RXEN | GMAC_NCR_TXEN));

  /* Modify the NCFGR register based on the configured speed and duplex */

  regval = sam_getreg(priv, SAM_GMAC_NCFGR);
  regval &= ~(GMAC_NCFGR_SPD | GMAC_NCFGR_FD | GMAC_NCFGR_GBE);

#ifdef SAMA5_GMAC_ETHFD
  regval |= GMAC_NCFGR_FD;
#endif

#if defined(SAMA5_GMAC_ETH100MBPS)
  regval |= GMAC_NCFGR_SPD;
#elif defined(SAMA5_GMAC_ETH1000MBPS) */
  regval |= GMAC_NCFGR_GBE;
#endif

  sam_puttreg(priv, SAM_GMAC_NCFGR, regval);
  sam_putreg(priv, SAM_GMAC_NCR, ncr);
}
#endif

/****************************************************************************
 * Function: sam_mdcclock
 *
 * Description:
 *  Configure the MDC clocking
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_mdcclock(struct sam_gmac_s *priv)
{
  uint32_t ncfgr;
  uint32_t ncr;
  uint32_t mck;

  /* Disable RX and TX momentarily */

  ncr = sam_getreg(priv, SAM_GMAC_NCR);
  sam_putreg(priv, SAM_GMAC_NCR, ncr & ~(GMAC_NCR_RXEN | GMAC_NCR_TXEN));

  /* Modify the NCFGR register based on the configured board MCK frequency */

  ncfgr  = sam_getreg(priv, SAM_GMAC_NCFGR);
  ncfgr &= ~GMAC_NCFGR_CLK_MASK;

  mck = BOARD_MCK_FREQUENCY;
  DEBUGASSERT(mck <= 240000000);

  if (mck <= 20000000)
    {
      ncfgr |= GMAC_NCFGR_CLK_DIV8;   /* MCK divided by 8 (MCK up to 20 MHz) */
    }
  else if (mck <= 40000000)
    {
      ncfgr |= GMAC_NCFGR_CLK_DIV16;  /* MCK divided by 16 (MCK up to 40 MHz) */
    }
  else if (mck <= 80000000)
    {
      ncfgr |= GMAC_NCFGR_CLK_DIV32;  /* MCK divided by 32 (MCK up to 80 MHz) */
    }
  else if (mck <= 120000000)
    {
      ncfgr |= GMAC_NCFGR_CLK_DIV48;  /* MCK divided by 48 (MCK up to 120 MHz) */
    }
  else if (mck <= 160000000)
    {
      ncfgr |= GMAC_NCFGR_CLK_DIV64;  /* MCK divided by 64 (MCK up to 160 MHz) */
    }
  else /* if (mck <= 240000000) */
    {
      ncfgr |= GMAC_NCFGR_CLK_DIV96;  /* MCK divided by 64 (MCK up to 240 MHz) */
    }

  sam_putreg(priv, SAM_GMAC_NCFGR, ncfgr);

  /* Restore RX and TX enable settings */

  sam_putreg(priv, SAM_GMAC_NCR, ncr);
}

/****************************************************************************
 * Function: sam_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int sam_phyinit(struct sam_gmac_s *priv)
{
  int ret;

  /* Configure PHY clocking */

  sam_mdcclock(priv);

  /* Check the PHY Address */

  priv->phyaddr = CONFIG_SAMA5_GMAC_PHYADDR;
  ret = sam_phyfind(priv, &priv->phyaddr);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phyfind failed: %d\n", ret);
      return ret;
    }

  /* We have a PHY address.  Reset the PHY */

  sam_phyreset(priv);
  return OK;
}

/****************************************************************************
 * Function: sam_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the GMAC interface.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void sam_ethgpioconfig(struct sam_gmac_s *priv)
{
  /* Configure PIO pins to support GMAC in RGMII mode */

  sam_configpio(PIO_GMAC_TX0);
  sam_configpio(PIO_GMAC_TX1);
  sam_configpio(PIO_GMAC_TX2);
  sam_configpio(PIO_GMAC_TX3);

  sam_configpio(PIO_GMAC_RX0);
  sam_configpio(PIO_GMAC_RX1);
  sam_configpio(PIO_GMAC_RX2);
  sam_configpio(PIO_GMAC_RX3);

  sam_configpio(PIO_GMAC_TXCK);
  sam_configpio(PIO_GMAC_TXEN);
  sam_configpio(PIO_GMAC_RXCK);
  sam_configpio(PIO_GMAC_RXDV);
  sam_configpio(PIO_GMAC_RXER);

  sam_configpio(PIO_GMAC_MDC);
  sam_configpio(PIO_GMAC_MDIO);
  sam_configpio(PIO_GMAC_125CK);
}

/****************************************************************************
 * Function: sam_txreset
 *
 * Description:
 *  Reset the transmit logic
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_txreset(struct sam_gmac_s *priv)
{
  uint8_t *txbuffer = priv->txbuffer;
  struct gmac_txdesc_s *txdesc = priv->txdesc;
  uintptr_t bufaddr;
  uint32_t physaddr;
  uint32_t regval;
  int ndx;

  /* Disable TX */

  regval  = sam_getreg(priv, SAM_GMAC_NCR);
  regval &= ~GMAC_NCR_TXEN;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  /* Configure the TX descriptors. */

  priv->txhead = 0;
  priv->txtail = 0;

  for (ndx = 0; ndx < CONFIG_SAMA5_GMAC_NTXBUFFERS; ndx++)
    {
      bufaddr = (uintptr_t)(&(txbuffer[ndx * GMAC_TX_UNITSIZE]));

      /* Set the buffer address and mark the descriptor as in used by
       * firmware.
       */

      physaddr           = sam_physramaddr(bufaddr);
      txdesc[ndx].addr   = physaddr;
      txdesc[ndx].status = (uint32_t)GMACTXD_STA_USED;
    }

  /* Mark the final descriptor in the list */

  txdesc[CONFIG_SAMA5_GMAC_NTXBUFFERS - 1].status = GMACTXD_STA_USED | GMACTXD_STA_WRAP;

  /* Flush the entire TX descriptor table to RAM */

  cp15_clean_dcache((uintptr_t)txdesc,
                    (uintptr_t)txdesc +
                    CONFIG_SAMA5_GMAC_NTXBUFFERS * sizeof(struct gmac_txdesc_s));

  /* Set the Transmit Buffer Queue Base Register */

  physaddr = sam_physramaddr((uintptr_t)txdesc);
  sam_putreg(priv, SAM_GMAC_TBQB, physaddr);
}

/****************************************************************************
 * Function: sam_rxreset
 *
 * Description:
 *  Reset the receive logic
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_rxreset(struct sam_gmac_s *priv)
{
  struct gmac_rxdesc_s *rxdesc = priv->rxdesc;
  uint8_t *rxbuffer = priv->rxbuffer;
  uintptr_t bufaddr;
  uint32_t physaddr;
  uint32_t regval;
  int ndx;

  /* Disable RX */

  regval  = sam_getreg(priv, SAM_GMAC_NCR);
  regval &= ~GMAC_NCR_RXEN;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  /* Configure the RX descriptors. */

  priv->rxndx = 0;
  for (ndx = 0; ndx < CONFIG_SAMA5_GMAC_NRXBUFFERS; ndx++)
  {
    bufaddr = (uintptr_t)(&(rxbuffer[ndx * GMAC_RX_UNITSIZE]));
    DEBUGASSERT((bufaddr & ~GMACRXD_ADDR_MASK) == 0);

    /* Set the buffer address and remove GMACRXD_ADDR_OWNER and
     * GMACRXD_ADDR_WRAP.
     */

    physaddr           = sam_physramaddr(bufaddr);
    rxdesc[ndx].addr   = physaddr;
    rxdesc[ndx].status = 0;
  }

  /* Mark the final descriptor in the list */

  rxdesc[CONFIG_SAMA5_GMAC_NRXBUFFERS - 1].addr |= GMACRXD_ADDR_WRAP;

  /* Flush the entire RX descriptor table to RAM */

  cp15_clean_dcache((uintptr_t)rxdesc,
                    (uintptr_t)rxdesc +
                    CONFIG_SAMA5_GMAC_NRXBUFFERS * sizeof(struct gmac_rxdesc_s));

  /* Set the Receive Buffer Queue Base Register */

  physaddr = sam_physramaddr((uintptr_t)rxdesc);
  sam_putreg(priv, SAM_GMAC_RBQB, physaddr);
}

/****************************************************************************
 * Function: sam_gmac_reset
 *
 * Description:
 *  Reset the GMAC block.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_gmac_reset(struct sam_gmac_s *priv)
{
  uint32_t regval;

  /* Disable all GMAC interrupts */

  sam_putreg(priv, SAM_GMAC_IDR, GMAC_INT_ALL);

  /* Reset RX and TX logic */

  sam_rxreset(priv);
  sam_txreset(priv);

  /* Disable RX, TX, and statistics */

  regval = GMAC_NCR_TXEN | GMAC_NCR_RXEN | GMAC_NCR_WESTAT | GMAC_NCR_CLRSTAT;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  /* Disable clocking to the GMAC peripheral */

  sam_gmac_disableclk();
}

/****************************************************************************
 * Function: sam_macaddress
 *
 * Description:
 *   Configure the selected MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_macaddress(struct sam_gmac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;
  uint32_t regval;

  nllvdbg("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          dev->d_ifname,
          dev->d_mac.ether_addr_octet[0], dev->d_mac.ether_addr_octet[1],
          dev->d_mac.ether_addr_octet[2], dev->d_mac.ether_addr_octet[3],
          dev->d_mac.ether_addr_octet[4], dev->d_mac.ether_addr_octet[5]);

  /* Set the MAC address */

  regval = (uint32_t)dev->d_mac.ether_addr_octet[0] |
           (uint32_t)dev->d_mac.ether_addr_octet[1] << 8 |
           (uint32_t)dev->d_mac.ether_addr_octet[2] << 16 |
           (uint32_t)dev->d_mac.ether_addr_octet[3] << 24;
  sam_putreg(priv, SAM_GMAC_SAB1, regval);

  regval = (uint32_t)dev->d_mac.ether_addr_octet[4] |
           (uint32_t)dev->d_mac.ether_addr_octet[5] << 8;
  sam_putreg(priv, SAM_GMAC_SAT1, regval);
}

/****************************************************************************
 * Function: sam_gmac_configure
 *
 * Description:
 *  Configure the GMAC interface for normal operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_gmac_configure(struct sam_gmac_s *priv)
{
  uint32_t regval;

  nllvdbg("Entry\n");

  /* Enable clocking to the GMAC peripheral */

  sam_gmac_enableclk();

  /* Disable TX, RX, interrupts, etc. */

  sam_putreg(priv, SAM_GMAC_NCR, 0);
  sam_putreg(priv, SAM_GMAC_IDR, GMAC_INT_ALL);

  regval = sam_getreg(priv, SAM_GMAC_NCR);
  regval |= GMAC_NCR_CLRSTAT;
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  /* Clear all status bits in the receive status register. */

  regval = (GMAC_RSR_RXOVR | GMAC_RSR_REC | GMAC_RSR_BNA | GMAC_RSR_HNO);
  sam_putreg(priv, SAM_GMAC_RSR, regval);

  /* Clear all status bits in the transmit status register */

  regval = GMAC_TSR_UBR | GMAC_TSR_COL | GMAC_TSR_RLE | GMAC_TSR_TXGO |
           GMAC_TSR_TFC | GMAC_TSR_TXCOMP | GMAC_TSR_UND | GMAC_TSR_HRESP |
           GMAC_TSR_LCO;
  sam_putreg(priv, SAM_GMAC_TSR, regval);

  /* Clear any pending interrupts */

  (void)sam_getreg(priv, SAM_GMAC_ISR);

  /* Initial configuration:
   *
   *   SPD = 0    : Assuming 1000Base-T full duplex
   *   FD  = 1    : Assuming 1000Base-T full duplex
   *   DNVLAN = 0 : Don't discard non-VLAN frames
   *   JFRAME = 0 : Disable jumbo frames
   *   CAF        : Depends on CONFIG_NET_PROMISCUOUS
   *   NBC        : Depends on CONFIG_SAMA5_GMAC_NBC
   *   MTIHEN = 0 : Multicast hash disabled
   *   UNIHEN = 0 : Unicast hash disabled
   *   MAXFS = 0  : Disable receive 1536 byte frames
   *   GBE = 1    : Assuming 1000Base-T full duplex
   *   RTY = 0    : Disable retry test
   *   PEN = 1    : Pause frames disabled
   *   RXBUFO = 0 : No receive buffer offset
   *   LFERD = 0  : No length field error discard
   *   RFCS = 1   : Remove FCS
   *   CLK = 4    : Assuming MCK <= 160MHz
   *   DBW = 1    : 64-bit data bus with
   *   DCPF = 0   : Copy of pause frames not disabled
   *   RXCOEN = 0 : RX checksum offload disabled
   *   EFRHD = 0  : Disable frames received in half duple
   *   IRXFCS = 0 : Disable ignore RX FCX
   *   IPGSEN = 0 : IP stretch disabled
   *   RXBP = 0   : Receive bad pre-ambled disabled
   *   IRXER = 0  : Disable ignore IPG GXER
   */

  regval = GMAC_NCFGR_FD | GMAC_NCFGR_GBE | GMAC_NCFGR_PEN |
           GMAC_NCFGR_RFCS | GMAC_NCFGR_CLK_DIV64 | GMAC_NCFGR_DBW_64;

#ifdef CONFIG_NET_PROMISCUOUS
  regval |= GMAC_NCFGR_CAF;
#endif

#ifdef CONFIG_SAMA5_GMAC_NBC
  regval |= GMAC_NCFGR_NBC;
#endif

  sam_putreg(priv, SAM_GMAC_NCFGR, regval);

  /* Reset TX and RX */

  sam_rxreset(priv);
  sam_txreset(priv);

  /* Enable Rx and Tx, plus the stats register. */

  regval  = sam_getreg(priv, SAM_GMAC_NCR);
  regval |= (GMAC_NCR_RXEN | GMAC_NCR_TXEN | GMAC_NCR_WESTAT);
  sam_putreg(priv, SAM_GMAC_NCR, regval);

  /* Setup the interrupts for TX events, RX events, and error events */

  regval = GMAC_INT_MFS | GMAC_INT_RCOMP | GMAC_INT_RXUBR | GMAC_INT_TXUBR |
           GMAC_INT_TUR | GMAC_INT_RLEX | GMAC_INT_TFC | GMAC_INT_TCOMP |
           GMAC_INT_ROVR | GMAC_INT_HRESP | GMAC_INT_PFNZ | GMAC_INT_PTZ |
           GMAC_INT_PFTR | GMAC_INT_EXINT | GMAC_INT_DRQFR | GMAC_INT_SFR |
           GMAC_INT_DRQFT | GMAC_INT_SFT | GMAC_INT_PDRQFR | GMAC_INT_PDRSFR |
           GMAC_INT_PDRQFT | GMAC_INT_PDRSFT;
  sam_putreg(priv, SAM_GMAC_IER, regval);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sam_gmac_initialize
 *
 * Description:
 *   Initialize the GMAC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

int sam_gmac_initialize(void)
{
  struct sam_gmac_s *priv = &g_gmac;
  int ret;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct sam_gmac_s));
  priv->dev.d_ifup    = sam_ifup;       /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = sam_ifdown;     /* I/F down callback */
  priv->dev.d_txavail = sam_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = sam_addmac;     /* Add multicast MAC address */
  priv->dev.d_rmmac   = sam_rmmac;      /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (void*)&g_gmac; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll = wd_create();
  if (!priv->txpoll)
    {
      nlldbg("ERROR: Failed to create periodic poll timer\n");
      ret = -EAGAIN;
      goto errout;
    }

  priv->txtimeout = wd_create();     /* Create TX timeout timer */
  if (!priv->txpoll)
    {
      nlldbg("ERROR: Failed to create periodic poll timer\n");
      ret = -EAGAIN;
      goto errout_with_txpoll;
    }

  /* Configure PIO pins to support GMAC */

  sam_ethgpioconfig(priv);

  /* Allocate buffers */

  ret = sam_buffer_initialize(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_buffer_initialize failed: %d\n", ret);
      goto errout_with_txtimeout;
    }

  /* Attach the IRQ to the driver.  It will not be enabled at the AIC until
   * the interface is in the 'up' state.
   */

  ret = irq_attach(SAM_IRQ_GMAC, sam_gmac_interrupt);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to attach the handler to the IRQ%d\n", SAM_IRQ_GMAC);
      goto errout_with_buffers;
    }

  /* Enable clocking to the GMAC peripheral (just for sam_ifdown()) */

  sam_gmac_enableclk();

  /* Put the interface in the down state (disabling clocking again). */

  ret = sam_ifdown(&priv->dev);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to put the interface in the down state: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev);
  if (ret >= 0)
    {
      return ret;
    }

  nlldbg("ERROR: netdev_register() failed: %d\n", ret);

errout_with_buffers:
  sam_buffer_free(priv);
errout_with_txtimeout:
  wd_delete(priv->txtimeout);
errout_with_txpoll:
  wd_delete(priv->txpoll);
errout:
  return ret;
}

#endif /* CONFIG_NET && CONFIG_SAMA5_GMAC */
