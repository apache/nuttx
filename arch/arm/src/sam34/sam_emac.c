/****************************************************************************
 * arch/arm/src/sam34/sam_emac.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic derives from the SAM34D3 Ethernet driver.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Which used Atmel NoOS sample code for reference (only).  That Atmel
 * sample code has a BSD compatible license that requires this copyright
 * notice:
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
#include <nuttx/net/mii.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "chip/sam_pinmap.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_cmcc.h"
#include "sam_emac.h"

#include <arch/board/board.h>

#if defined(CONFIG_NET) && defined(CONFIG_SAM34_EMAC)

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Number of buffer for RX */

#ifndef CONFIG_SAM34_EMAC_NRXBUFFERS
#  define CONFIG_SAM34_EMAC_NRXBUFFERS  16
#endif

/* Number of buffer for TX */

#ifndef CONFIG_SAM34_EMAC_NTXBUFFERS
#  define CONFIG_SAM34_EMAC_NTXBUFFERS  8
#endif

#undef CONFIG_SAM34_EMAC_NBC

#ifndef CONFIG_SAM34_EMAC_PHYADDR
#  error "CONFIG_SAM34_EMAC_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_SAM34_EMAC_MII) && !defined(CONFIG_SAM34_EMAC_RMII)
#  warning "Neither CONFIG_SAM34_EMAC_MII nor CONFIG_SAM34_EMAC_RMII defined"
#endif

#if defined(CONFIG_SAM34_EMAC_MII) && defined(CONFIG_SAM34_EMAC_RMII)
#  error "Both CONFIG_SAM34_EMAC_MII and CONFIG_SAM34_EMAC_RMII defined"
#endif

#ifdef CONFIG_SAM34_EMAC_AUTONEG
#  ifndef CONFIG_SAM34_EMAC_PHYSR
#    error "CONFIG_SAM34_EMAC_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_SAM34_EMAC_PHYSR_ALTCONFIG
#    ifndef CONFIG_SAM34_EMAC_PHYSR_ALTMODE
#      error "CONFIG_SAM34_EMAC_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_10HD
#      error "CONFIG_SAM34_EMAC_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_100HD
#      error "CONFIG_SAM34_EMAC_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_10FD
#      error "CONFIG_SAM34_EMAC_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_100FD
#      error "CONFIG_SAM34_EMAC_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_SAM34_EMAC_PHYSR_SPEED
#      error "CONFIG_SAM34_EMAC_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_100MBPS
#      error "CONFIG_SAM34_EMAC_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_MODE
#      error "CONFIG_SAM34_EMAC_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAM34_EMAC_PHYSR_FULLDUPLEX
#      error "CONFIG_SAM34_EMAC_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

/* PHY definitions */

#if defined(CONFIG_ETH0_PHY_DM9161)
#  define MII_OUI_MSB    0x0181
#  define MII_OUI_LSB    0x2e
#elif defined(CONFIG_ETH0_PHY_LAN8700)
#  define MII_OUI_MSB    0x0007
#  define MII_OUI_LSB    0x30
#elif defined(CONFIG_ETH0_PHY_KSZ8051)
#  define MII_OUI_MSB    0x0022
#  define MII_OUI_LSB    0x05
#else
#  error EMAC PHY unrecognized
#endif

#ifdef CONFIG_SAM34_EMAC_PHYSR_ALTCONFIG

#  define PHYSR_MODE(sr)     ((sr) & CONFIG_SAM34_EMAC_PHYSR_ALTMODE)
#  define PHYSR_ISMODE(sr,m) (PHYSR_MODE(sr) == (m))

#  define PHYSR_IS10HDX(sr)  PHYSR_ISMODE(sr,CONFIG_SAM34_EMAC_PHYSR_10HD)
#  define PHYSR_IS100HDX(sr) PHYSR_ISMODE(sr,CONFIG_SAM34_EMAC_PHYSR_100HD)
#  define PHYSR_IS10FDX(sr)  PHYSR_ISMODE(sr,CONFIG_SAM34_EMAC_PHYSR_10FD)
#  define PHYSR_IS100FDX(sr) PHYSR_ISMODE(sr,CONFIG_SAM34_EMAC_PHYSR_100FD)

#else

#  define PHYSR_MODESPEED    (CONFIG_PHYSR_MODE | CONFIG_PHYSR_SPEED)
#  define PHYSR_10HDX        (0)
#  define PHYSR_100HDX       (CONFIG_PHYSR_100MBPS)
#  define PHYSR_10FDX        (CONFIG_PHYSR_FULLDUPLEX)
#  define PHYSR_100FDX       (CONFIG_PHYSR_FULLDUPLEX | CONFIG_PHYSR_100MBPS)

#  define PHYSR_IS10HDX(sr)  (((sr) & PHYSR_MODESPEED) == PHYSR_10HDX)
#  define PHYSR_IS100HDX(sr) (((sr) & PHYSR_MODESPEED) == PHYSR_100HDX)
#  define PHYSR_IS10FDX(sr)  (((sr) & PHYSR_MODESPEED) == PHYSR_10FDX)
#  define PHYSR_IS100FDX(sr) (((sr) & PHYSR_MODESPEED) == PHYSR_100FDX)

#endif

/* EMAC buffer sizes, number of buffers, and number of descriptors.
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

#define EMAC_RX_UNITSIZE 128                 /* Fixed size for RX buffer  */
#define EMAC_TX_UNITSIZE CONFIG_NET_BUFSIZE  /* MAX size for Ethernet packet */

/* We need at least one more free buffer than transmit buffers */

#define SAM_EMAC_NFREEBUFFERS (CONFIG_SAM34_EMAC_NTXBUFFERS+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAM34_EMAC_REGDEBUG
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

#define PHY_RETRY_MAX    1000000

/* Helpers ******************************************************************/
/* This is a helper pointer for accessing the contents of the EMAC
 * header
 */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The sam_emac_s encapsulates all state information for the EMAC peripheral */

struct sam_emac_s
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
  struct emac_rxdesc_s *rxdesc;      /* Allocated RX descriptors */
  struct emac_txdesc_s *txdesc;      /* Allocated TX descriptors */

  /* Debug stuff */

#ifdef CONFIG_SAM34_EMAC_REGDEBUG
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

static struct sam_emac_s g_emac;

#ifdef CONFIG_SAM34_EMAC_PREALLOCATE
/* Preallocated data */
/* TX descriptors list */

static struct emac_txdesc_s g_txdesc[CONFIG_SAM34_EMAC_NTXBUFFERS]
              __attribute__((aligned(8)));

/* RX descriptors list */

static struct emac_rxdesc_s g_rxdesc[CONFIG_SAM34_EMAC_NRXBUFFERS]
              __attribute__((aligned(8)));

/* Transmit Buffers
 *
 * Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
 * Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
 * shall be set to 0
 */

static uint8_t g_txbuffer[CONFIG_SAM34_EMAC_NTXBUFFERS * EMAC_TX_UNITSIZE];
               __attribute__((aligned(8)))

/* Receive Buffers */

static uint8_t g_rxbuffer[CONFIG_SAM34_EMAC_NRXBUFFERS * EMAC_RX_UNITSIZE]
               __attribute__((aligned(8)));

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAM34_EMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool sam_checkreg(struct sam_emac_s *priv, bool wr,
                         uint32_t regval, uintptr_t address);
static uint32_t sam_getreg(struct sam_emac_s *priv, uintptr_t addr);
static void sam_putreg(struct sam_emac_s *priv, uintptr_t addr, uint32_t val);
#else
# define sam_getreg(priv,addr)      getreg32(addr)
# define sam_putreg(priv,addr,val)  putreg32(val,addr)
#endif

/* Buffer management */

static uint16_t sam_txinuse(struct sam_emac_s *priv);
static uint16_t sam_txfree(struct sam_emac_s *priv);
static int  sam_buffer_initialize(struct sam_emac_s *priv);
static void sam_buffer_free(struct sam_emac_s *priv);

/* Common TX logic */

static int  sam_transmit(struct sam_emac_s *priv);
static int  sam_uiptxpoll(struct uip_driver_s *dev);
static void sam_dopoll(struct sam_emac_s *priv);

/* Interrupt handling */

static int  sam_recvframe(struct sam_emac_s *priv);
static void sam_receive(struct sam_emac_s *priv);
static void sam_txdone(struct sam_emac_s *priv);
static int  sam_emac_interrupt(int irq, void *context);

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
static void sam_phydump(struct sam_emac_s *priv);
#else
#  define sam_phydump(priv)
#endif

static int  sam_phywait(struct sam_emac_s *priv);
static int  sam_phyreset(struct sam_emac_s *priv);
static int  sam_phyfind(struct sam_emac_s *priv, uint8_t *phyaddr);
static int  sam_phyread(struct sam_emac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t *phyval);
static int  sam_phywrite(struct sam_emac_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t phyval);
static int  sam_autonegotiate(struct sam_emac_s *priv);
static bool sam_linkup(struct sam_emac_s *priv);
static int  sam_phyinit(struct sam_emac_s *priv);

/* EMAC Initialization */

static void sam_txreset(struct sam_emac_s *priv);
static void sam_rxreset(struct sam_emac_s *priv);
static void sam_emac_reset(struct sam_emac_s *priv);
static void sam_macaddress(struct sam_emac_s *priv);
static int  sam_emac_configure(struct sam_emac_s *priv);

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

#ifdef CONFIG_SAM34_EMAC_REGDEBUG
static bool sam_checkreg(struct sam_emac_s *priv, bool wr, uint32_t regval,
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

#ifdef CONFIG_SAM34_EMAC_REGDEBUG
static uint32_t sam_getreg(struct sam_emac_s *priv, uintptr_t address)
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

#ifdef CONFIG_SAM34_EMAC_REGDEBUG
static void sam_putreg(struct sam_emac_s *priv, uintptr_t address,
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
 *   priv - The EMAC driver state
 *
 * Returned Value:
 *   The number of TX buffers in-use
 *
 ****************************************************************************/

static uint16_t sam_txinuse(struct sam_emac_s *priv)
{
  uint32_t txhead32 = (uint32_t)priv->txhead;
  if ((uint32_t)priv->txtail > txhead32)
    {
      return txhead32 += CONFIG_SAM34_EMAC_NTXBUFFERS;
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
 *   priv - The EMAC driver state
 *
 * Returned Value:
 *   The number of TX buffers available
 *
 ****************************************************************************/

static uint16_t sam_txfree(struct sam_emac_s *priv)
{
  /* The number available is equal to the total number of buffers, minus the
   * number of buffers in use.  Notice that that actual number of buffers is
   * the configured size minus 1.
   */

  return (CONFIG_SAM34_EMAC_NTXBUFFERS-1) - sam_txinuse(priv);
}

/****************************************************************************
 * Function: sam_buffer_initialize
 *
 * Description:
 *   Allocate aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function degenerates to a few assignements.
 *
 * Input Parameters:
 *   priv - The EMAC driver state
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

static int sam_buffer_initialize(struct sam_emac_s *priv)
{
#ifdef CONFIG_SAM34_EMAC_PREALLOCATE
  /* Use pre-allocated buffers */

  priv->txdesc   = g_txdesc;
  priv->rxdesc   = g_rxdesc;
  priv->txbuffer = g_txbuffer;
  priv->rxbuffer = g_rxbuffer;

#else
  size_t allocsize;

  /* Allocate buffers */

  allocsize = CONFIG_SAM34_EMAC_NTXBUFFERS * sizeof(struct emac_txdesc_s);
  priv->txdesc = (struct emac_txdesc_s *)kmemalign(8, allocsize);
  if (!priv->txdesc)
    {
      nlldbg("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  memset(priv->txdesc, 0, allocsize);

  allocsize = CONFIG_SAM34_EMAC_NRXBUFFERS * sizeof(struct emac_rxdesc_s);
  priv->rxdesc = (struct emac_rxdesc_s *)kmemalign(8, allocsize);
  if (!priv->rxdesc)
    {
      nlldbg("ERROR: Failed to allocate RX descriptors\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  memset(priv->rxdesc, 0, allocsize);

  allocsize = CONFIG_SAM34_EMAC_NTXBUFFERS * EMAC_TX_UNITSIZE;
  priv->txbuffer = (uint8_t *)kmemalign(8, allocsize);
  if (!priv->txbuffer)
    {
      nlldbg("ERROR: Failed to allocate TX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  allocsize = CONFIG_SAM34_EMAC_NRXBUFFERS * EMAC_RX_UNITSIZE;
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
 *   priv - The EMAC driver state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_buffer_free(struct sam_emac_s *priv)
{
#ifndef CONFIG_SAM34_EMAC_PREALLOCATE
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
 *   Start hardware transmission.  Called either from the txdone interrupt
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

static int sam_transmit(struct sam_emac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;
  volatile struct emac_txdesc_s *txdesc;
  uint32_t regval;
  uint32_t status;

  nllvdbg("d_len: %d txhead: %d\n",  dev->d_len, priv->txhead);
  sam_dumppacket("Transmit packet", dev->d_buf, dev->d_len);

  /* Check parameter */

  if (dev->d_len > EMAC_TX_UNITSIZE)
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

  /* Setup/Copy data to transmission buffer */

  if (dev->d_len > 0)
    {
      /* Driver managed the ring buffer */

      memcpy((void *)txdesc->addr, dev->d_buf, dev->d_len);
    }

  /* Update TX descriptor status. */

  status = dev->d_len | EMACTXD_STA_LAST;
  if (priv->txhead == CONFIG_SAM34_EMAC_NTXBUFFERS-1)
    {
      status |= EMACTXD_STA_WRAP;
    }

  /* Update the descriptor status */

  txdesc->status = status;

  /* Increment the head index */

  if (++priv->txhead >= CONFIG_SAM34_EMAC_NTXBUFFERS)
    {
      priv->txhead = 0;
    }

  /* Now start transmission (if it is not already done) */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_TSTART;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

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
      sam_putreg(priv, SAM_EMAC_IDR, EMAC_INT_RCOMP);
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
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;

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

static void sam_dopoll(struct sam_emac_s *priv)
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

static int sam_recvframe(struct sam_emac_s *priv)
{
  struct emac_rxdesc_s *rxdesc;
  struct uip_driver_s *dev;
  const uint8_t *src;
  uint8_t  *dest;
  uint32_t rxndx;
  uint32_t pktlen;
  uint16_t copylen;
  bool isframe;

  /* Process received RX descriptor.  The ownership bit is set by the EMAC
   * once it has successfully written a frame to memory.
   */

  dev        = &priv->dev;
  dev->d_len = 0;

  rxndx      = priv->rxndx;
  rxdesc     = &priv->rxdesc[rxndx];
  isframe    = false;

  /* Invalidate the RX descriptor to force re-fetching from RAM */

  sam_cmcc_invalidate((uintptr_t)rxdesc,
                      (uintptr_t)rxdesc + sizeof(struct emac_rxdesc_s));

  nllvdbg("rxndx: %d\n", rxndx);

  while ((rxdesc->addr & EMACRXD_ADDR_OWNER) != 0)
    {
      /* The start of frame bit indicates the beginning of a frame.  Discard
       * any previous fragments.
       */

      if ((rxdesc->status & EMACRXD_STA_SOF) != 0)
        {
          /* Skip previous fragments */

          while (rxndx != priv->rxndx)
            {
              /* Give ownership back to the EMAC */

              rxdesc = &priv->rxdesc[priv->rxndx];
              rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

              /* Increment the RX index */

              if (++priv->rxndx >= CONFIG_SAM34_EMAC_NRXBUFFERS)
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

      if (++rxndx >= CONFIG_SAM34_EMAC_NRXBUFFERS)
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
                  /* Give ownership back to the EMAC */

                  rxdesc = &priv->rxdesc[priv->rxndx];
                  rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

                  /* Increment the RX index */

                  if (++priv->rxndx >= CONFIG_SAM34_EMAC_NRXBUFFERS)
                    {
                      priv->rxndx = 0;
                    }
                }
              while (rxndx != priv->rxndx);
              return -EIO;
            }

          /* Get the number of bytes to copy from the buffer */

          copylen = EMAC_RX_UNITSIZE;
          if ((pktlen + copylen) > CONFIG_NET_BUFSIZE)
            {
              copylen = CONFIG_NET_BUFSIZE - pktlen;
            }

          /* Get the data source.  Invalidate the source memory region to
           * force reload from RAM.
           */

          src = (const uint8_t *)(rxdesc->addr & EMACRXD_ADDR_MASK);
          sam_cmcc_invalidate((uintptr_t)src, (uintptr_t)src + copylen);

          /* And do the copy */

          memcpy(dest, src, copylen);
          dest   += copylen;
          pktlen += copylen;

          /* If the end of frame has been received, return the data */

          if ((rxdesc->status & EMACRXD_STA_EOF) != 0)
            {
              /* Frame size from the EMAC */

              dev->d_len = (rxdesc->status & EMACRXD_STA_FRLEN_MASK);
              nllvdbg("packet %d-%d (%d)\n", priv->rxndx, rxndx, dev->d_len);

              /* All data have been copied in the application frame buffer,
               * release the RX descriptor
               */

              while (priv->rxndx != rxndx)
                {
                  /* Give ownership back to the EMAC */

                  rxdesc = &priv->rxdesc[priv->rxndx];
                  rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);

                  /* Increment the RX index */

                  if (++priv->rxndx >= CONFIG_SAM34_EMAC_NRXBUFFERS)
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
          /* Give ownership back to the EMAC */

          rxdesc->addr &= ~(EMACRXD_ADDR_OWNER);
          priv->rxndx   = rxndx;
        }

    /* Process the next buffer */

    rxdesc = &priv->rxdesc[rxndx];

    /* Invalidate the RX descriptor to force re-fetching from RAM */

    sam_cmcc_invalidate((uintptr_t)rxdesc,
                        (uintptr_t)rxdesc + sizeof(struct emac_rxdesc_s));
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

static void sam_receive(struct sam_emac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;

  /* Loop while while sam_recvframe() successfully retrieves valid
   * EMAC frames.
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

static void sam_txdone(struct sam_emac_s *priv)
{
  struct emac_txdesc_s *txdesc;

  /* Are there any outstanding transmssions?  Loop until either (1) all of
   * the TX have been examined, or (2) until we encounter the first
   * descriptor that is still in use by the hardware.
   */

  while (priv->txhead != priv->txtail)
    {
      /* Yes.. check the next buffer at the tail of the list */

      txdesc = &priv->txdesc[priv->txtail];
      sam_cmcc_invalidate((uintptr_t)txdesc,
                          (uintptr_t)txdesc + sizeof(struct emac_txdesc_s));

      /* Is this TX descriptor still in use? */

      if ((txdesc->status & EMACTXD_STA_USED) == 0)
        {
          /* Yes.. the descriptor is still in use.  However, I have seen a
           * case (only repeatable on start-up) where the USED bit is never
           * set.  Yikes!  If we have encountered the first still busy
           * descriptor, then we should also have TQBD equal to the descriptor
           * address.  If it is not, then treat is as used anyway.
           */

#if 0 /* The issue does not exist in the current configuration, but may return */
#warning REVISIT
          if (priv->txtail == 0 &&
              sam_physramaddr((uintptr_t)txdesc) != sam_getreg(priv, SAM_EMAC_TBQB))
            {
              txdesc->status = (uint32_t)EMACTXD_STA_USED;
            }
          else
#endif
            {
              /* Otherwise, the descriptor is truly in use.  Break out of the
               * loop now.
               */

              break;
            }
        }

      /* Increment the tail index */

      if (++priv->txtail >= CONFIG_SAM34_EMAC_NTXBUFFERS)
        {
          /* Wrap to the beginning of the TX descriptor list */

          priv->txtail = 0;
        }

      /* At least one TX descriptor is available.  Re-enable RX interrupts.
       * RX interrupts may previously have been disabled when we ran out of
       * TX desciptors (see commits in sam_transmit()).
       */

      sam_putreg(priv, SAM_EMAC_IER, EMAC_INT_RCOMP);
    }

  /* Then poll uIP for new XMIT data */

  sam_dopoll(priv);
}

/****************************************************************************
 * Function: sam_emac_interrupt
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

static int sam_emac_interrupt(int irq, void *context)
{
  struct sam_emac_s *priv = &g_emac;
  uint32_t isr;
  uint32_t rsr;
  uint32_t tsr;
  uint32_t imr;
  uint32_t regval;
  uint32_t pending;
  uint32_t clrbits;

  isr = sam_getreg(priv, SAM_EMAC_ISR);
  rsr = sam_getreg(priv, SAM_EMAC_RSR);
  tsr = sam_getreg(priv, SAM_EMAC_TSR);
  imr = sam_getreg(priv, SAM_EMAC_IMR);

  pending = isr & ~(imr | EMAC_INT_UNUSED);
  nllvdbg("isr: %08x pending: %08x\n", isr, pending);

  /* Check for the completion of a transmission.  This should be done before
   * checking for received data (because receiving can cause another transmission
   * before we had a chance to handle the last one).
   *
   * ISR:TCOMP is set when a frame has been transmitted. Cleared on read.
   * TSR:TXCOMP is set when a frame has been transmitted. Cleared by writing a
   *   one to this bit.
   */

  if ((pending & EMAC_INT_TCOMP) != 0 || (tsr & EMAC_TSR_TXCOMP) != 0)
    {
      /* A frame has been transmitted */

      clrbits = EMAC_TSR_TXCOMP;

      /* Check for Retry Limit Exceeded (RLE) */

      if ((tsr & EMAC_TSR_RLE) != 0)
        {
          /* Status RLE & Number of discarded buffers */

          clrbits = EMAC_TSR_RLE | sam_txinuse(priv);
          sam_txreset(priv);

          nlldbg("ERROR: Retry Limit Exceeded TSR: %08x\n", tsr);

          regval = sam_getreg(priv, SAM_EMAC_NCR);
          regval |= EMAC_NCR_TXEN;
          sam_putreg(priv, SAM_EMAC_NCR, regval);
        }

      /* Check Collision Occurred (COL) */

      if ((tsr & EMAC_TSR_COL) != 0)
        {
          nlldbg("ERROR: Collision occurred TSR: %08x\n", tsr);
          clrbits |= EMAC_TSR_COL;
        }

      /* Check Transmit Frame Corruption due to AHB error (TFC) */

      if ((tsr & EMAC_TSR_TFC) != 0)
        {
          nlldbg("ERROR: Transmit Frame Corruption due to AHB error: %08x\n", tsr);
          clrbits |= EMAC_TSR_TFC;
        }

      /* Check for Transmit Underrun (UND)
       *
       * ISR:UND is set transmit DMA was not able to read data from memory,
       *   either because the bus was not granted in time, because a not
       *   OK hresp(bus error) was returned or because a used bit was read
       *   midway through frame transmission. If this occurs, the
       *   transmitter forces bad CRC. Cleared by writing a one to this bit.
       */

      if ((tsr & EMAC_TSR_UND) != 0)
        {
          nlldbg("ERROR: Transmit Underrun TSR: %08x\n", tsr);
          clrbits |= EMAC_TSR_UND;
        }

      /* Clear status */

      sam_putreg(priv, SAM_EMAC_TSR, clrbits);

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

  if ((pending & EMAC_INT_RCOMP) != 0 || (rsr & EMAC_RSR_REC) != 0)
    {
      clrbits = EMAC_RSR_REC;

      /* Check for Receive Overrun.
       *
       * RSR:RXOVR will be set if the RX FIFO is not able to store the
       *   receive frame due to a FIFO overflow, or if the receive status
       *   was not taken at the end of the frame. This bit is also set in
       *   DMA packet buffer mode if the packet buffer overflows. For DMA
       *   operation, the buffer will be recovered if an overrun occurs. This
       *   bit is cleared when set to 1.
       */

      if ((rsr & EMAC_RSR_RXOVR) != 0)
        {
          nlldbg("ERROR: Receiver overrun RSR: %08x\n", rsr);
          clrbits |= EMAC_RSR_RXOVR;
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

      if ((rsr & EMAC_RSR_BNA) != 0)
        {
          nlldbg("ERROR: Buffer not available RSR: %08x\n", rsr);
          clrbits |= EMAC_RSR_BNA;
        }

      /* Clear status */

      sam_putreg(priv, SAM_EMAC_RSR, clrbits);

      /* Handle the received packet */

       sam_receive(priv);
    }

#ifdef CONFIG_DEBUG_NET
  /* Check for PAUSE Frame recieved (PFRE).
   *
   * ISR:PFRE indicates that a pause frame has been received with non-zero
   * pause quantum.  Cleared on a read.
   */

  if ((pending & EMAC_INT_PFNX) != 0)
    {
      nlldbg("Pause frame received\n");
    }

  /* Check for Pause Time Zero (PTZ)
   *
   * ISR:PTZ is set Pause Time Zero
   */

  if ((pending & EMAC_INT_PTZ) != 0)
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
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;

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
  struct sam_emac_s *priv = (struct sam_emac_s *)arg;
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
 *   NuttX Callback: Bring up the EMAC interface when an IP address is
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
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
  int ret;

  nlldbg("Bringing up: %d.%d.%d.%d\n",
         dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
         (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Configure the EMAC interface for normal operation. */

  nllvdbg("Initialize the EMAC\n");
  sam_emac_configure(priv);

  /* Set the MAC address (should have been configured while we were down) */

  sam_macaddress(priv);

  /* Initialize for PHY access */

  ret = sam_phyinit(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phyinit failed: %d\n", ret);
      return ret;
    }

  /* Auto Negotiate, working in RMII mode */

  ret = sam_autonegotiate(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_autonegotiate failed: %d\n", ret);
      return ret;
    }

  while (sam_linkup(priv) == 0);
  nllvdbg("Link detected \n");

  /* Enable normal MAC operation */

  nllvdbg("Enable normal operation\n");

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, SAM_WDDELAY, sam_polltimer, 1, (uint32_t)priv);

  /* Enable the EMAC interrupt */

  priv->ifup = true;
  up_enable_irq(SAM_IRQ_EMAC);
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
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
  irqstate_t flags;

  nlldbg("Taking the network down\n");

  /* Disable the EMAC interrupt */

  flags = irqsave();
  up_disable_irq(SAM_IRQ_EMAC);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the sam_ifup() always
   * successfully brings the interface back up.
   */

  sam_emac_reset(priv);

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
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;
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
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;

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
  struct sam_emac_s *priv = (struct sam_emac_s *)dev->d_private;

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
static void sam_phydump(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t phyval;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

#ifdef CONFIG_SAM34_EMAC_RMII
  nllvdbg("RMII Registers (Address %02x)\n", priv->phyaddr);
#else /* defined(CONFIG_SAM34_EMAC_MII) */
  nllvdbg("MII Registers (Address %02x)\n", priv->phyaddr);
#endif

  sam_phyread(priv, priv->phyaddr, MII_MCR, &phyval);
  nllvdbg("  MCR:       %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, MII_MSR, &phyval);
  nllvdbg("  MSR:       %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, MII_ADVERTISE, &phyval);
  nllvdbg("  ADVERTISE: %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, MII_LPA, &phyval);
  nllvdbg("  LPR:       %04x\n", phyval);
  sam_phyread(priv, priv->phyaddr, CONFIG_SAM34_EMAC_PHYSR, &phyval);
  nllvdbg("  PHYSR:     %04x\n", phyval);

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);
}
#endif

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

static int sam_phywait(struct sam_emac_s *priv)
{
  volatile unsigned int retries;

  /* Loop for the configured number of attempts */

  for (retries = 0; retries < PHY_RETRY_MAX; retries++)
    {
      /* Is the PHY IDLE */

      if ((sam_getreg(priv, SAM_EMAC_NSR) & EMAC_NSR_IDLE) != 0)
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

static int sam_phyreset(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t mcr;
  int timeout;
  int ret;

  nllvdbg(" sam_phyreset\n");

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Reset the PHY */

  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywrite failed: %d\n", ret);
    }

  /* Wait for the PHY reset to complete */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < 10; timeout++)
    {
      mcr = MII_MCR_RESET;
      int result = sam_phyread(priv, priv->phyaddr, MII_MCR, &mcr);
      if (result < 0)
        {
          nlldbg("ERROR: Failed to read the MCR register: %d\n", ret);
          ret = result;
        }
      else if ((mcr & MII_MCR_RESET) == 0)
        {
          ret = OK;
          break;
        }
    }

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);
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

static int sam_phyfind(struct sam_emac_s *priv, uint8_t *phyaddr)
{
  uint32_t regval;
  uint16_t phyval;
  uint8_t candidate;
  unsigned int offset;
  int ret = -ESRCH;

  nllvdbg("Find a valid PHY address\n");

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  candidate = *phyaddr;

  /* Check current candidate address */

  ret = sam_phyread(priv, candidate, MII_PHYID1, &phyval);
  if (ret == OK && phyval == MII_OUI_MSB)
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

          ret = sam_phyread(priv, candidate, MII_PHYID1, &phyval);
          if (ret == OK && phyval == MII_OUI_MSB)
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
      sam_phyread(priv, candidate, CONFIG_SAM34_EMAC_PHYSR, &phyval);
      nllvdbg("  PHYSR:  %04x PHY addr: %d\n", phyval, candidate);
    }

  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);
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

static int sam_phyread(struct sam_emac_s *priv, uint8_t phyaddr,
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

  regval = EMAC_MAN_DATA(0)       | EMAC_MAN_WTN  | EMAC_MAN_REGA(regaddr) |
           EMAC_MAN_PHYA(phyaddr) | EMAC_MAN_READ | EMAC_MAN_WZO;

#ifndef CONFIG_SAM34_EMAC_CLAUSE45
  /* CLTTO must be set for Clause 22 operation. To read clause 45 PHYs, bit
   * 30 should be written with a 0 rather than a 1.
   */

  regval |= EMAC_MAN_CLTTO;
#endif

  sam_putreg(priv, SAM_EMAC_MAN, regval);

  /* Wait until the PHY is again idle */

  ret = sam_phywait(priv);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phywait failed: %d\n", ret);
      return ret;
    }

  /* Return data */

  *phyval = (uint16_t)(sam_getreg(priv, SAM_EMAC_MAN) & EMAC_MAN_DATA_MASK);
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

static int sam_phywrite(struct sam_emac_s *priv, uint8_t phyaddr,
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

  regval = EMAC_MAN_DATA(phyval)  | EMAC_MAN_WTN   | EMAC_MAN_REGA(regaddr) |
           EMAC_MAN_PHYA(phyaddr) | EMAC_MAN_WRITE | EMAC_MAN_WZO;

#ifndef CONFIG_SAM34_EMAC_CLAUSE45
  /* CLTTO must be set for Clause 22 operation. To read clause 45 PHYs, bit
   * 30 should be written with a 0 rather than a 1.
   */

  regval |= EMAC_MAN_CLTTO;
#endif

  sam_putreg(priv, SAM_EMAC_MAN, regval);

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

static int sam_autonegotiate(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t phyid1;
  uint16_t phyid2;
  uint16_t mcr;
  uint16_t msr;
  uint16_t advertise;
  uint16_t lpa;
  int timeout;
  int ret;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Verify tht we can read the PHYID register */

  ret = sam_phyread(priv, priv->phyaddr, MII_PHYID1, &phyid1);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read PHYID1\n");
      goto errout;
    }

  nllvdbg("PHYID1: %04x PHY address: %02x\n", phyid1, priv->phyaddr);

  ret = sam_phyread(priv, priv->phyaddr, MII_PHYID2, &phyid2);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read PHYID2\n");
      goto errout;
    }

  nllvdbg("PHYID2: %04x PHY address: %02x\n", phyid2, priv->phyaddr);

  if (phyid1 == MII_OUI_MSB &&
     ((phyid2 & MII_PHYID2_OUI_MASK) >> MII_PHYID2_OUI_SHIFT) == MII_OUI_LSB)
    {
      nllvdbg("  Vendor Model Number:   %04x\n",
             (phyid2 & MII_PHYID2_MODEL_MASK) >> MII_PHYID2_MODEL_SHIFT);
      nllvdbg("  Model Revision Number: %04x\n",
             (phyid2 & MII_PHYID2_REV_MASK) >> MII_PHYID2_REV_SHIFT);
    }
  else
    {
      nlldbg("ERROR: PHY not recognized\n");
    }

  /* Setup control register */

  ret = sam_phyread(priv, priv->phyaddr, MII_MCR, &mcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read MCR\n");
      goto errout;
    }

  mcr &= ~MII_MCR_ANENABLE;  /* Remove autonegotiation enable */
  mcr &= ~(MII_MCR_LOOPBACK | MII_MCR_PDOWN);
  mcr |= MII_MCR_ISOLATE;    /* Electrically isolate PHY */

  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, mcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write MCR\n");
      goto errout;
    }

  /* Set the Auto_negotiation Advertisement Register MII advertising for
   * Next page 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3
   */

  advertise = MII_ADVERTISE_100BASETXFULL | MII_ADVERTISE_100BASETXHALF |
              MII_ADVERTISE_10BASETXFULL | MII_ADVERTISE_10BASETXHALF |
              MII_ADVERTISE_8023;

  ret = sam_phywrite(priv, priv->phyaddr, MII_ADVERTISE, advertise);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write ANAR\n");
      goto errout;
    }

  /* Read and modify control register */

  ret = sam_phyread(priv, priv->phyaddr, MII_MCR, &mcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read MCR\n");
      goto errout;
    }

  mcr |= (MII_MCR_SPEED100 | MII_MCR_ANENABLE | MII_MCR_FULLDPLX);
  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, mcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write MCR\n");
      goto errout;
    }

  /* Restart Auto_negotiation */

  mcr |=  MII_MCR_ANRESTART;
  mcr &= ~MII_MCR_ISOLATE;

  ret = sam_phywrite(priv, priv->phyaddr, MII_MCR, mcr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to write MCR\n");
      goto errout;
    }

  nllvdbg("  MCR: %04x\n", mcr);

  /* Check AutoNegotiate complete */

  timeout = 0;
  for (;;)
    {
      ret = sam_phyread(priv, priv->phyaddr, MII_MSR, &msr);
      if (ret < 0)
        {
          nlldbg("ERROR: Failed to read MSR\n");
          goto errout;
        }

      /* Completed successfully? */

      if ((msr & MII_MSR_ANEGCOMPLETE) != 0)
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

  /* Get the AutoNeg Link partner base page */

  ret = sam_phyread(priv, priv->phyaddr, MII_LPA, &lpa);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read ANLPAR\n");
      goto errout;
    }

  /* Setup the EMAC link speed */

  regval  = sam_getreg(priv, SAM_EMAC_NCFGR);
  regval &= (EMAC_NCFGR_SPD | EMAC_NCFGR_FD);

  if (((advertise & lpa) & MII_ADVERTISE_100BASETXFULL) != 0)
    {
      /* Set MII for 100BaseTX and Full Duplex */

      regval |= (EMAC_NCFGR_SPD | EMAC_NCFGR_FD);
    }
  else if (((advertise & lpa) & MII_ADVERTISE_10BASETXFULL) != 0)
    {
      /* Set MII for 10BaseT and Full Duplex */

      regval |= EMAC_NCFGR_FD;
    }
  else if (((advertise & lpa) & MII_ADVERTISE_100BASETXHALF) != 0)
    {
      /* Set MII for 100BaseTX and half Duplex */

      regval |= EMAC_NCFGR_SPD;
    }
#if 0
  else if (((advertise & lpa) & MII_ADVERTISE_10BASETXHALF) != 0)
    {
      /* set MII for 10BaseT and half Duplex */
    }
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR, regval);

  /* Select RMII/MII */

  regval = sam_getreg(priv, SAM_EMAC_UR);
#ifdef CONFIG_SAM34_EMAC_RMII
  regval &= ~EMAC_UR_MII;
#else /* defined(CONFIG_SAM34_EMAC_MII) */
  regval |= EMAC_UR_MII;
#endif
  sam_putreg(priv, SAM_EMAC_UR, regval);

errout:
  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);
  return ret;
}

/****************************************************************************
 * Function: sam_linkup
 *
 * Description:
 *   Check if the link is up
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   true: The link is up
 *
 ****************************************************************************/

static bool sam_linkup(struct sam_emac_s *priv)
{
  uint32_t regval;
  uint16_t msr;
  uint16_t physr;
  bool linkup = false;
  int ret;

  /* Enable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  ret = sam_phyread(priv, priv->phyaddr, MII_MSR, &msr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read MSR: %d\n", ret);
      goto errout;
    }

  if ((msr & MII_MSR_LINKSTATUS) == 0)
    {
      nlldbg("ERROR: MSR LinkStatus: %04x\n", msr);
      goto errout;
    }

  /* Re-configure Link speed */

  ret = sam_phyread(priv, priv->phyaddr, CONFIG_SAM34_EMAC_PHYSR, &physr);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to read PHYSR: %d\n", ret);
      goto errout;
    }

  regval = sam_getreg(priv, SAM_EMAC_NCFGR);
  regval &= ~(EMAC_NCFGR_SPD | EMAC_NCFGR_FD);

  if ((msr & MII_MSR_100BASETXFULL) != 0 && PHYSR_IS100FDX(physr))
    {
      /* Set EMAC for 100BaseTX and Full Duplex */

      regval |= (EMAC_NCFGR_SPD | EMAC_NCFGR_FD);
    }
  else if ((msr & MII_MSR_10BASETXFULL) != 0  && PHYSR_IS10FDX(physr))
    {
      /* Set MII for 10BaseT and Full Duplex */

      regval |= EMAC_NCFGR_FD;
    }

  else if ((msr & MII_MSR_100BASETXHALF) != 0  && PHYSR_IS100HDX(physr))
    {
      /* Set MII for 100BaseTX and Half Duplex */

      regval |= EMAC_NCFGR_SPD;
    }

#if 0
  else if ((msr & MII_MSR_10BASETXHALF) != 0  && PHYSR_IS10HDX(physr))
    {
      /* Set MII for 10BaseT and Half Duplex */
    }
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR, regval);

  /* Start the EMAC transfers */

  nllvdbg("Link is up\n");
  linkup = true;

errout:
  /* Disable management port */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_MPE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  return linkup;
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

static int sam_phyinit(struct sam_emac_s *priv)
{
  uint32_t regval;
  int ret;

  /* Configure PHY clocking */

  regval = sam_getreg(priv, SAM_EMAC_NCFGR);
  regval &= ~EMAC_NCFGR_CLK_MASK;

#if BOARD_MCK_FREQUENCY > (160*1000*1000)
#  error Supported MCK frequency
#elif BOARD_MCK_FREQUENCY > (80*1000*1000)
  regval |= EMAC_NCFGR_CLK_DIV64; /* MCK divided by 64 (MCK up to 160 MHz) */
#elif BOARD_MCK_FREQUENCY > (40*1000*1000)
  regval |= EMAC_NCFGR_CLK_DIV32; /* MCK divided by 32 (MCK up to 80 MHz) */
#elif BOARD_MCK_FREQUENCY > (20*1000*1000)
  regval |= EMAC_NCFGR_CLK_DIV16; /* MCK divided by 16 (MCK up to 40 MHz) */
#else
  regval |= EMAC_NCFGR_CLK_DIV8;  /* MCK divided by 8 (MCK up to 20 MHz) */
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR, regval);

  /* Check the PHY Address */

  priv->phyaddr = CONFIG_SAM34_EMAC_PHYADDR;
  ret = sam_phyfind(priv, &priv->phyaddr);
  if (ret < 0)
    {
      nlldbg("ERROR: sam_phyfind failed: %d\n", ret);
      return ret;
    }

  if (priv->phyaddr != CONFIG_SAM34_EMAC_PHYADDR)
    {
      sam_phyreset(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: sam_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the EMAC MII interface.
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

static inline void sam_ethgpioconfig(struct sam_emac_s *priv)
{
  /* Configure PIO pins to support EMAC in MII mode*/

  sam_configgpio(GPIO_EMAC_TXCK);    /* Transmit Clock (or Reference Clock) */
  sam_configgpio(GPIO_EMAC_TXEN);    /* Transmit Enable */
  sam_configgpio(GPIO_EMAC_TX0);     /* Transmit data TXD0 */
  sam_configgpio(GPIO_EMAC_TX1);     /* Transmit data TXD1 */
  sam_configgpio(GPIO_EMAC_TX2);     /* Transmit data TXD2 */
  sam_configgpio(GPIO_EMAC_TX3);     /* Transmit data TXD3 */
//sam_configgpio(GPIO_EMAC_TXER);    /* Transmit Coding Error */
  sam_configgpio(GPIO_EMAC_RXCK);    /* Receive Clock */
  sam_configgpio(GPIO_EMAC_RXDV);    /* Receive Data Valid */
  sam_configgpio(GPIO_EMAC_RX0);     /* Receive data RXD0 */
  sam_configgpio(GPIO_EMAC_RX1);     /* Receive data RXD0 */
  sam_configgpio(GPIO_EMAC_RX2);     /* Receive data RXD0 */
  sam_configgpio(GPIO_EMAC_RX3);     /* Receive data RXD0 */
  sam_configgpio(GPIO_EMAC_RXER);    /* Receive Error */
  sam_configgpio(GPIO_EMAC_CRS);     /* Carrier Sense and Data Valid */
  sam_configgpio(GPIO_EMAC_COL);     /* Collision Detect */
  sam_configgpio(GPIO_EMAC_MDC);     /* Management Data Clock */
  sam_configgpio(GPIO_EMAC_MDIO);    /* Management Data Input/Output */
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

static void sam_txreset(struct sam_emac_s *priv)
{
  uint8_t *txbuffer = priv->txbuffer;
  struct emac_txdesc_s *txdesc = priv->txdesc;
  uintptr_t bufaddr;
  uint32_t regval;
  int ndx;

  /* Disable TX */

  regval = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_TXEN;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Configure the TX descriptors. */

  priv->txhead = 0;
  priv->txtail = 0;

  for (ndx = 0; ndx < CONFIG_SAM34_EMAC_NTXBUFFERS; ndx++)
  {
    bufaddr = (uint32_t)(&(txbuffer[ndx * EMAC_TX_UNITSIZE]));

    /* Set the buffer address and mark the descriptor as in used by firmware */

    txdesc[ndx].addr   = bufaddr;
    txdesc[ndx].status = EMACTXD_STA_USED;
  }

  /* Mark the final descriptor in the list */

  txdesc[CONFIG_SAM34_EMAC_NTXBUFFERS - 1].status =
    EMACTXD_STA_USED | EMACTXD_STA_WRAP;

  /* Set the Transmit Buffer Queue Pointer Register */

  sam_putreg(priv, SAM_EMAC_TBQB, (uintptr_t)txdesc);
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

static void sam_rxreset(struct sam_emac_s *priv)
{
  struct emac_rxdesc_s *rxdesc = priv->rxdesc;
  uint8_t *rxbuffer = priv->rxbuffer;
  uint32_t bufaddr;
  uint32_t regval;
  int ndx;

  /* Disable RX */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_RXEN;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Configure the RX descriptors. */

  priv->rxndx = 0;
  for (ndx = 0; ndx < CONFIG_SAM34_EMAC_NRXBUFFERS; ndx++)
  {
    bufaddr = (uintptr_t)(&(rxbuffer[ndx * EMAC_RX_UNITSIZE]));
    DEBUGASSERT((bufaddr & ~EMACRXD_ADDR_MASK) == 0);

    /* Set the buffer address and remove EMACRXD_ADDR_OWNER and
     * EMACRXD_ADDR_WRAP.
     */

    rxdesc[ndx].addr   = bufaddr;
    rxdesc[ndx].status = 0;
  }

  /* Mark the final descriptor in the list */

  rxdesc[CONFIG_SAM34_EMAC_NRXBUFFERS - 1].addr |= EMACRXD_ADDR_WRAP;

  /* Set the Receive Buffer Queue Pointer Register */

  sam_putreg(priv, SAM_EMAC_RBQB, (uintptr_t)rxdesc);
}

/****************************************************************************
 * Function: sam_emac_reset
 *
 * Description:
 *  Reset the EMAC block.
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

static void sam_emac_reset(struct sam_emac_s *priv)
{
  uint32_t regval;

  /* Disable all EMAC interrupts */

  sam_putreg(priv, SAM_EMAC_IDR, EMAC_INT_ALL);

  /* Reset RX and TX logic */

  sam_rxreset(priv);
  sam_txreset(priv);

  /* Disable RX, TX, and statistics */

  regval = EMAC_NCR_TXEN | EMAC_NCR_RXEN | EMAC_NCR_WESTAT | EMAC_NCR_CLRSTAT;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Disable clocking to the EMAC peripheral */

  sam_emac_disableclk();
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

static void sam_macaddress(struct sam_emac_s *priv)
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
  sam_putreg(priv, SAM_EMAC_SAB1, regval);

  regval = (uint32_t)dev->d_mac.ether_addr_octet[4] |
           (uint32_t)dev->d_mac.ether_addr_octet[5] << 8;
  sam_putreg(priv, SAM_EMAC_SAT1, regval);
}

/****************************************************************************
 * Function: sam_emac_configure
 *
 * Description:
 *  Configure the EMAC interface for normal operation.
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

static int sam_emac_configure(struct sam_emac_s *priv)
{
  uint32_t regval;

  nllvdbg("Entry\n");

  /* Enable clocking to the EMAC peripheral */

  sam_emac_enableclk();

  /* Disable TX, RX, interrupts, etc. */

  sam_putreg(priv, SAM_EMAC_NCR, 0);
  sam_putreg(priv, SAM_EMAC_IDR, EMAC_INT_ALL);

  regval = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_CLRSTAT;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Clear all status bits in the receive status register. */

  regval = (EMAC_RSR_RXOVR | EMAC_RSR_REC | EMAC_RSR_BNA);
  sam_putreg(priv, SAM_EMAC_RSR, regval);

  /* Clear all status bits in the transmit status register */

  regval = (EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLE | EMAC_TSR_TFC |
            EMAC_TSR_TXCOMP | EMAC_TSR_UND);
  sam_putreg(priv, SAM_EMAC_TSR, regval);

  /* Clear any pending interrupts */

  (void)sam_getreg(priv, SAM_EMAC_ISR);

  /* Enable/disable the copy of data into the buffers, ignore broadcasts.
   * Don't copy FCS.
   */

  regval  = sam_getreg(priv, SAM_EMAC_NCFGR);
  regval |= (EMAC_NCFGR_RFCS | EMAC_NCFGR_PEN);

#ifdef CONFIG_NET_PROMISCUOUS
  regval |=  EMAC_NCFGR_CAF;
#else
  regval &= ~EMAC_NCFGR_CAF;
#endif

#ifdef CONFIG_SAM34_EMAC_NBC
  regval |=  EMAC_NCFGR_NBC;
#else
  regval &= ~EMAC_NCFGR_NBC;
#endif

  sam_putreg(priv, SAM_EMAC_NCFGR, regval);

  /* Reset TX and RX */

  sam_rxreset(priv);
  sam_txreset(priv);

  /* Enable Rx and Tx, plus the stats register. */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= (EMAC_NCR_RXEN | EMAC_NCR_TXEN | EMAC_NCR_WESTAT);
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Setup the interrupts for TX events, RX events, and error events */

  regval = (EMAC_INT_RCOMP | EMAC_INT_RXUBR | EMAC_INT_TUR  | EMAC_INT_RLEX |
            EMAC_INT_TFC   | EMAC_INT_TCOMP | EMAC_INT_ROVR | EMAC_INT_HRESP |
            EMAC_INT_PFNX  | EMAC_INT_PTZ);
  sam_putreg(priv, SAM_EMAC_IER, regval);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_netinitialize
 *
 * Description:
 *   Initialize the EMAC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

void up_netinitialize(void)
{
  struct sam_emac_s *priv = &g_emac;
  int ret;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct sam_emac_s));
  priv->dev.d_ifup    = sam_ifup;       /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = sam_ifdown;     /* I/F down callback */
  priv->dev.d_txavail = sam_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = sam_addmac;     /* Add multicast MAC address */
  priv->dev.d_rmmac   = sam_rmmac;      /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (void*)&g_emac; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll = wd_create();
  if (!priv->txpoll)
    {
      nlldbg("ERROR: Failed to create periodic poll timer\n");
      return;
    }

  priv->txtimeout = wd_create();     /* Create TX timeout timer */
  if (!priv->txpoll)
    {
      nlldbg("ERROR: Failed to create periodic poll timer\n");
      goto errout_with_txpoll;
    }

  /* Configure PIO pins to support EMAC MII */

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

  ret = irq_attach(SAM_IRQ_EMAC, sam_emac_interrupt);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to attach the handler to the IRQ%d\n", SAM_IRQ_EMAC);
      goto errout_with_buffers;
    }

  /* Enable clocking to the EMAC peripheral (just for sam_ifdown()) */

  sam_emac_enableclk();

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
      return;
    }

  nlldbg("ERROR: netdev_register() failed: %d\n", ret);

errout_with_buffers:
  sam_buffer_free(priv);
errout_with_txtimeout:
  wd_delete(priv->txtimeout);
errout_with_txpoll:
  wd_delete(priv->txpoll);
}

#endif /* CONFIG_NET && CONFIG_SAM34_EMAC */
