/****************************************************************************
 * arch/arm/src/sama5/sam_emac.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_SAMA5_EMAC)

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
#include "cache.h"

#include "chip.h"
#include "chip/sam_pinmap.h"
#include "sam_pio.h"
#include "sam_periphclks.h"
#include "sam_memories.h"
#include "sam_ethernet.h"

#include <arch/board/board.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Number of buffer for RX */

#ifndef CONFIG_SAMA5_EMAC_NRXBUFFERS
#  define CONFIG_SAMA5_EMAC_NRXBUFFERS  16
#endif

/* Number of buffer for TX */

#ifndef CONFIG_SAMA5_EMAC_NTXBUFFERS
#  define CONFIG_SAMA5_EMAC_NTXBUFFERS  32
#endif

#undef CONFIG_SAMA5_EMAC_NBC

#ifndef CONFIG_SAMA5_EMAC_PHYADDR
#  error "CONFIG_SAMA5_EMAC_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_SAMA5_EMAC_MII) && !defined(CONFIG_SAMA5_EMAC_RMII)
#  warning "Neither CONFIG_SAMA5_EMAC_MII nor CONFIG_SAMA5_EMAC_RMII defined"
#endif

#if defined(CONFIG_SAMA5_EMAC_MII) && defined(CONFIG_SAMA5_EMAC_RMII)
#  error "Both CONFIG_SAMA5_EMAC_MII and CONFIG_SAMA5_EMAC_RMII defined"
#endif

#ifdef CONFIG_SAMA5_EMAC_AUTONEG
#  ifndef CONFIG_SAMA5_EMAC_PHYSR
#    error "CONFIG_SAMA5_EMAC_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_SAMA5_EMAC_PHYSR_ALTCONFIG
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_ALTMODE
#      error "CONFIG_SAMA5_EMAC_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_10HD
#      error "CONFIG_SAMA5_EMAC_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_100HD
#      error "CONFIG_SAMA5_EMAC_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_10FD
#      error "CONFIG_SAMA5_EMAC_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_100FD
#      error "CONFIG_SAMA5_EMAC_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_SPEED
#      error "CONFIG_SAMA5_EMAC_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_100MBPS
#      error "CONFIG_SAMA5_EMAC_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_MODE
#      error "CONFIG_SAMA5_EMAC_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_EMAC_PHYSR_FULLDUPLEX
#      error "CONFIG_SAMA5_EMAC_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

/* EMAC buffer sizes, number of buffers, and number of descriptors */

#ifdef CONFIG_NET_MULTIBUFFER
#  error CONFIG_NET_MULTIBUFFER must not be set
#endif

#define EMAC_RX_UNITSIZE            128     /* Fixed size for RX buffer  */
#define EMAC_TX_UNITSIZE            1518    /* Size for ETH frame length */

/* We need at least one more free buffer than transmit buffers */

#define SAM_EMAC_NFREEBUFFERS (CONFIG_SAMA5_EMAC_NTXBUFFERS+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAMA5_EMAC_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* Timing *******************************************************************/
/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define SAM_WDDELAY     (1*CLK_TCK)
#define SAM_POLLHSEC    (1*2)

/* TX timeout = 1 minute */

#define SAM_TXTIMEOUT   (60*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0004ffff)

/* Register values **********************************************************/

/* Interrupt bit sets *******************************************************/

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
  uint8_t               mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t               fduplex : 1; /* Full (vs. half) duplex */
  WDOG_ID               txpoll;      /* TX poll timer */
  WDOG_ID               txtimeout;   /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s   dev;         /* Interface understood by uIP */

  /* Used to track transmit and receive descriptors */

  uint8_t               phyaddr;     /* PHY address (pre-defined by pins on reset) */
  uint16_t              txhead;      /* Circular buffer head index */
  uint16_t              txtail;      /* Circualr buffer tail index */
  uint16_t              rxndx;       /* RX index for current processing RX descriptor */
  uint16_t              segments;    /* RX segment count */
  uint16_t              inflight;    /* Number of TX transfers "in_flight" */
  uint16_t              retries;     /* PHY retry count */

  uint8_t              *rxbuffer;    /* Allocated RX buffers */
  uint8_t              *txbuffer;    /* Allocated TX buffers */
  struct emac_rxdesc_s *rxdesc;      /* Allocated RX descriptors */
  struct emac_txdesc_s *txdesc;      /* Allocated TX descriptors */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_EMAC_REGDEBUG
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

#ifdef CONFIG_SAMA5_EMAC_PREALLOCATE
/* Preallocated data */
/* TX descriptors list */

static struct emac_txdesc_s g_txdesc[TX_BUFFERS] __attribute__((aligned(8)));

/* RX descriptors list */

static struct emac_rxdesc_s g_rxdesc[RX_BUFFERS]__attribute__((aligned(8)));

/* Transmit Buffers
 *
 * Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
 * Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
 * shall be set to 0
 */

static uint8_t g_txbuffer[CONFIG_SAMA5_EMAC_NTXBUFFERS * EMAC_TX_UNITSIZE];
               __attribute__((aligned(8)))

/* Receive Buffers */

static uint8_t g_rxbuffer[CONFIG_SAMA5_EMAC_NRXBUFFERS * EMAC_RX_UNITSIZE]
               __attribute__((aligned(8)));

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAMA5_EMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool sasm_checkreg(struct twi_dev_s *priv, bool wr,
                          uint32_t value, uintptr_t address);
static uint32_t sam_getreg(uintptr_t addr);
static void sam_putreg(struct sam_emac_s *priv, uintptr_t addr, uint32_t val);
#else
# define sam_getreg(priv,addr)      getreg32(addr)
# define sam_putreg(priv,addr,val)  putreg32(val,addr)
#endif

/* Common TX logic */

static int  sam_transmit(FAR struct sam_emac_s *priv);
static int  sam_uiptxpoll(struct uip_driver_s *dev);
static void sam_dopoll(FAR struct sam_emac_s *priv);

/* Interrupt handling */

static void sam_freesegment(FAR struct sam_emac_s *priv,
                            FAR struct emac_rxdesc_s *rxfirst, int segments);
static int  sam_recvframe(FAR struct sam_emac_s *priv);
static void sam_receive(FAR struct sam_emac_s *priv);
static void sam_freeframe(FAR struct sam_emac_s *priv);
static void sam_txdone(FAR struct sam_emac_s *priv);
static int  sam_emac_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void sam_polltimer(int argc, uint32_t arg, ...);
static void sam_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int  sam_ifup(struct uip_driver_s *dev);
static int  sam_ifdown(struct uip_driver_s *dev);
static int  sam_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int  sam_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int  sam_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/* PHY Initialization */

static int  sam_phyread(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t *value);
static int  sam_phywrite(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t value);
#ifdef CONFIG_PHY_DM9161
static inline int sam_dm9161(FAR struct sam_emac_s *priv);
#endif
static int  sam_autonegotiate(struct sam_emac_s *priv);
static bool sam_linkup(struct sam_emac_s *priv);
static int  sam_phyinit(FAR struct sam_emac_s *priv);

/* MAC/DMA Initialization */

static void sam_txreset(struct sam_emac_s *priv);
static void sam_rxreset(struct sam_emac_s *priv);
static void sam_emac_reset(FAR struct sam_emac_s *priv);
static void sam_macaddress(FAR struct sam_emac_s *priv);
static int  sam_emac_configure(FAR struct sam_emac_s *priv);
static int  sam_buffer_initialize(struct sam_emac_s *priv);
static void sam_buffer_free(struct sam_emac_s *priv);

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
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_EMAC_REGDEBUG
static bool sam_checkreg(struct twi_dev_s *priv, bool wr, uint32_t value,
                         uintptr_t address)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)  /* Same address? */
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
      priv->vallast  = value;
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

#ifdef CONFIG_SAMA5_EMAC_REGDEBUG
static uint32_t sam_getreg(struct twi_dev_s *priv, uintptr_t address)
{
  uint32_t value = getreg32(address);

  if (twi_checkreg(priv, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
    }

  return value;
}
#endif

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_EMAC_REGDEBUG
static void sam_putreg(struct twi_dev_s *priv, uintptr_t address,
                       uint32_t value)
{
  if (twi_checkreg(priv, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
    }

  putreg32(value, address);
}
#endif

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

static int sam_transmit(FAR struct sam_emac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  struct emac_txdesc_s *txfirst;
  uint8_t *buffer;
  int bufcount;
  int lastsize;
  int txndx;
  int i;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  txndx   = priv->txhead;
  txdesc  = &priv->txdesc[txndx];
  txfirst = txdesc;

  nllvdbg("d_len: %d d_buf: %p txhead: %d\n",
          priv->dev.d_len, priv->dev.d_buf, txndx);

  /* Now many buffers will be need to send the packet? */

  bufcount = (priv->dev.d_len + (CONFIG_NET_BUFSIZE-1)) / CONFIG_NET_BUFSIZE;
  lastsize = priv->dev.d_len - (bufcount - 1) * CONFIG_NET_BUFSIZE;

  nllvdbg("bufcount: %d lastsize: %d\n", bufcount, lastsize);

  /* Set the first segment bit in the first TX descriptor */
#warning Missing logic

  /* Set up all but the last TX descriptor */

  buffer = priv->dev.d_buf;

  for (i = 0; i < bufcount; i++)
    {
      /* Set the Buffer1 address pointer */
#warning Missing logic

      /* Set the buffer size in all TX descriptors */

      if (i == (bufcount-1))
        {
          /* This is the last segment.  Set the last segment bit in the
           * last TX descriptor and ask for an interrupt when this
           * segment transfer completes.
           */
#warning Missing logic

          /* This segement is, most likely, of fractional buffersize */
#warning Missing logic
          buffer        += lastsize;
        }
      else
        {
          /* This is not the last segment.  We don't want an interrupt
           * when this segment transfer completes.
           */
#warning Missing logic

          /* The size of the transfer is the whole buffer */
#warning Missing logic

        }

      /* Give the descriptor to DMA */
#warning Missing logic
    }

  /* Remember where we left off in the TX descriptor chain */

  priv->txhead = txndx;

  /* Detach the buffer from priv->dev structure.  That buffer is now
   * "in-flight".
   */

  priv->dev.d_len = 0;

  /* If there is no other TX buffer, in flight, then remember the location
   * of the TX descriptor.  This is the location to check for TX done events.
   */

  if (!priv->txtail)
    {
      DEBUGASSERT(priv->inflight == 0);
      priv->txtail = txndx;
    }

  /* Increment the number of TX transfer in-flight */

  priv->inflight++;

  nllvdbg("txhead: %d txtail: %d inflight: %d\n",
          priv->txhead, priv->txtail, priv->inflight);

  /* If all TX descriptors are in-flight, then we have to disable receive interrupts
   * too.  This is because receive events can trigger more un-stoppable transmit
   * events.
   */

  if (priv->inflight >= CONFIG_SAMA5_EMAC_NTXBUFFERS)
    {
#warning "Missing logic"
    }

  /* Check if the TX Buffer unavailable flag is set */
#warning "Missing logic"

  /* Enable TX interrupts */
#warning "Missing logic"

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, SAM_TXTIMEOUT, sam_txtimeout, 1, (uint32_t)priv);
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
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      uip_arp_out(&priv->dev);
      sam_transmit(priv);
      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

      /* Check if the next TX descriptor is owned by the EMAC DMA or CPU.  We
       * cannot perform the TX poll if we are unable to accept another packet for
       * transmission.
       */
#warning Missing logic

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
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the received frame.
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

static void sam_dopoll(FAR struct sam_emac_s *priv)
{
  FAR struct uip_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the EMAC DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   */

#warning Missing logic
    {
      /* If we have the descriptor, then poll uIP for new XMIT data. */

      (void)uip_poll(dev, sam_uiptxpoll);
    }
}

/****************************************************************************
 * Function: sam_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the received frame.
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

static void sam_freesegment(FAR struct sam_emac_s *priv,
                              FAR struct emac_rxdesc_s *rxfirst, int segments)
{
  struct emac_rxdesc_s *rxdesc;
  int i;

  nllvdbg("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set OWN bit in RX descriptors.  This gives the buffers back to DMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
#warning Missing logic
    }

  /* Reset the segment managment logic */

  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */
#warning "Missing logic"
}

/****************************************************************************
 * Function: sam_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
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
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static int sam_recvframe(FAR struct sam_emac_s *priv)
{
  struct emac_rxdesc_s *rxdesc;
  struct emac_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int rxndx;
  int i;

  nllvdbg("rxndx: %d segments: %d\n", priv->rxndx, priv->segments);

  /* Scan descriptors owned by the CPU.  Scan until:
   *
   *   1) We find a descriptor still owned by the DMA,
   *   2) We have examined all of the RX descriptors, or
   *   3) All of the TX descriptors are in flight.
   *
   * This last case is obscure.  It is due to that fact that each packet
   * that we receive can generate an unstoppable transmisson.  So we have
   * to stop receiving when we can not longer transmit.  In this case, the
   * transmit logic should also have disabled further RX interrupts.
   */

  rxndx  = priv->rxndx;
  rxdesc = &priv->rxdesc[rxndx];

#warning Missing logic
    {
      /* Check if this is the first segment in the frame */
#warning Missing logic

      /* Check if this is an intermediate segment in the frame */
#warning Missing logic
      if (true)
        {
          priv->segments++;
        }

      /* Otherwise, it is the last segment in the frame */
#warning Missing logic

      else
        {
          priv->segments++;

          /* Check if the there is only one segment in the frame */

          if (priv->segments == 1)
            {
              rxcurr = rxdesc;
            }
          else
            {
              rxcurr = &priv->rxdesc[priv->rxndx];
            }

          nllvdbg("rxndx: %d rxcurr: %p segments: %d\n",
              priv->rxndx, rxcurr, priv->segments);

          /* Check if any errors are reported in the frame */

#warning Missing logic
          if (true)
            {
              struct uip_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */
#warning Missing logic

              /* Return success, remebering where we should re-start scanning
               * and resetting the segment scanning logic
               */
#warning Missing logic

              nllvdbg("rxndx: %d d_buf: %p d_len: %d\n",
                      priv->rxndx, dev->d_buf, dev->d_len);

              return OK;
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */
#warning Missing logic
              sam_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */
#warning Missing logic
    }

  /* We get here after all of the descriptors have been scanned or when rxdesc points
   * to the first descriptor owned by the DMA.  Remember where we left off.
   */

  priv->rxndx = rxndx;

  nllvdbg("rxndx: %d segments: %d\n", priv->rxndx, priv->segments);

  return -EAGAIN;
}

/****************************************************************************
 * Function: sam_receive
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
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void sam_receive(FAR struct sam_emac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;

  /* Loop while while sam_recvframe() successfully retrieves valid
   * EMAC frames.
   */

  while (sam_recvframe(priv) == OK)
    {
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
 * Function: sam_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the buffers of completed TX transfers.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void sam_freeframe(FAR struct sam_emac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  int txtail;
  int i;

  nllvdbg("txhead: %d txtail: %d inflight: %d\n",
          priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txtail = priv->txtail;
  txdesc = &priv->txdesc[txtail];

  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

#warning Missing logic
        {
          /* Check if this is the first segment of a TX frame. */
#warning Missing logic

          /* In any event, make sure that xxx is nullified. */
#warning Missing logic

          /* Check if this is the last segement of a TX frame */

#warning Missing logic
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight, then RX interrupts
               * may have been disabled... we can re-enable them now.
               */
#warning "Missing logic"

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = 0;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */
#warning Missing logic
        }

      /* We get here if (1) there are still frames "in-flight". Remember
       * where we left off.
       */

      priv->txtail = txtail;

      nllvdbg("txhead: %d txtail: %d inflight: %d\n",
              priv->txhead, priv->txtail, priv->inflight);
    }
}

/****************************************************************************
 * Function: sam_txdone
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
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void sam_txdone(FAR struct sam_emac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX desciptor change, returning buffers to free list */

  sam_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      wd_cancel(priv->txtimeout);

      /* And disable further TX interrupts. */
#warning "Missing logic"
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

static int sam_emac_interrupt(int irq, FAR void *context)
{
  register FAR struct sam_emac_s *priv = &g_emac;
  uint32_t dmasr;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */
#warning "Missing logic"

  /* Mask only enabled interrupts.  This depends on the fact that the interrupt
   * related bits (0-16) correspond in these two registers.
   */
#warning "Missing logic"

  /* Check if there are pending "normal" interrupts */

#warning "Missing logic"
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * sam_receive()
       */
#warning "Missing logic"
        {
          /* Clear the pending receive interrupt */
#warning "Missing logic"

          /* Handle the received package */

          sam_receive(priv);
        }

      /* Check if a packet transmission just completed.  If so, call
       * sam_txdone(). This may disable further TX interrupts if there
       * are no pending tansmissions.
       */

#warning "Missing logic"
        {
          /* Clear the pending receive interrupt */
#warning "Missing logic"

          /* Check if there are pending transmissions */

          sam_txdone(priv);
        }

      /* Clear the pending normal summary interrupt */
#warning "Missing logic"
    }

  /* Handle error interrupt only if CONFIG_DEBUG_NET is eanbled */

#ifdef CONFIG_DEBUG_NET

  /* Check if there are pending error interrupts */

#warning "Missing logic"
    {
      /* Just let the user know what happened */

      nlldbg("Abormal event(s): %08x\n", dmasr);

      /* Clear all pending error interrupts */
#warning "Missing logic"
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
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)arg;

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
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)arg;
  FAR struct uip_driver_s   *dev  = &priv->dev;

  /* Check if the next TX descriptor is owned by the EMAC DMA or CPU.  We
   * cannot perform the timer poll if we are unable to accept another packet
   * for transmission.  Hmmm.. might be bug here.  Does this mean if there is
   * a transmit in progress, we will miss TCP time state updates?
   */
#warning Missing logic
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
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;
  int ret;

  ndbg("Bringing up: %d.%d.%d.%d\n",
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
      ndbg("ERROR: sam_phyinit failed: %d\n", ret);
      return ret;
    }

  /* Auto Negotiate, working in RMII mode */

  ret = sam_autonegotiate(priv);
  if (ret < 0)
    {
      ndbg("ERROR: sam_autonegotiate failed: %d\n", ret);
      return ret;
    }

  while (sam_linkup(priv) == 0);
  nvdbg("Link detected \n");

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
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;
  irqstate_t flags;

  ndbg("Taking the network down\n");

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
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;
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
static int sam_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;

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
static int sam_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;

  nllvdbg("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast routing table */
#error "Missing logic"

  return OK;
}
#endif

/****************************************************************************
 * Function: sam_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phyread(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMIIAR register, preserving CSR Clock Range CR[2:0] bits */
#warning Missing logic

  /* Set the PHY device address, PHY register address, and set the buy bit.
   * the  EMAC_MACMIIAR_MW is clear, indicating a read operation.
   */
#warning Missing logic

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
#warning Missing logic
    }

  ndbg("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
       phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: sam_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The 16-bit value to write to the PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phywrite(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMIIAR register, preserving CSR Clock Range CR[2:0] bits */
#warning Missing logic

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the  EMAC_MACMIIAR_MW is set, indicating a write operation.
   */
#warning Missing logic

  /* Write the value into the MACIIDR register before setting the new MACMIIAR
   * register value.
   */
#warning Missing logic

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
#warning Missing logic
    }

  ndbg("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x value: %04x\n",
       phydevaddr, phyregaddr, value);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: sam_dm9161
 *
 * Description:
 *   Special workaround for the Davicom DM9161 PHY is required.  On power,
 *   up, the PHY is not usually configured correctly but will work after
 *   a powered-up reset.  This is really a workaround for some more
 *   fundamental issue with the PHY clocking initialization, but the
 *   root cause has not been studied (nor will it be with this workaround).
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PHY_DM9161
static inline int sam_dm9161(FAR struct sam_emac_s *priv)
{
  uint16_t phyval;
  int ret;

  /* Read the PHYID1 register;  A failure to read the PHY ID is one
   * indication that check if the DM9161 PHY CHIP is not ready.
   */

  ret = sam_phyread(CONFIG_SAMA5_EMAC_PHYADDR, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read the PHY ID1: %d\n", ret);
      return ret;
    }

  /* If we failed to read the PHY ID1 register, the reset the MCU to recover */

  else if (phyval == 0xffff)
    {
      up_systemreset();
    }

  nvdbg("PHY ID1: 0x%04X\n", phyval);

  /* Now check the "DAVICOM Specified Configuration Register (DSCR)", Register 16 */

  ret = sam_phyread(CONFIG_SAMA5_EMAC_PHYADDR, 16, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read the PHY Register 0x10: %d\n", ret);
      return ret;
    }

  /* Bit 8 of the DSCR register is zero, then the DM9161 has not selected RMII.
   * If RMII is not selected, then reset the MCU to recover.
   */

  else if ((phyval & (1 << 8)) == 0)
    {
      up_systemreset();
    }

  return OK;
}
#endif

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
 * Assumptions:
 *
 ****************************************************************************/

static int sam_phyinit(FAR struct sam_emac_s *priv)
{
  volatile uint32_t timeout;
  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the SR field in the MACMIIAR register */
#warning Missing logic

  /* Put the PHY in reset mode */

  ret = sam_phywrite(CONFIG_SAMA5_EMAC_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      ndbg("Failed to reset the PHY: %d\n", ret);
      return ret;
    }
  up_mdelay(PHY_RESET_DELAY);

  /* Perform any necessary, board-specific PHY initialization */

#ifdef CONFIG_SAMA5_EMAC_PHYINIT
  ret = sam_phy_boardinitialize(EMAC_INTF);
  if (ret < 0)
    {
      ndbg("Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Special workaround for the Davicom DM9161 PHY is required. */

#ifdef CONFIG_PHY_DM9161
  ret = sam_dm9161(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Perform auto-negotion if so configured */

#ifdef CONFIG_SAMA5_EMAC_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = sam_phyread(CONFIG_SAMA5_EMAC_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          ndbg("Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      ndbg("Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

  /* Enable auto-gegotiation */

  ret = sam_phywrite(CONFIG_SAMA5_EMAC_PHYADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      ndbg("Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = sam_phyread(CONFIG_SAMA5_EMAC_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          ndbg("Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      ndbg("Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }

  /* Read the result of the auto-negotiation from the PHY-specific register */

  ret = sam_phyread(CONFIG_SAMA5_EMAC_PHYADDR, CONFIG_SAMA5_EMAC_PHYSR, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  nvdbg("PHYSR[%d]: %04x\n", CONFIG_SAMA5_EMAC_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.  IF
   * This CONFIG_SAMA5_EMAC_PHYSR_ALTCONFIG is selected, this indicates that the PHY
   * represents speed and mode information are combined, for example, with
   * separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_SAMA5_EMAC_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_SAMA5_EMAC_PHYSR_ALTMODE)
    {
      default:
      case CONFIG_SAMA5_EMAC_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_SAMA5_EMAC_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_SAMA5_EMAC_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_SAMA5_EMAC_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;
    }

  /* Different PHYs present speed and mode information in different ways.  Some
   * will present separate information for speed and mode (this is the default).
   * Those PHYs, for example, may provide a 10/100 Mbps indication and a separate
   * full/half duplex indication.
   */

#else
  if ((phyval & CONFIG_SAMA5_EMAC_PHYSR_MODE) == CONFIG_SAMA5_EMAC_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_SAMA5_EMAC_PHYSR_SPEED) == CONFIG_SAMA5_EMAC_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotion not selected */

  phyval = 0;
#ifdef CONFIG_SAMA5_EMAC_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_SAMA5_EMAC_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  ret = sam_phywrite(CONFIG_SAMA5_EMAC_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     ndbg("Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }
  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_SAMA5_EMAC_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_SAMA5_EMAC_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ndbg("Duplex: %s Speed: %d MBps\n",
       priv->fduplex ? "FULL" : "HALF",
       priv->mbps100 ? 100 : 10);

  return OK;
}

/****************************************************************************
 * Function: sam_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the EMAC interface.
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

static inline void sam_ethgpioconfig(FAR struct sam_emac_s *priv)
{
  /* Configure PIO pins to support EMAC */
  /* Configure EMAC PIO pins common to both MII and RMII */

  sam_configpio(PIO_EMAC_TX0);
  sam_configpio(PIO_EMAC_TX1);
  sam_configpio(PIO_EMAC_RX0);
  sam_configpio(PIO_EMAC_RX1);

  sam_configpio(PIO_EMAC_TXEN);
  sam_configpio(PIO_EMAC_CRSDV);
  sam_configpio(PIO_EMAC_RXER);
  sam_configpio(PIO_EMAC_REFCK);

  /* MDC and MDIO are common to both modes */

  sam_configpio(PIO_EMAC_MDC);
  sam_configpio(PIO_EMAC_MDIO);

#if defined(CONFIG_SAMA5_EMAC_MII)
  /* Provide clocking for the MII interface */
#warning Missing logic

# endif

  /* Set up the RMII interface. */

#elif defined(CONFIG_SAMA5_EMAC_RMII)
  /* Provide clocking for the RMII interface */
#warning Missing logic

#endif
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
  uint32_t physaddr;
  uint32_t regval;
  int ndx;

  /* Disable TX */

  regval = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_TE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Configure the TX descriptors. */

  priv->txhead = 0;
  priv->txtail = 0;

  for (ndx = 0; ndx < CONFIG_SAMA5_EMAC_NTXBUFFERS; ndx++)
  {
    bufaddr = (uint32_t)(&(txbuffer[ndx * EMAC_TX_UNITSIZE]));
    DEBUGASSERT((bufaddr & ~EMACTXD_ADDR_MASK) == 0);

    /* Set the buffer address and mark the descriptor as used */

    physaddr           = sam_physramaddr(bufaddr);
    txdesc[ndx].addr   = physaddr;
    txdesc[ndx].status = EMACTXD_CTRL_USED;
  }

  /* Mark the final descriptor in the list */

  txdesc[CONFIG_SAMA5_EMAC_NTXBUFFERS - 1].status =
    EMACTXD_CTRL_USED | EMACTXD_CTRL_WRAP;

  /* Set the Transmit Buffer Queue Pointer Register */

  physaddr = sam_physramaddr((uint32_t)txdesc);
  sam_putreg(priv, SAM_EMAC_TBQP, physaddr);
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
  uint32_t physaddr;
  uint32_t regval;
  int ndx;

  /* Disable RX */

  regval  = sam_getreg(priv, SAM_EMAC_NCR);
  regval &= ~EMAC_NCR_RE;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Configure the RX descriptors. */

  priv->rxndx = 0;
  for (ndx = 0; ndx < CONFIG_SAMA5_EMAC_NRXBUFFERS; ndx++)
  {
    bufaddr = (uintptr_t)(&(rxbuffer[ndx * EMAC_RX_UNITSIZE]));
    DEBUGASSERT((bufaddr & ~EMACRXD_ADDR_MASK) == 0);

    /* Set the buffer address and remove EMACRXD_ADDR_OWNER and
     * EMACRXD_ADDR_WRAP.
     */

    physaddr           = sam_physramaddr(bufaddr);
    rxdesc[ndx].addr   = physaddr;
    rxdesc[ndx].status = 0;
  }

  /* Mark the final descriptor in the list */

  rxdesc[CONFIG_SAMA5_EMAC_NRXBUFFERS - 1].addr |= EMACRXD_ADDR_WRAP;

  /* Set the Receive Buffer Queue Pointer Register */

  physaddr = sam_physramaddr((uint32_t)rxdesc);
  sam_putreg(priv, SAM_EMAC_RBQP, physaddr);
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

static void sam_emac_reset(FAR struct sam_emac_s *priv)
{
  uint32_t regval;

  /* Disable all EMAC interrupts */

  sam_putreg(priv, SAM_EMAC_IDR, EMAC_INT_ALL);

  /* Reset RX and TX logic */

  sam_rxreset(priv);
  sam_txreset(priv);

  /* Disable RX, TX, and statistics */

  regval = EMAC_NCR_TE | EMAC_NCR_RE | EMAC_NCR_WESTAT | EMAC_NCR_CLRSTAT;
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

static void sam_macaddress(FAR struct sam_emac_s *priv)
{
  FAR struct uip_driver_s *dev = &priv->dev;
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
  sam_putreg(priv, SAM_EMAC_SA1B, regval);

  regval = (uint32_t)dev->d_mac.ether_addr_octet[4] |
           (uint32_t)dev->d_mac.ether_addr_octet[5] << 8;
  sam_putreg(priv, SAM_EMAC_SA1T, regval);
}

/****************************************************************************
 * Function: sam_emac_configure
 *
 * Description:
 *  Configure the EMAC interface for DMA operation.
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

static int sam_emac_configure(FAR struct sam_emac_s *priv)
{
  uint32_t regval;

  nvdbg("Entry\n");

  /* Enable clocking to the EMAC peripheral */

  sam_emac_enableclk();

  /* Disable TX, RX, interrupts, etc. */

  sam_putreg(priv, SAM_EMAC_NCR, 0);
  sam_putreg(priv, SAM_EMAC_IDR, EMAC_INT_ALL);

  regval = sam_getreg(priv, SAM_EMAC_NCR);
  regval |= EMAC_NCR_CLRSTAT;
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Clear all status bits in the receive status register. */

  regval = (EMAC_RSR_OVR | EMAC_RSR_REC | EMAC_RSR_BNA);
  sam_putreg(priv, SAM_EMAC_RSR, regval);

  /* Clear all status bits in the transmit status register */

  regval = (EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLES | EMAC_TSR_BEX |
            EMAC_TSR_COMP | EMAC_TSR_UND);
  sam_putreg(priv, SAM_EMAC_TSR, regval);

  /* Clear any pending interrupts */

  (void)sam_getreg(priv, SAM_EMAC_ISR);

  /* Enable/disable the copy of data into the buffers, ignore broadcasts.
   * Don't copy FCS.
   */

  regval  = sam_getreg(priv, SAM_EMAC_NCFGR);
  regval |= (EMAC_NCFGR_DRFCS | EMAC_NCFGR_PAE);

#ifdef CONFIG_NET_PROMISCUOUS
  regval |=  EMAC_NCFGR_CAF;
#else
  regval &= ~EMAC_NCFGR_CAF;
#endif

#ifdef CONFIG_SAMA5_EMAC_NBC
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
  regval |= (EMAC_NCR_RE | EMAC_NCR_TE | EMAC_NCR_WESTAT);
  sam_putreg(priv, SAM_EMAC_NCR, regval);

  /* Setup the interrupts for TX (and errors) */

  regval = (EMAC_INT_RXUBR | EMAC_INT_TUND | EMAC_INT_RLE | EMAC_INT_TXERR |
            EMAC_INT_TCOMP | EMAC_INT_ROVR | EMAC_INT_HRESP | EMAC_INT_PFR |
            EMAC_INT_PTZ);
  sam_putreg(priv, SAM_EMAC_IER, regval);
  return OK;
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
#ifdef CONFIG_SAMA5_EMAC_PREALLOCATE
  /* Use pre-allocated buffers */

  priv->txdesc   = g_txdesc;
  priv->rxdesc   = g_rxdesc;
  priv->txbuffer = g_txbuffer;
  priv->rxbuffer = g_rxbuffer;

#else
  size_t allocsize;

  /* Allocate buffers */

  allocsize = CONFIG_SAMA5_EMAC_NTXBUFFERS * sizeof(struct emac_txdesc_s);
  priv->txdesc = (struct emac_txdesc_s *)kmemalign(8, allocsize);
  if (!priv->txdesc)
    {
      nlldbg("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  memset(priv->txdesc, 0, allocsize);

  allocsize = CONFIG_SAMA5_EMAC_NRXBUFFERS * sizeof(struct emac_rxdesc_s);
  priv->rxdesc = (struct emac_rxdesc_s *)kmemalign(8, allocsize);
  if (!priv->rxdesc)
    {
      nlldbg("ERROR: Failed to allocate RX descriptors\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  memset(priv->rxdesc, 0, allocsize);

  allocsize = CONFIG_SAMA5_EMAC_NTXBUFFERS * EMAC_TX_UNITSIZE;
  priv->txbuffer = (uint8_t *)kmemalign(8, allocsize);
  if (!priv->txbuffer)
    {
      nlldbg("ERROR: Failed to allocate TX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

  allocsize = CONFIG_SAMA5_EMAC_NRXBUFFERS * EMAC_RX_UNITSIZE;
  priv->rxbuffer = (uint8_t *)kmemalign(8, allocsize);
  if (!priv->rxbuffer)
    {
      nlldbg("ERROR: Failed to allocate RX buffer\n");
      sam_buffer_free(priv);
      return -ENOMEM;
    }

#endif

  DEBUGASSERT(((uintptr_t)priv->rxdesc   & 7) = 0 &&
              ((uintptr_t)priv->rxbuffer & 7) = 0 &&
              ((uintptr_t)priv->txdesc   & 7) = 0 &&
              ((uintptr_t)priv->txbuffer & 7) = 0);
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
#ifndef CONFIG_SAMA5_EMAC_PREALLOCATE
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sam_emac_initialize
 *
 * Description:
 *   Initialize the EMAC driver.
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

int sam_emac_initialize(void)
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
      ndbg("ERROR: Failed to create periodic poll timer\n");
      ret = -EAGAIN;
      goto errout;
    }
 
  priv->txtimeout = wd_create();     /* Create TX timeout timer */
  if (!priv->txpoll)
    {
      ndbg("ERROR: Failed to create periodic poll timer\n");
      ret = -EAGAIN;
      goto errout_with_txpoll;
    }

  /* Configure PIO pins to support EMAC */

  sam_ethgpioconfig(priv);

  /* Allocate buffers */

  ret = sam_buffer_initialize(priv);
  if (ret < 0)
    {
      ndbg("ERROR: sam_buffer_initialize failed: %d\n", ret);
      goto errout_with_txtimeout;
    }

  /* Attach the IRQ to the driver.  It will not be enabled at the AIC until
   * the interface is in the 'up' state.
   */

  ret = irq_attach(SAM_IRQ_EMAC, sam_emac_interrupt);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to attach the handler to the IRQ%d\n", SAM_IRQ_EMAC);
      goto errout_with_buffers;
    }

  /* Enable clocking to the EMAC peripheral (just for sam_ifdown()) */

  sam_emac_enableclk();

  /* Put the interface in the down state (disabling clocking again). */

  ret = sam_ifdown(&priv->dev);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to put the interface in the down state: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev);
  if (ret >= 0)
    {
      ndbg("ERROR: netdev_register() failed: %d\n", ret);
      return ret;
    }

errout_with_buffers:
  sam_buffer_free(priv);
errout_with_txtimeout:
  wd_delete(priv->txtimeout);
errout_with_txpoll:
  wd_delete(priv->txpoll);
errout:
  return ret;
}
