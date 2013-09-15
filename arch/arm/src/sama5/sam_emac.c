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
#include <nuttx/net/mii.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "up_internal.h"

#include "chip.h"
#include "sam_pio.h"
#include "sam_ethernet.h"

#include <arch/board/board.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SAMA5_PHYADDR
#  error "CONFIG_SAMA5_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_SAMA5_MII) && !defined(CONFIG_SAMA5_RMII)
#  warning "Neither CONFIG_SAMA5_MII nor CONFIG_SAMA5_RMII defined"
#endif

#if defined(CONFIG_SAMA5_MII) && defined(CONFIG_SAMA5_RMII)
#  error "Both CONFIG_SAMA5_MII and CONFIG_SAMA5_RMII defined"
#endif

#ifdef CONFIG_SAMA5_AUTONEG
#  ifndef CONFIG_SAMA5_PHYSR
#    error "CONFIG_SAMA5_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_SAMA5_PHYSR_ALTCONFIG
#    ifndef CONFIG_SAMA5_PHYSR_ALTMODE
#      error "CONFIG_SAMA5_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_10HD
#      error "CONFIG_SAMA5_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_100HD
#      error "CONFIG_SAMA5_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_10FD
#      error "CONFIG_SAMA5_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_100FD
#      error "CONFIG_SAMA5_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_SAMA5_PHYSR_SPEED
#      error "CONFIG_SAMA5_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_100MBPS
#      error "CONFIG_SAMA5_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_MODE
#      error "CONFIG_SAMA5_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_SAMA5_PHYSR_FULLDUPLEX
#      error "CONFIG_SAMA5_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

/* Ethernet buffer sizes, number of buffers, and number of descriptors */

#ifndef CONFIG_NET_MULTIBUFFER
#  error "CONFIG_NET_MULTIBUFFER is required"
#endif

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_EMAC_BUFSIZE ((CONFIG_NET_BUFSIZE + 4 + 15) & ~15)

#ifndef CONFIG_SAMA5_EMAC_BUFSIZE
#  define CONFIG_SAMA5_EMAC_BUFSIZE OPTIMAL_EMAC_BUFSIZE
#endif

#if CONFIG_SAMA5_EMAC_BUFSIZE > EMAC_TDES1_TBS1_MASK
#  error "CONFIG_SAMA5_EMAC_BUFSIZE is too large"
#endif

#if (CONFIG_SAMA5_EMAC_BUFSIZE & 15) != 0
#  error "CONFIG_SAMA5_EMAC_BUFSIZE must be aligned"
#endif

#if CONFIG_SAMA5_EMAC_BUFSIZE != OPTIMAL_EMAC_BUFSIZE
#  warning "You using an incomplete/untested configuration"
#endif

#ifndef CONFIG_SAMA5_EMAC_NRXDESC
#  define CONFIG_SAMA5_EMAC_NRXDESC 8
#endif
#ifndef CONFIG_SAMA5_EMAC_NTXDESC
#  define CONFIG_SAMA5_EMAC_NTXDESC 4
#endif

/* We need at least one more free buffer than transmit buffers */

#define SAM_EMAC_NFREEBUFFERS (CONFIG_SAMA5_EMAC_NTXDESC+1)

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
/* This is a helper pointer for accessing the contents of the Ethernet
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

  uint8_t              *rxbuffer;    /* Allocated RX buffers */
  uint8_t              *txbuffer;    /* Allocated TX buffers */
  struct emac_rxdesc_s *rxdesc;      /* Allocated RX descriptors */
  struct emac_txdesc_s *txdesc;      /* Allocated TX descriptors */
  struct emac_txdesc_s *txhead;      /* Next available TX descriptor */
  struct emac_rxdesc_s *rxhead;      /* Next available RX descriptor */

  struct emac_txdesc_s *txtail;      /* First "in_flight" TX descriptor */
  struct emac_rxdesc_s *rxcurr;      /* First RX descriptor of the segment */
  uint16_t              segments;    /* RX segment count */
  uint16_t              inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t            freeb;       /* The free buffer list */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_EMAC_REGDEBUG
   bool               wrlast;     /* Last was a write */
   uintptr_t          addrlast;   /* Last address */
   uint32_t           vallast;    /* Last value */
   int                ntimes;     /* Number of times */
#endif

  /* Descriptor allocations */

  struct emac_rxdesc_s rxtable[CONFIG_SAMA5_EMAC_NRXDESC];
  struct emac_txdesc_s txtable[CONFIG_SAMA5_EMAC_NTXDESC];

  /* Buffer allocations */

  uint8_t rxbuffer[CONFIG_SAMA5_EMAC_NRXDESC*CONFIG_SAMA5_EMAC_BUFSIZE];
  uint8_t alloc[SAM_EMAC_NFREEBUFFERS*CONFIG_SAMA5_EMAC_BUFSIZE];
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

/* Free buffer management */

static void sam_initbuffer(FAR struct sam_emac_s *priv);
static inline uint8_t *sam_allocbuffer(FAR struct sam_emac_s *priv);
static inline void sam_freebuffer(FAR struct sam_emac_s *priv, uint8_t *buffer);
static inline bool sam_isfreebuffer(FAR struct sam_emac_s *priv);

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
static int  sam_interrupt(int irq, FAR void *context);

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

/* Descriptor Initialization */

static void sam_txdescinit(FAR struct sam_emac_s *priv);
static void sam_rxdescinit(FAR struct sam_emac_s *priv);

/* PHY Initialization */

static int  sam_phyread(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t *value);
static int  sam_phywrite(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t value);
#ifdef CONFIG_PHY_DM9161
static inline int sam_dm9161(FAR struct sam_emac_s *priv);
#endif
static int  sam_phyinit(FAR struct sam_emac_s *priv);

/* MAC/DMA Initialization */

#ifdef CONFIG_SAMA5_MII
static inline void sam_selectmii(void);
#endif
#ifdef CONFIG_SAMA5_RMII
static inline void sam_selectrmii(void);
#endif
static inline void sam_ethgpioconfig(FAR struct sam_emac_s *priv);
static void sam_ethreset(FAR struct sam_emac_s *priv);
static int  sam_macconfig(FAR struct sam_emac_s *priv);
static void sam_macaddress(FAR struct sam_emac_s *priv);
static int  sam_macenable(FAR struct sam_emac_s *priv);
static int  sam_ethconfig(FAR struct sam_emac_s *priv);
static int  sam_buffer_initialize(struct sam_emac_s *priv);

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
 * Function: sam_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled.
 *
 ****************************************************************************/

static void sam_initbuffer(FAR struct sam_emac_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc;
       i < SAM_EMAC_NFREEBUFFERS;
       i++, buffer += CONFIG_SAMA5_EMAC_BUFSIZE)
    {
      sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: sam_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline uint8_t *sam_allocbuffer(FAR struct sam_emac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: sam_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline void sam_freebuffer(FAR struct sam_emac_s *priv, uint8_t *buffer)
{
  /* Free the buffer by adding it to to the end of the free buffer list */

  sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: sam_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   True if there are one or more buffers in the free buffer list;
 *   false if the free buffer list is empty
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

static inline bool sam_isfreebuffer(FAR struct sam_emac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
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

static int sam_transmit(FAR struct sam_emac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  struct emac_txdesc_s *txfirst;

  /* The internal (optimal) uIP buffer size may be configured to be larger
   * than the Ethernet buffer size.
   */

#if OPTIMAL_EMAC_BUFSIZE > CONFIG_SAMA5_EMAC_BUFSIZE
  uint8_t *buffer;
  int bufcount;
  int lastsize;
  int i;
#endif

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  txdesc  = priv->txhead;
  txfirst = txdesc;

  nllvdbg("d_len: %d d_buf: %p txhead: %p tdes0: %08x\n",
          priv->dev.d_len, priv->dev.d_buf, txdesc, txdesc->tdes0);

  DEBUGASSERT(txdesc && (txdesc->tdes0 & EMAC_TDES0_OWN) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_EMAC_BUFSIZE > CONFIG_SAMA5_EMAC_BUFSIZE
  if (priv->dev.d_len > CONFIG_SAMA5_EMAC_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (CONFIG_SAMA5_EMAC_BUFSIZE-1)) / CONFIG_SAMA5_EMAC_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * CONFIG_SAMA5_EMAC_BUFSIZE;

      nllvdbg("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= EMAC_TDES0_FS;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & EMAC_TDES0_OWN) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount-1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (EMAC_TDES0_LS | EMAC_TDES0_IC);

              /* This segement is, most likely, of fractional buffersize */

              txdesc->tdes1  = lastsize;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~EMAC_TDES0_IC;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = CONFIG_SAMA5_EMAC_BUFSIZE;
              buffer        += CONFIG_SAMA5_EMAC_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= EMAC_TDES0_OWN;
          txdesc         = (struct emac_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (EMAC_TDES0_FS | EMAC_TDES0_LS | EMAC_TDES0_IC);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_BUFSIZE);
      txdesc->tdes1 = priv->dev.d_len;

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set OWN bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= EMAC_TDES0_OWN;

      /* Point to the next available TX descriptor */

      txdesc = (struct emac_txdesc_s *)txdesc->tdes3;
    }

  /* Remember where we left off in the TX descriptor chain */

  priv->txhead = txdesc;

  /* Detach the buffer from priv->dev structure.  That buffer is now
   * "in-flight".
   */

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* If there is no other TX buffer, in flight, then remember the location
   * of the TX descriptor.  This is the location to check for TX done events.
   */

  if (!priv->txtail)
    {
      DEBUGASSERT(priv->inflight == 0);
      priv->txtail = txfirst;
    }

  /* Increment the number of TX transfer in-flight */

  priv->inflight++;

  nllvdbg("txhead: %p txtail: %p inflight: %d\n",
          priv->txhead, priv->txtail, priv->inflight);

  /* If all TX descriptors are in-flight, then we have to disable receive interrupts
   * too.  This is because receive events can trigger more un-stoppable transmit
   * events.
   */

  if (priv->inflight >= CONFIG_SAMA5_EMAC_NTXDESC)
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

      /* Check if the next TX descriptor is owned by the Ethernet DMA or CPU.  We
       * cannot perform the TX poll if we are unable to accept another packet for
       * transmission.
       *
       * In a race condition, EMAC_TDES0_OWN may be cleared BUT still not available
       * because sam_freeframe() has not yet run.  If sam_freeframe() has run,
       * the buffer1 pointer (tdes2) will be nullified (and inflight should be <
       * CONFIG_SAMA5_EMAC_NTXDESC).
       */

      if ((priv->txhead->tdes0 & EMAC_TDES0_OWN) != 0 ||
           priv->txhead->tdes2 != 0)
        {
          /* We have to terminate the poll if we have no more descriptors
           * available for another transfer.
           */

          return -EBUSY;
        }

      /* We have the descriptor, we can continue the poll. Allocate a new
       * buffer for the poll.
       */

      dev->d_buf = sam_allocbuffer(priv);

      /* We can't continue the poll if we have no buffers */

      if (dev->d_buf == NULL)
        {
          /* Terminate the poll. */

          return -ENOMEM;
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

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, EMAC_TDES0_OWN may be cleared BUT still not available
   * because sam_freeframe() has not yet run.  If sam_freeframe() has run,
   * the buffer1 pointer (tdes2) will be nullified (and inflight should be <
   * CONFIG_SAMA5_EMAC_NTXDESC).
   */

  if ((priv->txhead->tdes0 & EMAC_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll uIP for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = sam_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          (void)uip_poll(dev, sam_uiptxpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              sam_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
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
      rxdesc->rdes0 = EMAC_RDES0_OWN;
      rxdesc = (struct emac_rxdesc_s *)rxdesc->rdes3;
    }

  /* Reset the segment managment logic */

  priv->rxcurr   = NULL;
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
  int i;

  nllvdbg("rxhead: %p rxcurr: %p segments: %d\n",
          priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!sam_isfreebuffer(priv))
    {
      nlldbg("No free buffers\n");
      return -ENOMEM;
    }

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

  rxdesc = priv->rxhead;
  for (i = 0;
       (rxdesc->rdes0 & EMAC_RDES0_OWN) == 0 &&
        i < CONFIG_SAMA5_EMAC_NRXDESC &&
        priv->inflight < CONFIG_SAMA5_EMAC_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & EMAC_RDES0_FS) != 0 &&
          (rxdesc->rdes0 & EMAC_RDES0_LS) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;
        }

      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & EMAC_RDES0_LS) == 0)&&
               ((rxdesc->rdes0 & EMAC_RDES0_FS) == 0))
        {
          priv->segments++;
        }

      /* Otherwise, it is the last segment in the frame */

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
              rxcurr = priv->rxcurr;
            }

          nllvdbg("rxhead: %p rxcurr: %p segments: %d\n",
              priv->rxhead, priv->rxcurr, priv->segments);

          /* Check if any errors are reported in the frame */

          if ((rxdesc->rdes0 & EMAC_RDES0_ES) == 0)
            {
              struct uip_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes0 & EMAC_RDES0_FL_MASK) >> EMAC_RDES0_FL_SHIFT) - 4;

              /* Get a buffer from the free list.  We don't even check if
               * this is successful because we already assure the free
               * list is not empty above.
               */

              buffer = sam_allocbuffer(priv);

              /* Take the buffer from the RX descriptor of the first free
               * segment, put it into the uIP device structure, then replace
               * the buffer in the RX descriptor with the newly allocated
               * buffer.
               */

              DEBUGASSERT(dev->d_buf == NULL);
              dev->d_buf    = (uint8_t*)rxcurr->rdes2;
              rxcurr->rdes2 = (uint32_t)buffer;

              /* Return success, remebering where we should re-start scanning
               * and resetting the segment scanning logic
               */

              priv->rxhead   = (struct emac_rxdesc_s*)rxdesc->rdes3;
              sam_freesegment(priv, rxcurr, priv->segments);

              nllvdbg("rxhead: %p d_buf: %p d_len: %d\n",
                      priv->rxhead, dev->d_buf, dev->d_len);

              return OK;
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nlldbg("DROPPED: RX descriptor errors: %08x\n", rxdesc->rdes0);
              sam_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct emac_rxdesc_s*)rxdesc->rdes3;
    }

  /* We get here after all of the descriptors have been scanned or when rxdesc points
   * to the first descriptor owned by the DMA.  Remember where we left off.
   */

  priv->rxhead = rxdesc;

  nllvdbg("rxhead: %p rxcurr: %p segments: %d\n",
          priv->rxhead, priv->rxcurr, priv->segments);

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
   * Ethernet frames.
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

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          sam_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
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
  int i;

  nllvdbg("txhead: %p txtail: %p inflight: %d\n",
          priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      for (i = 0; (txdesc->tdes0 & EMAC_TDES0_OWN) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          nllvdbg("txtail: %p tdes0: %08x tdes2: %08x tdes3: %08x\n",
                  txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & EMAC_TDES0_FS) != 0)
            {
              /* Yes.. Free the buffer */

              sam_freebuffer(priv, (uint8_t*)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segement of a TX frame */

          if ((txdesc->tdes0 & EMAC_TDES0_LS) != 0)
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
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct emac_txdesc_s*)txdesc->tdes3;
        }

      /* We get here if (1) there are still frames "in-flight". Remember
       * where we left off.
       */

      priv->txtail = txdesc;

      nllvdbg("txhead: %p txtail: %p inflight: %d\n",
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
 * Function: sam_interrupt
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

static int sam_interrupt(int irq, FAR void *context)
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

  /* Check if the next TX descriptor is owned by the Ethernet DMA or CPU.  We
   * cannot perform the timer poll if we are unable to accept another packet
   * for transmission.  Hmmm.. might be bug here.  Does this mean if there is
   * a transmit in progress, we will miss TCP time state updates?
   *
   * In a race condition, EMAC_TDES0_OWN may be cleared BUT still not available
   * because sam_freeframe() has not yet run.  If sam_freeframe() has run,
   * the buffer1 pointer (tdes2) will be nullified (and inflight should be <
   * CONFIG_SAMA5_EMAC_NTXDESC).
   */

  if ((priv->txhead->tdes0 & EMAC_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then perform the timer poll.  Allocate a
       * buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = sam_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          /* Update TCP timing states and poll uIP for new XMIT data.
           */

          (void)uip_timer(dev, sam_uiptxpoll, SAM_POLLHSEC);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              sam_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, SAM_WDDELAY, sam_polltimer, 1, arg);
}

/****************************************************************************
 * Function: sam_ifup
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

static int sam_ifup(struct uip_driver_s *dev)
{
  FAR struct sam_emac_s *priv = (FAR struct sam_emac_s *)dev->d_private;
  int ret;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Configure the Ethernet interface for DMA operation. */

  ret = sam_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, SAM_WDDELAY, sam_polltimer, 1, (uint32_t)priv);

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(SAM_IRQ_ETH);

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

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(SAM_IRQ_ETH);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the sam_ifup() always
   * successfully brings the interface back up.
   */

  sam_ethreset(priv);

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
 * Function: sam_txdescinit
 *
 * Description:
 *   Initializes the DMA TX descriptors in chain mode.
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

static void sam_txdescinit(FAR struct sam_emac_s *priv)
{
  struct emac_txdesc_s *txdesc;
  int i;

  /* priv->txhead will point to the first, available TX descriptor in the chain.
   * Set the priv->txhead pointer to the first descriptor in the table.
   */

  priv->txhead = priv->txtable;

  /* priv->txtail will point to the first segment of the oldest pending
   * "in-flight" TX transfer.  NULL means that there are no active TX
   * transfers.
   */

   priv->txtail   = NULL;
   priv->inflight = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_SAMA5_EMAC_NTXDESC; i++)
    {
      txdesc = &priv->txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = EMAC_TDES0_TCH;

#ifdef CHECKSUM_BY_HARDWARE
      /* Enable the checksum insertion for the TX frames */

      txdesc->tdes0 |= EMAC_TDES0_CIC_ALL;
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->tdes2 = 0;

      /* Initialize the next descriptor with the Next Descriptor Polling Enable */

      if (i < (CONFIG_SAMA5_EMAC_NTXDESC-1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          txdesc->tdes3 = (uint32_t)&priv->txtable[i+1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          txdesc->tdes3 = (uint32_t)priv->txtable;
        }
    }

  /* Set Transmit Desciptor List Address Register */
#warning "Missing logic"
}

/****************************************************************************
 * Function: sam_rxdescinit
 *
 * Description:
 *   Initializes the DMA RX descriptors in chain mode.
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

static void sam_rxdescinit(FAR struct sam_emac_s *priv)
{
  struct emac_rxdesc_s *rxdesc;
  int i;

  /* priv->rxhead will point to the first,  RX descriptor in the chain.
   * This will be where we receive the first incomplete frame.
   */

  priv->rxhead = priv->rxtable;

  /* If we accumulate the frame in segments, priv->rxcurr points to the
   * RX descriptor of the first segment in the current TX frame.
   */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Initialize each TX descriptor */

  for (i = 0; i < CONFIG_SAMA5_EMAC_NRXDESC; i++)
    {
      rxdesc = &priv->rxtable[i];

      /* Set Own bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = EMAC_RDES0_OWN;

      /* Set Buffer1 size and Second Address Chained bit and enabled DMA
       * RX desc receive interrupt
       */

      rxdesc->rdes1 = EMAC_RDES1_RCH | (uint32_t)CONFIG_SAMA5_EMAC_BUFSIZE;

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&priv->rxbuffer[i*CONFIG_SAMA5_EMAC_BUFSIZE];

      /* Initialize the next descriptor with the Next Descriptor Polling Enable */

      if (i < (CONFIG_SAMA5_EMAC_NRXDESC-1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          rxdesc->rdes3 = (uint32_t)&priv->rxtable[i+1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          rxdesc->rdes3 = (uint32_t)priv->rxtable;
        }
    }

  /* Set Receive Descriptor List Address Register */
#warning Missing logic
}

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

  ret = sam_phyread(CONFIG_SAMA5_PHYADDR, MII_PHYID1, &phyval);
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

  ret = sam_phyread(CONFIG_SAMA5_PHYADDR, 16, &phyval);
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

  ret = sam_phywrite(CONFIG_SAMA5_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      ndbg("Failed to reset the PHY: %d\n", ret);
      return ret;
    }
  up_mdelay(PHY_RESET_DELAY);

  /* Perform any necessary, board-specific PHY initialization */

#ifdef CONFIG_SAMA5_PHYINIT
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

#ifdef CONFIG_SAMA5_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = sam_phyread(CONFIG_SAMA5_PHYADDR, MII_MSR, &phyval);
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

  ret = sam_phywrite(CONFIG_SAMA5_PHYADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      ndbg("Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = sam_phyread(CONFIG_SAMA5_PHYADDR, MII_MSR, &phyval);
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

  ret = sam_phyread(CONFIG_SAMA5_PHYADDR, CONFIG_SAMA5_PHYSR, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  nvdbg("PHYSR[%d]: %04x\n", CONFIG_SAMA5_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.  IF
   * This CONFIG_SAMA5_PHYSR_ALTCONFIG is selected, this indicates that the PHY
   * represents speed and mode information are combined, for example, with
   * separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_SAMA5_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_SAMA5_PHYSR_ALTMODE)
    {
      default:
      case CONFIG_SAMA5_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_SAMA5_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_SAMA5_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_SAMA5_PHYSR_100FD:
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
  if ((phyval & CONFIG_SAMA5_PHYSR_MODE) == CONFIG_SAMA5_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_SAMA5_PHYSR_SPEED) == CONFIG_SAMA5_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotion not selected */

  phyval = 0;
#ifdef CONFIG_SAMA5_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_SAMA5_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  ret = sam_phywrite(CONFIG_SAMA5_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     ndbg("Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }
  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_SAMA5_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_SAMA5_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ndbg("Duplex: %s Speed: %d MBps\n",
       priv->fduplex ? "FULL" : "HALF",
       priv->mbps100 ? 100 : 10);

  return OK;
}

/************************************************************************************
 * Name: sam_selectmii
 *
 * Description:
 *   Selects the MII inteface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_MII
static inline void sam_selectmii(void)
{
#warning Missing logic
}
#endif

/************************************************************************************
 * Name: sam_selectrmii
 *
 * Description:
 *   Selects the RMII inteface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void sam_selectrmii(void)
{
#warning Missing logic
}

/****************************************************************************
 * Function: sam_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the Ethernet interface.
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

#if defined(CONFIG_SAMA5_MII)

  /* Select the MII interface */

  sam_selectmii();

  /* Provide clocking */
#warning Missing logic

# endif

  /* Set up the RMII interface. */

#elif defined(CONFIG_SAMA5_RMII)

  /* Select the RMII interface */

  sam_selectrmii();

  /* Provide clocking */
#warning Missing logic

#endif
}

/****************************************************************************
 * Function: sam_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
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

static void sam_ethreset(FAR struct sam_emac_s *priv)
{
  uint32_t regval;

  /* Reset the EMAC */
#warning Missing logic

  /* Wait for reset to complete */
#warning Missing logic
}

/****************************************************************************
 * Function: sam_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
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

static int sam_macconfig(FAR struct sam_emac_s *priv)
{
  uint32_t regval;

#warning Missing logic

  if (priv->fduplex)
    {
#warning Missing logic
    }

  if (priv->mbps100)
    {
#warning Missing logic
    }

#warning Missing logic

  /* DMA Configuration */
#warning Missing logic

  return OK;
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

  regval = (uint32_t)dev->d_mac.ether_addr_octet[0]       |,
           (uint32_t)dev->d_mac.ether_addr_octet[1] << 8  |,
           (uint32_t)dev->d_mac.ether_addr_octet[2] << 16 |,
           (uint32_t)dev->d_mac.ether_addr_octet[3] << 24 |,
  sam_putreg(priv, SAM_GMAC_SAB1, regval);

  regval = (uint32_t)dev->d_mac.ether_addr_octet[4]       |,
           (uint32_t)dev->d_mac.ether_addr_octet[5] << 8  |,
  sam_putreg(priv, SAM_GMAC_SAT1, regval);
}

/****************************************************************************
 * Function: sam_macenable
 *
 * Description:
 *  Enable normal MAC operation.
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

static int sam_macenable(FAR struct sam_emac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  sam_macaddress(priv);

  /* Enable transmit state machine of the MAC for transmission on the MII */
#warning Missing logic

  /* Flush Transmit FIFO */
#warning Missing logic

  /* Enable receive state machine of the MAC for reception from the MII */
#warning Missing logic

  /* Start DMA transmission */
#warning Missing logic

  /* Start DMA reception */
#warning Missing logic

  /* Enable Ethernet DMA interrupts */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * Function: sam_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
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

static int sam_ethconfig(FAR struct sam_emac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized early in the boot-up
   * sequence in sam_rcc.c.
   */

  /* Reset the Ethernet block */

  nllvdbg("Reset the Ethernet block\n");
  sam_ethreset(priv);

  /* Initialize the PHY */

  nllvdbg("Initialize the PHY\n");
  ret = sam_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  nllvdbg("Initialize the MAC and DMA\n");
  ret = sam_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the free buffer list */

  sam_initbuffer(priv);

  /* Initialize TX Descriptors list: Chain Mode */

  sam_txdescinit(priv);

  /* Initialize RX Descriptors list: Chain Mode  */

  sam_rxdescinit(priv);

  /* Enable normal MAC operation */

  nllvdbg("Enable normal operation\n");
  return sam_macenable(priv);
}

/****************************************************************************
 * Function: sam_buffer_initialize
 *
 * Description:
 *   Allocate alligned TX and RX descriptors and buffers.  For the case of
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
  /* Allocate buffers */

  priv->txdesc = (struct emac_txdesc_s *)
    kmemalign(8, CONFIG_SAMA5_EMAC_NTXBUFFERS * sizeof(struct emac_txdesc_s));
  if (!priv->txdesc)
    {
      nlldbg("ERROR: Failed to allocate TX descriptors\n");
      return -ENOMEM;
    }

  priv->rxdesc = (struct emac_rxdesc_s *)
    kmemalign(8, CONFIG_SAMA5_EMAC_NRXBUFFERS * sizeof(struct emac_rxdesc_s));
  if (!priv->txdesc)
    {
      nlldbg("ERROR: Failed to allocate RX descriptors\n");
      kfree(priv->txdesc);
      return -ENOMEM;
    }

  priv->txbuffer = (uint8_t *)
    kmemalign(8, CONFIG_SAMA5_GMAC_NTXBUFFERS * GMAC_TX_UNITSIZE);
  if (!priv->txbuffer)
    {
      nlldbg("ERROR: Failed to allocate TX buffer\n");
      kfree(priv->txdesc);
      kfree(priv->rxdesc);
      return -ENOMEM;
    }

  priv->rxbuffer = (uint8_t *)kmemalign(8, CONFIG_SAMA5_GMAC_NRXBUFFERS * GMAC_RX_UNITSIZE);
  if (!priv->rxbuffer)
    {
      nlldbg("ERROR: Failed to allocate RX buffer\n");
      kfree(priv->txdesc);
      kfree(priv->rxdesc);
      kfree(priv->rxbuffer);
      return -ENOMEM;
    }
#endif

  DEBUGASSERT(((uintptr_t)priv->rxdesc   & 7) = 0 &&
              ((uintptr_t)priv->rxbuffer & 7) = 0 &&
              ((uintptr_t)priv->txdesc   & 7) = 0 &&
              ((uintptr_t)priv->rxbuffer & 7) = 0 &&
  return OK;
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
  struct sam_emac_s *priv;

  /* Get the interface structure associated with this interface number. */

  priv = &g_emac;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct sam_emac_s));
  priv->dev.d_ifup    = sam_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = sam_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = sam_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = sam_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = sam_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (void*)&g_emac; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll       = wd_create();   /* Create periodic poll timer */
  priv->txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Configure PIO pins to support EMAC */

  sam_ethgpioconfig(priv);

  /* Attach the IRQ to the driver */

  if (irq_attach(SAM_IRQ_ETH, sam_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Put the interface in the down state. */

  sam_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->dev);
  return OK;
}
