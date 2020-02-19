/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flexcan.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "up_arch.h"
#include "chip.h"
#include "s32k1xx_config.h"
#include "hardware/s32k1xx_flexcan.h"
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_flexcan.h"

#ifdef CONFIG_S32K1XX_FLEXCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

  /* Select work queue.  Always use the LP work queue if available.  If not,
   * then LPWORK will re-direct to the HP work queue.
   *
   * NOTE:  However, the network should NEVER run on the high priority work
   * queue!  That queue is intended only to service short back end interrupt
   * processing that never suspends.  Suspending the high priority work queue
   * may bring the system to its knees!
   */

#  define ETHWORK LPWORK
#endif

/* CONFIG_S32K1XX_FLEXCAN_NETHIFS determines the number of physical interfaces
 * that will be supported.
 */
/*
#if CONFIG_S32K1XX_FLEXCAN_NETHIFS != 1
#  error "CONFIG_S32K1XX_FLEXCAN_NETHIFS must be one for now"
#endif

#if CONFIG_S32K1XX_FLEXCAN_NTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_S32K1XX_FLEXCAN_NRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif*/

#define MaskStdID                   0x000007FF;
#define MaskExtID                   0x1FFFFFFF;

//Fixme nice variables/constants
#define RxMBCount                   10
#define TxMBCount                   6
#define TotalMBcount                RxMBCount + TxMBCount
#define TXMBMask                    (((1 << TxMBCount)-1) << RxMBCount)

#define CAN_FIFO_NE                 (1 << 5)
#define CAN_FIFO_OV                 (1 << 6)
#define CAN_FIFO_WARN               (1 << 7)
#define FIFO_IFLAG1                 CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV

static int peak_tx_mailbox_index_ = 0;




/* Normally you would clean the cache after writing new values to the DMA
 * memory so assure that the dirty cache lines are flushed to memory
 * before the DMA occurs.  And you would invalid the cache after a data is
 * received via DMA so that you fetch the actual content of the data from
 * the cache.
 *
 * These conditions are not fully supported here.  If the write-throuch
 * D-Cache is enabled, however, then many of these issues go away:  The
 * cache clean operation does nothing (because there are not dirty cache
 * lines) and the cache invalid operation is innocuous (because there are
 * never dirty cache lines to be lost; valid data will always be reloaded).
 *
 * At present, we simply insist that write through cache be enabled.
 */

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
#  error Write back D-Cache not yet supported
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second.
 */

#define S32K1XX_WDDELAY     (1*CLK_TCK)

/* Align assuming that the D-Cache is enabled (probably 32-bytes).
 *
 * REVISIT: The size of descriptors and buffers must also be in even units
 * of the cache line size  That is because the operations to clean and
 * invalidate the cache will operate on a full 32-byte cache line.  If
 * CONFIG_FLEXCAN_ENHANCEDBD is selected, then the size of the descriptor is
 * 32-bytes (and probably already the correct size for the cache line);
 * otherwise, the size of the descriptors much smaller, only 8 bytes.
 */

#define FLEXCAN_ALIGN        ARMV7M_DCACHE_LINESIZE
#define FLEXCAN_ALIGN_MASK   (FLEXCAN_ALIGN - 1)
#define FLEXCAN_ALIGN_UP(n)  (((n) + FLEXCAN_ALIGN_MASK) & ~FLEXCAN_ALIGN_MASK)

/* TX timeout = 1 minute */

#define S32K1XX_TXTIMEOUT   (60*CLK_TCK)
#define MII_MAXPOLLS      (0x1ffff)
#define LINK_WAITUS       (500*1000)
#define LINK_NLOOPS       (10)

/* Interrupt groups */

#define RX_INTERRUPTS     (FLEXCAN_INT_RXF | FLEXCAN_INT_RXB)
#define TX_INTERRUPTS      FLEXCAN_INT_TXF
#define ERROR_INTERRUPTS  (FLEXCAN_INT_UN    | FLEXCAN_INT_RL   | FLEXCAN_INT_LC | \
                           FLEXCAN_INT_EBERR | FLEXCAN_INT_BABT | FLEXCAN_INT_BABR)

/* The subset of errors that require us to reset the hardware - this list
 * may need to be revisited if it's found that some error above leads to a
 * locking up of the Ethernet interface.
 */

#define CRITICAL_ERROR    (FLEXCAN_INT_UN | FLEXCAN_INT_RL | FLEXCAN_INT_EBERR )

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

#define S32K1XX_BUF_SIZE  FLEXCAN_ALIGN_UP(CONFIG_NET_ETH_PKTSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/


union TXcsType
{
	volatile uint32_t w;
	struct
	{
		volatile uint32_t time_stamp : 16;
		volatile uint32_t dlc : 4;
		volatile uint32_t rtr : 1;
		volatile uint32_t ide : 1;
		volatile uint32_t srr : 1;
		volatile uint32_t res : 1;
		volatile uint32_t code : 4;
		volatile uint32_t res2 : 4;
	};
};

union RXcsType
{
	volatile uint32_t cs;
	struct
	{
		volatile uint32_t time_stamp : 16;
		volatile uint32_t dlc : 4;
		volatile uint32_t rtr : 1;
		volatile uint32_t ide : 1;
		volatile uint32_t srr : 1;
		volatile uint32_t res : 9;
	};
};

union IDType
{
	volatile uint32_t w;
	struct
	{
		volatile uint32_t ext : 29;
		volatile uint32_t resex : 3;
	};
	struct
	{
		volatile uint32_t res : 18;
		volatile uint32_t std : 11;
		volatile uint32_t resstd : 3;
	};
};

union DataType
{
	volatile uint32_t l;
	volatile uint32_t h;
	struct
	{
		volatile uint32_t b3 : 8;
		volatile uint32_t b2 : 8;
		volatile uint32_t b1 : 8;
		volatile uint32_t b0 : 8;
		volatile uint32_t b7 : 8;
		volatile uint32_t b6 : 8;
		volatile uint32_t b5 : 8;
		volatile uint32_t b4 : 8;
	};
};

struct MbTx
{
	union TXcsType CS;
	union IDType ID;
	union DataType data;
};

struct MbRx
{
	union RXcsType CS;
	union IDType ID;
	union DataType data;
};

/* The s32k1xx_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct s32k1xx_driver_s
{
  bool bifup;                  /* true:ifup false:ifdown */
  uint8_t txtail;              /* The oldest busy TX descriptor */
  uint8_t txhead;              /* The next TX descriptor to use */
  uint8_t rxtail;              /* The next RX descriptor to use */
  uint8_t phyaddr;             /* Selected PHY address */
  WDOG_ID txpoll;              /* TX poll timer */
  WDOG_ID txtimeout;           /* TX timeout timer */
  struct work_s irqwork;       /* For deferring interrupt work to the work queue */
  struct work_s pollwork;      /* For deferring poll work to the work queue */
  struct enet_desc_s *txdesc;  /* A pointer to the list of TX descriptor */
  struct enet_desc_s *rxdesc;  /* A pointer to the list of RX descriptors */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;     /* Interface understood by the network */

  struct MbRx *rx;
  struct MbTx *tx;

};


/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct s32k1xx_driver_s g_flexcan[CONFIG_S32K1XX_ENET_NETHIFS];

static uint8_t g_desc_pool[2000]
               __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

#ifndef S32K1XX_BUFFERS_SWAP
#  define s32k1xx_swap32(value) (value)
#  define s32k1xx_swap16(value) (value)
#else
#if 0 /* Use builtins if the compiler supports them */
static inline uint32_t s32k1xx_swap32(uint32_t value);
static inline uint16_t s32k1xx_swap16(uint16_t value);
#else
#  define s32k1xx_swap32 __builtin_bswap32
#  define s32k1xx_swap16 __builtin_bswap16
#endif
#endif

/* Common TX logic */

static bool s32k1xx_txringfull(FAR struct s32k1xx_driver_s *priv);
static int  s32k1xx_transmit(FAR struct s32k1xx_driver_s *priv);
static int  s32k1xx_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void s32k1xx_dispatch(FAR struct s32k1xx_driver_s *priv);
static void s32k1xx_receive(FAR struct s32k1xx_driver_s *priv);
static void s32k1xx_txdone(FAR struct s32k1xx_driver_s *priv);

static void s32k1xx_flexcan_interrupt_work(FAR void *arg);
static int  s32k1xx_flexcan_interrupt(int irq, FAR void *context,
                                   FAR void *arg);

/* Watchdog timer expirations */

static void s32k1xx_txtimeout_work(FAR void *arg);
static void s32k1xx_txtimeout_expiry(int argc, uint32_t arg, ...);

static void s32k1xx_poll_work(FAR void *arg);
static void s32k1xx_polltimer_expiry(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int  s32k1xx_ifup(struct net_driver_s *dev);
static int  s32k1xx_ifdown(struct net_driver_s *dev);

static void s32k1xx_txavail_work(FAR void *arg);
static int  s32k1xx_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  s32k1xx_addmac(struct net_driver_s *dev,
              FAR const uint8_t *mac);
static int  s32k1xx_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
            unsigned long arg);
#endif

/* Initialization */

static void s32k1xx_initbuffers(struct s32k1xx_driver_s *priv);
static void s32k1xx_reset(struct s32k1xx_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Function: s32k1xx_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool s32k1xx_txringfull(FAR struct s32k1xx_driver_s *priv)
{
  uint8_t txnext;

  /* Check if there is room in the hardware to hold another outgoing
   * packet.  The ring is full if incrementing the head pointer would
   * collide with the tail pointer.
   */

  txnext = priv->txhead + 1;
  
  return priv->txtail == txnext;
}

/****************************************************************************
 * Function: s32k1xx_transmit
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
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/


static int s32k1xx_transmit(FAR struct s32k1xx_driver_s *priv)
{
  #warning Missing logic

  struct can_frame *frame = (struct can_frame*)priv->dev.d_buf;

  /*ninfo("CAN id: %i dlc: %i", frame->can_id, frame->can_dlc);

  for(int i = 0; i < frame->can_dlc; i++){
	  ninfo(" %02X", frame->data[i]);
  }
  ninfo("\r\n");*/

  /* Attempt to write frame */
  uint32_t mbi = 0;
  if ((getreg32(S32K1XX_CAN0_ESR2) & (CAN_ESR2_IMB | CAN_ESR2_VPS)) == (CAN_ESR2_IMB | CAN_ESR2_VPS))
  {
	  mbi = (getreg32(S32K1XX_CAN0_ESR2) & CAN_ESR2_LPTM_MASK) >> CAN_ESR2_LPTM_SHIFT;
  }

  uint32_t mb_bit = 1 << (RxMBCount + mbi);

  while (mbi < TxMBCount)
  {

	  if (priv->tx[mbi].CS.code != CAN_TXMB_DATAORREMOTE)
	  {
		  putreg32(mb_bit, S32K1XX_CAN0_IFLAG1);
		  break;
	  }
	  mb_bit <<= 1;
	  mbi++;
  }

  if (mbi == TxMBCount)
  {
	  return 0;       // No transmission for you!
  }

  peak_tx_mailbox_index_ = (peak_tx_mailbox_index_ > mbi ? peak_tx_mailbox_index_ : mbi );

  union TXcsType cs;
  cs.code = CAN_TXMB_DATAORREMOTE;
  struct MbTx* mb = &priv->tx[mbi];
  mb->CS.code = CAN_TXMB_INACTIVE;

  if (0) //FIXME detect Std or Ext id
  {
	  cs.ide = 1;
	  mb->ID.ext = frame->can_id & MaskExtID;
  }
  else
  {
	  mb->ID.std = frame->can_id & MaskStdID;
  }

  //cs.rtr = frame.isRemoteTransmissionRequest();

  cs.dlc = frame->can_dlc;

  //FIXME endian swap instruction or somekind takes 1.5us right now
  mb->data.b0 = frame->data[0];
  mb->data.b1 = frame->data[1];
  mb->data.b2 = frame->data[2];
  mb->data.b3 = frame->data[3];
  mb->data.b4 = frame->data[4];
  mb->data.b5 = frame->data[5];
  mb->data.b6 = frame->data[6];
  mb->data.b7 = frame->data[7];

  /*
   * Registering the pending transmission so we can track its deadline and loopback it as needed
   */
  /*TxItem& txi = pending_tx_[mbi];
  txi.deadline       = tx_deadline;
  txi.frame          = frame;
  txi.loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
  txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
  txi.pending        = TxItem::busy;*/


  s32k1xx_gpiowrite(PIN_PORTD | PIN31, 0);

  mb->CS = cs; // Go.

  uint32_t regval;
  regval = getreg32(S32K1XX_CAN0_IMASK1);
  regval |= mb_bit;
  putreg32(regval, S32K1XX_CAN0_IMASK1);

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txpoll
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

static int s32k1xx_txpoll(struct net_driver_s *dev)
{
  #warning Missing logic

  FAR struct s32k1xx_driver_s *priv =
    (FAR struct s32k1xx_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {

      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          s32k1xx_transmit(priv);
          /*priv->dev.d_buf =
            (uint8_t *)s32k1xx_swap32((uint32_t)priv->txdesc[priv->txhead].data);*/

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if (s32k1xx_txringfull(priv))
            {
              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: s32k1xx_dispatch
 *
 * Description:
 *   A new Rx packet was received; dispatch that packet to the network layer
 *   as necessary.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static inline void s32k1xx_dispatch(FAR struct s32k1xx_driver_s *priv)
{
    
  #warning Missing logic
}

/****************************************************************************
 * Function: s32k1xx_receive
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
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void s32k1xx_receive(FAR struct s32k1xx_driver_s *priv)
{
  #warning Missing logic
	ninfo("FLEXCAN: receive\r\n");
}

/****************************************************************************
 * Function: s32k1xx_txdone
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
 *   Global interrupts are disabled by the watchdog logic.
 *   The network is locked.
 *
 ****************************************************************************/

static void s32k1xx_txdone(FAR struct s32k1xx_driver_s *priv)
{
  #warning Missing logic

  uint32_t tx_iflags;
  tx_iflags = getreg32(S32K1XX_CAN0_IFLAG1) & TXMBMask;

  //FIXME process aborts

  /* Process TX completions */

  uint32_t mb_bit = 1 << RxMBCount;
  for(uint32_t mbi = 0; tx_iflags && mbi < TxMBCount; mbi++)
  {
      if (tx_iflags & mb_bit)
      {
    	  putreg32(mb_bit, S32K1XX_CAN0_IFLAG1);
          tx_iflags &= ~mb_bit;
          //const bool txok = priv->tx[mbi].CS.code != CAN_TXMB_ABORT;
          //handleTxMailboxInterrupt(mbi, txok, utc_usec);
      }
      mb_bit <<= 1;
  }
}

/****************************************************************************
 * Function: s32k1xx_flexcan_interrupt_work
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

static void s32k1xx_flexcan_interrupt_work(FAR void *arg)
{
  #warning Missing logic
}

/****************************************************************************
 * Function: s32k1xx_flexcan_interrupt
 *
 * Description:
 *   Three interrupt sources will vector this this function:
 *   1. Ethernet MAC transmit interrupt handler
 *   2. Ethernet MAC receive interrupt handler
 *   3.
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


static int s32k1xx_flexcan_interrupt(int irq, FAR void *context, FAR void *arg)
{
  #warning Missing logic

	ninfo("FLEXCAN INT %i\r\n", irq);

	FAR struct s32k1xx_driver_s *priv = &g_flexcan[0];
	uint32_t flags;
	flags  = getreg32(S32K1XX_CAN0_IFLAG1);
	flags &= FIFO_IFLAG1;

	if(flags)
	{
		s32k1xx_receive(priv);
	}

	flags  = getreg32(S32K1XX_CAN0_IFLAG1);
	flags &= TXMBMask;

	if(flags)
	{
        s32k1xx_txdone(priv);
	}
}

/****************************************************************************
 * Function: s32k1xx_txtimeout_work
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
 *
 ****************************************************************************/

static void s32k1xx_txtimeout_work(FAR void *arg)
{
  #warning Missing logic
	  ninfo("FLEXCAN: tx timeout work\r\n");
}

/****************************************************************************
 * Function: s32k1xx_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
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

static void s32k1xx_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  #warning Missing logic
	  ninfo("FLEXCAN: tx timeout expiry\r\n");
}

/****************************************************************************
 * Function: s32k1xx_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
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

static void s32k1xx_poll_work(FAR void *arg)
{
  #warning Missing logic
	  //ninfo("FLEXCAN: poll work\r\n");

	  FAR struct s32k1xx_driver_s *priv = (FAR struct s32k1xx_driver_s *)arg;

	  /* Check if there is there is a transmission in progress.  We cannot
	   * perform the TX poll if he are unable to accept another packet for
	   * transmission.
	   */

	  net_lock();
	  if (1) //!s32k1xx_txringfull(priv))
	    {
	      /* If so, update TCP timing states and poll the network for new XMIT
	       * data. Hmmm.. might be bug here.  Does this mean if there is a
	       * transmit in progress, we will missing TCP time state updates?
	       */

	      devif_timer(&priv->dev, S32K1XX_WDDELAY, s32k1xx_txpoll);
	    }

	  /* Setup the watchdog poll timer again in any case */

	  wd_start(priv->txpoll, S32K1XX_WDDELAY, s32k1xx_polltimer_expiry,
	           1, (wdparm_t)priv);
	  net_unlock();

}

/****************************************************************************
 * Function: s32k1xx_polltimer_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Input Parameters:
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

static void s32k1xx_polltimer_expiry(int argc, uint32_t arg, ...)
{
  #warning Missing logic
  FAR struct s32k1xx_driver_s *priv = (FAR struct s32k1xx_driver_s *)arg;

  /* Schedule to perform the poll processing on the worker thread. */

  work_queue(ETHWORK, &priv->pollwork, s32k1xx_poll_work, priv, 0);
}

static void s32k1xx_setfreeze(uint32_t freeze)
{
	uint32_t regval;
	if(freeze)
	{
		/* Enter freeze mode */
		regval  = getreg32(S32K1XX_CAN0_MCR);
		regval |= (CAN_MCR_HALT | CAN_MCR_FRZ);
		putreg32(regval, S32K1XX_CAN0_MCR);
	}
	else
	{
		/* Exit freeze mode */
		regval  = getreg32(S32K1XX_CAN0_MCR);
		regval &= ~(CAN_MCR_HALT | CAN_MCR_FRZ);
		putreg32(regval, S32K1XX_CAN0_MCR);
	}
}

static uint32_t s32k1xx_waitmcr_change(uint32_t mask, uint32_t target_state)
{
	const unsigned Timeout = 1000;
	for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++)
	{
		const bool state = (getreg32(S32K1XX_CAN0_MCR) & mask) != 0;
		if (state == target_state)
		{
			return true;
		}
		up_udelay(10);
	}
	return false;
}

static uint32_t s32k1xx_waitfreezeack_change(uint32_t target_state)
{
    return s32k1xx_waitmcr_change(CAN_MCR_FRZACK, target_state);
}


/****************************************************************************
 * Function: s32k1xx_ifup
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

static int s32k1xx_ifup(struct net_driver_s *dev)
{
  FAR struct s32k1xx_driver_s *priv =
	(FAR struct s32k1xx_driver_s *)dev->d_private;
  uint32_t regval;

  #warning Missing logic
  ninfo("FLEXCAN: test ifup\r\n");

  /* initialize CAN device */
  //FIXME we only support a single can device for now

  //TEST GPIO tming
  s32k1xx_pinconfig(PIN_PORTD | PIN31 | GPIO_OUTPUT);


  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval |= CAN_MCR_MDIS;
  putreg32(regval, S32K1XX_CAN0_MCR);

  /* Set SYS_CLOCK src */
  regval  = getreg32(S32K1XX_CAN0_CTRL1);
  regval |= CAN_CTRL1_CLKSRC;
  putreg32(regval, S32K1XX_CAN0_CTRL1);

  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval &= ~(CAN_MCR_MDIS);
  putreg32(regval, S32K1XX_CAN0_MCR);


  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval |= CAN_MCR_RFEN | CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_SRXDIS
		  | CAN_MCR_IRMQ | CAN_MCR_AEN |
		  (((TotalMBcount - 1) << CAN_MCR_MAXMB_SHIFT) & CAN_MCR_MAXMB_MASK);
  putreg32(regval, S32K1XX_CAN0_MCR);

  regval  = CAN_CTRL2_RRS | CAN_CTRL2_EACEN | CAN_CTRL2_RFFN_16MB; //FIXME TASD
  putreg32(regval, S32K1XX_CAN0_CTRL2);

  /* Enter freeze mode */
  s32k1xx_setfreeze(1);
  if(!s32k1xx_waitfreezeack_change(1))
  {
	  ninfo("FLEXCAN: freeze fail\r\n");
	  return -1;
  }

  /*regval  = getreg32(S32K1XX_CAN0_CTRL1);
  regval |= ((0  << CAN_CTRL1_PRESDIV_SHIFT) & CAN_CTRL1_PRESDIV_MASK)
		  | ((46 << CAN_CTRL1_ROPSEG_SHIFT) & CAN_CTRL1_ROPSEG_MASK)
	      | ((18 << CAN_CTRL1_PSEG1_SHIFT) & CAN_CTRL1_PSEG1_MASK)
		  | ((12 << CAN_CTRL1_PSEG2_SHIFT) & CAN_CTRL1_PSEG2_MASK)
		  | ((12 << CAN_CTRL1_RJW_SHIFT) & CAN_CTRL1_RJW_MASK)
		  | CAN_CTRL1_ERRMSK
		  | CAN_CTRL1_TWRNMSK
		  | CAN_CTRL1_RWRNMSK;

  putreg32(regval, S32K1XX_CAN0_CTRL1);*/

  /* CAN Bit Timing (CBT) configuration for a nominal phase of 1 Mbit/s
   * with 80 time quantas,in accordance with Bosch 2012 specification,
   * sample point at 83.75% */
  regval  = getreg32(S32K1XX_CAN0_CBT);
  regval |= CAN_CBT_BTF |     /* Enable extended bit timing configurations for CAN-FD
                                      for setting up separetely nominal and data phase */
            CAN_CBT_EPRESDIV(0) |  /* Prescaler divisor factor of 1 */
            CAN_CBT_EPROPSEG(46) | /* Propagation segment of 47 time quantas */
            CAN_CBT_EPSEG1(18) |   /* Phase buffer segment 1 of 19 time quantas */
            CAN_CBT_EPSEG2(12) |   /* Phase buffer segment 2 of 13 time quantas */
            CAN_CBT_ERJW(12);      /* Resynchronization jump width same as PSEG2 */
  putreg32(regval, S32K1XX_CAN0_CBT);

#ifdef CAN_FD

  /* Enable CAN FD feature */
  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval |= CAN_MCR_FDEN;
  putreg32(regval, S32K1XX_CAN0_MCR);

  /* CAN-FD Bit Timing (FDCBT) for a data phase of 4 Mbit/s with 20 time quantas,
                 in accordance with Bosch 2012 specification, sample point at 75% */
  regval  = getreg32(S32K1XX_CAN0_FDCBT);
  regval |= CAN_FDCBT_FPRESDIV(0) | /* Prescaler divisor factor of 1 */
		  CAN_FDCBT_FPROPSEG(7) | /* Propagation semgment of 7 time quantas
                                                              (only register that doesn't add 1) */
		  CAN_FDCBT_FPSEG1(6) |   /* Phase buffer segment 1 of 7 time quantas */
		  CAN_FDCBT_FPSEG2(4) |   /* Phase buffer segment 2 of 5 time quantas */
		  CAN_FDCBT_FRJW(4);      /* Resynchorinzation jump width same as PSEG2 */
  putreg32(regval, S32K1XX_CAN0_FDCBT);

  /* Additional CAN-FD configurations */
  regval  = getreg32(S32K1XX_CAN0_FDCTRL);
  regval |= CAN_FDCTRL_FDRATE | /* Enable bit rate switch in data phase of frame */
		  CAN_FDCTRL_TDCEN |  /* Enable transceiver delay compensation */
		  CAN_FDCTRL_TDCOFF(5) |   /* Setup 5 cycles for data phase sampling delay */
		  CAN_FDCTRL_MBDSR0(3);    /* Setup 64 bytes per message buffer (7 MB's) */
  putreg32(regval, S32K1XX_CAN0_FDCTRL);

  regval  = getreg32(S32K1XX_CAN0_CTRL2);
  regval |= CAN_CTRL2_ISOCANFDEN;
  putreg32(regval, S32K1XX_CAN0_CTRL2);
#endif



  /* Iniatilize all MB rx and tx */
  for(int i = 0; i < TotalMBcount; i++)
  {
	  ninfo("MB %i %p\r\n", i, &priv->rx[i]);
	  ninfo("MB %i %p\r\n", i, &priv->rx[i].ID.w);
	  priv->rx[i].CS.cs = 0x0;
	  priv->rx[i].ID.w = 0x0;
	  priv->rx[i].data.l = 0x0;
	  priv->rx[i].data.h = 0x0;
  }

  /* Filtering catchall */
  putreg32(0x0, S32K1XX_CAN0_RXFGMASK);

  for(int i = 0; i < TotalMBcount; i++)
  {
	  putreg32(0,S32K1XX_CAN0_RXIMR(i));
  }

  putreg32(FIFO_IFLAG1 | TXMBMask, S32K1XX_CAN0_IFLAG1);
  putreg32(FIFO_IFLAG1, S32K1XX_CAN0_IMASK1);


  /* Exit freeze mode */
  s32k1xx_setfreeze(0);
  if(!s32k1xx_waitfreezeack_change(0))
  {
	  ninfo("FLEXCAN: unfreeze fail\r\n");
	  return -1;
  }


  /* Set and activate a timer process */

  wd_start(priv->txpoll, S32K1XX_WDDELAY, s32k1xx_polltimer_expiry, 1,
           (wdparm_t)priv);

  priv->bifup = true;

  priv->dev.d_buf = &g_desc_pool;

  /* Set interrupts */
  up_enable_irq(S32K1XX_IRQ_CAN0_BUS);
  up_enable_irq(S32K1XX_IRQ_CAN0_ERROR);
  up_enable_irq(S32K1XX_IRQ_CAN0_LPRX);
  up_enable_irq(S32K1XX_IRQ_CAN0_0_15);

  return OK;
}

/****************************************************************************
 * Function: s32k1xx_ifdown
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

static int s32k1xx_ifdown(struct net_driver_s *dev)
{
  #warning Missing logic
  return OK;
}

/****************************************************************************
 * Function: s32k1xx_txavail_work
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

static void s32k1xx_txavail_work(FAR void *arg)
{
	  FAR struct s32k1xx_driver_s *priv = (FAR struct s32k1xx_driver_s *)arg;

	  /* Ignore the notification if the interface is not yet up */

	  net_lock();
	  if (priv->bifup)
	    {
	      /* Check if there is room in the hardware to hold another outgoing
	       * packet.
	       */

	      if (!s32k1xx_txringfull(priv))
	        {
	          /* No, there is space for another transfer.  Poll the network for
	           * new XMIT data.
	           */

	          devif_poll(&priv->dev, s32k1xx_txpoll);
	        }
	    }

	  net_unlock();
}

/****************************************************************************
 * Function: s32k1xx_txavail
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

static int s32k1xx_txavail(struct net_driver_s *dev)
{
  FAR struct s32k1xx_driver_s *priv =
    (FAR struct s32k1xx_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, s32k1xx_txavail_work, priv, 0);
    }

  return OK;
}


/****************************************************************************
 * Function: s32k1xx_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  int ret;

  switch (cmd)
    {
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */


/****************************************************************************
 * Function: s32k1xx_initbuffers
 *
 * Description:
 *   Initialize FLEXCAN buffers and descriptors
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void s32k1xx_initbuffers(struct s32k1xx_driver_s *priv)
{
  #warning Missing logic
}

/****************************************************************************
 * Function: s32k1xx_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void s32k1xx_reset(struct s32k1xx_driver_s *priv)
{
  unsigned int i;

  /* Set the reset bit and clear the enable bit */

  
  #warning Missing logic

  /* Wait at least 8 clock cycles */

  for (i = 0; i < 10; i++)
    {
      asm volatile ("nop");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: s32k1xx_netinitialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int s32k1xx_netinitialize(int intf)
{
  struct s32k1xx_driver_s *priv;
  int ret;

  //FIXME dynamic board config
  s32k1xx_pinconfig(PIN_CAN0_TX_4);
  s32k1xx_pinconfig(PIN_CAN0_RX_4);

  priv = &g_flexcan[intf];

  ninfo("initialize\r\n");

  /* Get the interface structure associated with this interface number. */

    #warning Missing logic


  /* Attach the flexcan interrupt handler */
  if (irq_attach(S32K1XX_IRQ_CAN0_BUS, s32k1xx_flexcan_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN bus IRQ\n");
      return -EAGAIN;
    }
  if (irq_attach(S32K1XX_IRQ_CAN0_ERROR, s32k1xx_flexcan_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN error IRQ\n");
      return -EAGAIN;
    }
  if (irq_attach(S32K1XX_IRQ_CAN0_LPRX, s32k1xx_flexcan_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN LPRX IRQ\n");
      return -EAGAIN;
    }
  if (irq_attach(S32K1XX_IRQ_CAN0_0_15, s32k1xx_flexcan_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN OR'ed Message buffer (0-15) IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct s32k1xx_driver_s));
  priv->dev.d_ifup    = s32k1xx_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = s32k1xx_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = s32k1xx_txavail;  /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = s32k1xx_ioctl;    /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = (void *)g_flexcan;   /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmissions */
  priv->txpoll        = wd_create();      /* Create periodic poll timer */
  priv->txtimeout     = wd_create();      /* Create TX timeout timer */
  priv->rx            = (struct MbRx *)(S32K1XX_CAN0_MB);
  priv->tx            = (struct MbTx *)(S32K1XX_CAN0_MB + (sizeof(struct MbRx)
		                                * RxMBCount) );

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling s32k1xx_ifdown().
   */

  ninfo("callbacks done\r\n");

  s32k1xx_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

//FIXME CONFIG_S32K1XX_FLEXCAN_NETHIFS == 1 && 

#if !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void)
{
  s32k1xx_netinitialize(0);
}
#endif

#endif /* CONFIG_S32K1XX_FLEXCAN */
