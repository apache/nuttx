/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flexcan.c
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

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>

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
   //FIXME maybe for enet not sure for FLEXCAN
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

#if 0
#if CONFIG_S32K1XX_FLEXCAN_NETHIFS != 1
#  error "CONFIG_S32K1XX_FLEXCAN_NETHIFS must be one for now"
#endif

#if CONFIG_S32K1XX_FLEXCAN_NTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_S32K1XX_FLEXCAN_NRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif
#endif

#define MASKSTDID                   0x000007ff
#define MASKEXTID                   0x1fffffff
#define FLAGEFF                     (1 << 31) /* Extended frame format */
#define FLAGRTR                     (1 << 30) /* Remote transmission request */

/* Fixme nice variables/constants */

#define CAN_FD

#define RXMBCOUNT                   5
#define TXMBCOUNT                   2
#define TOTALMBCOUNT                RXMBCOUNT + TXMBCOUNT

#define IFLAG1_RX                   ((1 << RXMBCOUNT)-1)
#define IFLAG1_TX                   (((1 << TXMBCOUNT)-1) << RXMBCOUNT)

#define CAN_FIFO_NE                 (1 << 5)
#define CAN_FIFO_OV                 (1 << 6)
#define CAN_FIFO_WARN               (1 << 7)

#define POOL_SIZE                   1

/* Interrupt flags for RX fifo */
#define IFLAG1_RXFIFO               (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)

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


/****************************************************************************
 * Private Types
 ****************************************************************************/

union cs_e
{
  volatile uint32_t cs;
  struct
  {
    volatile uint32_t time_stamp : 16;
    volatile uint32_t dlc : 4;
    volatile uint32_t rtr : 1;
    volatile uint32_t ide : 1;
    volatile uint32_t srr : 1;
    volatile uint32_t res : 1;
    volatile uint32_t code : 4;
    volatile uint32_t res2 : 1;
    volatile uint32_t esi : 1;
    volatile uint32_t brs : 1;
    volatile uint32_t edl : 1;
  };
};

union id_e
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

union data_e
{
  volatile uint32_t w00;
  struct
  {
    volatile uint32_t b03 : 8;
    volatile uint32_t b02 : 8;
    volatile uint32_t b01 : 8;
    volatile uint32_t b00 : 8;
  };
};

struct mb_s
{
  union cs_e cs;
  union id_e id;
#ifdef CAN_FD
  union data_e data[16];
#else
  union data_e data[2];
#endif
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
#ifdef CAN_FD
  struct canfd_frame *txdesc;  /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc;  /* A pointer to the list of RX descriptors */
#else
  struct can_frame *txdesc;  /* A pointer to the list of TX descriptor */
  struct can_frame *rxdesc;  /* A pointer to the list of RX descriptors */
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;     /* Interface understood by the network */

  struct mb_s *rx;
  struct mb_s *tx;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct s32k1xx_driver_s g_flexcan[CONFIG_S32K1XX_ENET_NETHIFS];

#ifdef CAN_FD
static uint8_t g_tx_pool[sizeof(struct canfd_frame)*POOL_SIZE];
static uint8_t g_rx_pool[sizeof(struct canfd_frame)*POOL_SIZE];
#else
static uint8_t g_tx_pool[sizeof(struct can_frame)*POOL_SIZE]
               __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
static uint8_t g_rx_pool[sizeof(struct can_frame)*POOL_SIZE]
               __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif


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

/****************************************************************************
 * Name: arm_clz
 *
 * Description:
 *   Access to CLZ instructions
 *
 * Input Parameters:
 *   value - The value to perform the Count Leading Zeros operation on
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t arm_clz(unsigned int value)
{
  uint32_t ret;

  __asm__ __volatile__ ("clz %0, %1" : "=r"(ret) : "r"(value));
  return ret;
}

/* Common TX logic */

static bool s32k1xx_txringfull(FAR struct s32k1xx_driver_s *priv);
static int  s32k1xx_transmit(FAR struct s32k1xx_driver_s *priv);
static int  s32k1xx_txpoll(struct net_driver_s *dev);

/* Helper functions */

static void s32k1xx_setenable(uint32_t enable);
static void s32k1xx_setfreeze(uint32_t freeze);
static uint32_t s32k1xx_waitmcr_change(uint32_t mask,
                                       uint32_t target_state);

/* Interrupt handling */

static void s32k1xx_dispatch(FAR struct s32k1xx_driver_s *priv);
static void s32k1xx_receive(FAR struct s32k1xx_driver_s *priv, uint32_t flags);
static void s32k1xx_txdone(FAR struct s32k1xx_driver_s *priv, uint32_t flags);

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

#ifdef CONFIG_NETDEV_IOCTL
static int  s32k1xx_ioctl(struct net_driver_s *dev, int cmd,
                          unsigned long arg);
#endif

/* Initialization */

static void s32k1xx_initbuffers(struct s32k1xx_driver_s *priv);
static int  s32k1xx_initialize(struct s32k1xx_driver_s *priv);
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
  uint32_t mbi = 0;

  while (mbi < TXMBCOUNT)
	{
	  if (priv->tx[mbi].cs.code != CAN_TXMB_DATAORREMOTE)
		{
		  return 0;
		}
	  mbi++;
	}
  return 1;
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

  /* Attempt to write frame */

  uint32_t mbi = 0;
  if ((getreg32(S32K1XX_CAN0_ESR2) & (CAN_ESR2_IMB | CAN_ESR2_VPS)) ==
      (CAN_ESR2_IMB | CAN_ESR2_VPS))
    {
      mbi  = ((getreg32(S32K1XX_CAN0_ESR2) &
    		CAN_ESR2_LPTM_MASK) >> CAN_ESR2_LPTM_SHIFT);
      mbi -= RXMBCOUNT;
    }

  uint32_t mb_bit = 1 << (RXMBCOUNT + mbi);

  while (mbi < TXMBCOUNT)
    {
      if (priv->tx[mbi].cs.code != CAN_TXMB_DATAORREMOTE)
        {
          putreg32(mb_bit, S32K1XX_CAN0_IFLAG1);
          break;
        }

      mb_bit <<= 1;
      mbi++;
    }

  if (mbi == TXMBCOUNT)
    {
      nwarn("No TX MB available mbi %i\r\n", mbi);
      return 0;       /* No transmission for you! */
    }

  peak_tx_mailbox_index_ =
    (peak_tx_mailbox_index_ > mbi ? peak_tx_mailbox_index_ : mbi);

  union cs_e cs;
  cs.code = CAN_TXMB_DATAORREMOTE;
  struct mb_s *mb = &priv->tx[mbi];
  mb->cs.code = CAN_TXMB_INACTIVE;

  if(priv->dev.d_len == sizeof(struct can_frame))
    {
	  struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;

	  if (0) /* FIXME detect Std or Ext id */
	    {
	      cs.ide = 1;
	      mb->id.ext = frame->can_id & MASKEXTID;
	    }
	  else
	    {
	      mb->id.std = frame->can_id & MASKSTDID;
	    }

	#if 0
	  cs.rtr = frame.isRemoteTransmissionRequest();
	#endif

	  cs.dlc = frame->can_dlc;

	  mb->data[0].w00 = __builtin_bswap32(*(uint32_t*)&frame->data[0]);
	  mb->data[1].w00 = __builtin_bswap32(*(uint32_t*)&frame->data[4]);

    }
  else /* CAN FD frame */
    {
  	  struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;

  	  cs.edl = 1; /* CAN FD Frame */

  	  if (0) /* FIXME detect Std or Ext id */
  	    {
  	      cs.ide = 1;
  	      mb->id.ext = frame->can_id & MASKEXTID;
  	    }
  	  else
  	    {
  	      mb->id.std = frame->can_id & MASKSTDID;
  	    }

  	#if 0
  	  cs.rtr = frame.isRemoteTransmissionRequest();
  	#endif

  	  if(frame->len < 9)
  	    {
  		  cs.dlc = frame->len;
  	    }
  	  else
  	    {
  	      if (frame->len < 13)
  	        {
			  cs.dlc = 9;
  	        }
  	      else if (frame->len < 17)
  	        {
			   cs.dlc = 10;
  	        }
  	      else if (frame->len < 21)
  	        {
			   cs.dlc = 11;
  	        }
  	      else if (frame->len < 25)
  	        {
			   cs.dlc = 12;
  	        }
  	      else if (frame->len < 33)
  	        {
			   cs.dlc = 13;
  	        }
  	      else if (frame->len < 49)
  	        {
			   cs.dlc = 14;
  	        }
  	      else if (frame->len < 65)
  	        {
			   cs.dlc = 15;
  	        }
  	      else
  	        {
  	    	   cs.dlc = 15; /* FIXME check CAN FD spec */
  	        }
  	    }

  	  uint32_t* frame_data_word = (uint32_t*)&frame->data[0];

	  for(int i = 0; i < (frame->len + 4 - 1) / 4; i++)
		{
	  	  mb->data[i].w00 = __builtin_bswap32(frame_data_word[i]);
		}
    }


  s32k1xx_gpiowrite(PIN_PORTD | PIN31, 0);

  mb->cs = cs; /* Go. */

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
#if 0
          //FIXME implement ring buffer and increment pointer just like the enet driver??
          priv->dev.d_buf =
            (uint8_t *)s32k1xx_swap32((uint32_t)priv->txdesc[priv->txhead].data);
#endif

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

static void s32k1xx_receive(FAR struct s32k1xx_driver_s *priv, uint32_t flags)
{
  #warning Missing logic
  uint32_t regval;

  s32k1xx_gpiowrite(PIN_PORTD | PIN31, 1);


  //FIXME naive what if multiple flags are high??
  uint32_t mb_index = arm_clz(flags);

  if (mb_index)
    {
      struct mb_s *rf = &priv->rx[31 - mb_index];

      /* Read the frame contents */

      if(rf->cs.edl) /* CAN FD frame */
        {
    	  struct canfd_frame* frame = priv->rxdesc;

          if (rf->cs.ide)
            {
              frame->can_id = MASKEXTID & rf->id.ext;
              frame->can_id |= FLAGEFF;
            }
          else
            {
              frame->can_id = MASKSTDID & rf->id.std;
            }

          if (rf->cs.rtr)
            {
              frame->can_id |= FLAGRTR;
            }

          if(rf->cs.dlc < 9){
              frame->len = rf->cs.dlc;
          } else {
        	  switch(rf->cs.dlc)
        	    {
        	     case 9:
        	       frame->len = 12;
        	       break;

        	     case 10:
        	       frame->len = 16;
        	       break;

        	     case 11:
        	       frame->len = 20;
        	       break;

        	     case 12:
        	       frame->len = 24;
        	       break;

        	     case 13:
        	       frame->len = 32;
        	       break;

        	     case 14:
        	       frame->len = 48;
        	       break;

        	     case 15:
        	       frame->len = 64;
        	       break;
        	    }
          }

    	  uint32_t* frame_data_word = (uint32_t*)&frame->data[0];

          for(int i = 0; i < (frame->len + 4 - 1) / 4; i++)
            {
        	  frame_data_word[i] = __builtin_bswap32(rf->data[i].w00);
            }

          /* Clear MB interrupt flag */
          regval  = getreg32(S32K1XX_CAN0_IFLAG1);
          regval |= (0x80000000 >> mb_index);
          putreg32(regval, S32K1XX_CAN0_IFLAG1);

          /* Copy the buffer pointer to priv->dev..  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct canfd_frame);
          priv->dev.d_buf = frame;
        }
      else /* CAN 2.0 Frame */
        {
    	  struct can_frame* frame = priv->rxdesc;

          if (rf->cs.ide)
            {
              frame->can_id = MASKEXTID & rf->id.ext;
              frame->can_id |= FLAGEFF;
            }
          else
            {
              frame->can_id = MASKSTDID & rf->id.std;
            }

          if (rf->cs.rtr)
            {
              frame->can_id |= FLAGRTR;
            }

          frame->can_dlc = rf->cs.dlc;

    	  *(uint32_t*)&frame->data[0] = __builtin_bswap32(rf->data[0].w00);
    	  *(uint32_t*)&frame->data[4] = __builtin_bswap32(rf->data[1].w00);

          /* Clear MB interrupt flag */
          regval  = getreg32(S32K1XX_CAN0_IFLAG1);
          regval |= (1 << mb_index);
          putreg32(regval, S32K1XX_CAN0_IFLAG1);

          /* Copy the buffer pointer to priv->dev..  Set amount of data
           * in priv->dev.d_len
           */

          priv->dev.d_len = sizeof(struct can_frame);
          priv->dev.d_buf = frame;
        }

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);

      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
       * used during the next write.  If the write queue is full, then
       * this will point at an active buffer, which must not be written
       * to.  This is OK because devif_poll won't be called unless the
       * queue is not full.
       */
      priv->dev.d_buf = priv->txdesc;
    }
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

static void s32k1xx_txdone(FAR struct s32k1xx_driver_s *priv, uint32_t flags)
{
  #warning Missing logic

  /* We are here because a transmission completed, so the watchdog can be
   * canceled.
   */

  wd_cancel(priv->txtimeout);

  /* FIXME process aborts */

  /* Process TX completions */

  uint32_t mb_bit = 1 << RXMBCOUNT;
  for (uint32_t mbi = 0; flags && mbi < TXMBCOUNT; mbi++)
    {
      if (flags & mb_bit)
        {
          putreg32(mb_bit, S32K1XX_CAN0_IFLAG1);
          flags &= ~mb_bit;
#if 0
          const bool txok = priv->tx[mbi].cs.code != CAN_TXMB_ABORT;
          handleTxMailboxInterrupt(mbi, txok, utc_usec);
#endif

          NETDEV_TXDONE(&priv->dev);
        }

      mb_bit <<= 1;
    }

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  devif_poll(&priv->dev, s32k1xx_txpoll);
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
  FAR struct s32k1xx_driver_s *priv = &g_flexcan[0];
  uint32_t flags;
  flags  = getreg32(S32K1XX_CAN0_IFLAG1);
  flags &= IFLAG1_RX;

  if (flags)
    {
      s32k1xx_receive(priv, flags);
    }

  flags  = getreg32(S32K1XX_CAN0_IFLAG1);
  flags &= IFLAG1_TX;

  if (flags)
    {
      s32k1xx_txdone(priv, flags);
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
  FAR struct s32k1xx_driver_s *priv = (FAR struct s32k1xx_driver_s *)arg;

  /* Check if there is there is a transmission in progress.  We cannot
   * perform the TX poll if he are unable to accept another packet for
   * transmission.
   */

  net_lock();
  if (!s32k1xx_txringfull(priv))
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

static void s32k1xx_setenable(uint32_t enable)
{
  uint32_t regval;

  if (enable)
    {
      regval  = getreg32(S32K1XX_CAN0_MCR);
      regval &= ~(CAN_MCR_MDIS);
      putreg32(regval, S32K1XX_CAN0_MCR);
    }
  else
    {
      regval  = getreg32(S32K1XX_CAN0_MCR);
      regval |= CAN_MCR_MDIS;
      putreg32(regval, S32K1XX_CAN0_MCR);
    }

  s32k1xx_waitmcr_change(CAN_MCR_LPMACK, 1);
}

static void s32k1xx_setfreeze(uint32_t freeze)
{
  uint32_t regval;
  if (freeze)
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
  const unsigned timeout = 1000;
  for (unsigned wait_ack = 0; wait_ack < timeout; wait_ack++)
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

  if (!s32k1xx_initialize(priv))
    {
      nerr("initialize failed");
      return -1;
    }

  /* Set and activate a timer process */

  wd_start(priv->txpoll, S32K1XX_WDDELAY, s32k1xx_polltimer_expiry, 1,
           (wdparm_t)priv);

  priv->bifup = true;

  priv->txdesc = &g_tx_pool;
  priv->rxdesc = &g_rx_pool;

  priv->dev.d_buf = priv->txdesc;

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

#ifdef WORK_QUEUE_BYPASS
      s32k1xx_txavail_work(priv);
#else
      work_queue(ETHWORK, &priv->pollwork, s32k1xx_txavail_work, priv, 0);
#endif
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
 * Function: s32k1xx_initalize
 *
 * Description:
 *   Initialize FLEXCAN device
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

static int s32k1xx_initialize(struct s32k1xx_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  /* initialize CAN device */

  /* FIXME we only support a single can device for now */

  /* TEST GPIO tming */

  s32k1xx_pinconfig(PIN_PORTD | PIN31 | GPIO_OUTPUT);

  s32k1xx_setenable(0);

  /* Set SYS_CLOCK src */

  regval  = getreg32(S32K1XX_CAN0_CTRL1);
  regval |= CAN_CTRL1_CLKSRC;
  putreg32(regval, S32K1XX_CAN0_CTRL1);

  s32k1xx_setenable(1);

  s32k1xx_reset(priv);

  /* Enter freeze mode */

  s32k1xx_setfreeze(1);
  if (!s32k1xx_waitfreezeack_change(1))
    {
      ninfo("FLEXCAN: freeze fail\r\n");
      return -1;
    }


  /* Based on 80Mhz BUS clock calc through S32DS */
  regval  = getreg32(S32K1XX_CAN0_CBT);
  regval |= CAN_CBT_BTF |          /* Enable extended bit timing configurations
                                    * for CAN-FD for setting up separately
                                    * nominal and data phase */
            CAN_CBT_EPRESDIV(3) |  /* Prescaler divisor factor of 3 */
            CAN_CBT_EPROPSEG(7) | /* Propagation segment of 7 time quantas */
            CAN_CBT_EPSEG1(6) |   /* Phase buffer segment 1 of 6 time quantas */
            CAN_CBT_EPSEG2(3) |   /* Phase buffer segment 2 of 3 time quantas */
            CAN_CBT_ERJW(1);      /* Resynchronization jump width */
  putreg32(regval, S32K1XX_CAN0_CBT);

#ifdef CAN_FD
  /* Enable CAN FD feature */

  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval |= CAN_MCR_FDEN;
  putreg32(regval, S32K1XX_CAN0_MCR);

  /* Based on 80Mhz BUS clock calc through S32DS */
  regval  = getreg32(S32K1XX_CAN0_FDCBT);
  regval |= CAN_FDCBT_FPRESDIV(0) | /* Prescaler divisor factor of 1 */
            CAN_FDCBT_FPROPSEG(15) | /* Propagation semgment of 7 time quantas
                                     * (only register that doesn't add 1) */
            CAN_FDCBT_FPSEG1(1) |   /* Phase buffer segment 1 of 7 time quantas */
            CAN_FDCBT_FPSEG2(1) |   /* Phase buffer segment 2 of 5 time quantas */
            CAN_FDCBT_FRJW(1);      /* Resynchorinzation jump width same as PSEG2 */
  putreg32(regval, S32K1XX_CAN0_FDCBT);

  /* Additional CAN-FD configurations */

  regval  = getreg32(S32K1XX_CAN0_FDCTRL);

  regval |= CAN_FDCTRL_FDRATE |     /* Enable bit rate switch in data phase of frame */
            CAN_FDCTRL_TDCEN |      /* Enable transceiver delay compensation */
            CAN_FDCTRL_TDCOFF(5) |  /* Setup 5 cycles for data phase sampling delay */
            CAN_FDCTRL_MBDSR0(3);   /* Setup 64 bytes per message buffer (7 MB's) */
  putreg32(regval, S32K1XX_CAN0_FDCTRL);

  regval  = getreg32(S32K1XX_CAN0_CTRL2);
  regval |= CAN_CTRL2_ISOCANFDEN;
  putreg32(regval, S32K1XX_CAN0_CTRL2);
#endif

  for (i = TXMBCOUNT; i < TOTALMBCOUNT; i++)
    {
      priv->rx[i].id.w = 0x0;
    }

  putreg32(0x0, S32K1XX_CAN0_RXFGMASK);

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      putreg32(0, S32K1XX_CAN0_RXIMR(i));
    }

  for (i = 0; i < RXMBCOUNT; i++)
    {
      ninfo("Set MB%i to receive %p\r\n", i, &priv->rx[i]);
      priv->rx[i].cs.edl = 0x1;
      priv->rx[i].cs.brs = 0x1;
      priv->rx[i].cs.esi = 0x0;
      priv->rx[i].cs.code = 4;
      priv->rx[i].cs.srr = 0x0;
      priv->rx[i].cs.ide = 0x1;
      priv->rx[i].cs.rtr = 0x0;
    }

  putreg32(IFLAG1_RX, S32K1XX_CAN0_IFLAG1);
  putreg32(IFLAG1_RX, S32K1XX_CAN0_IMASK1);

  /* Exit freeze mode */

  s32k1xx_setfreeze(0);
  if (!s32k1xx_waitfreezeack_change(0))
    {
      ninfo("FLEXCAN: unfreeze fail\r\n");
      return -1;
    }

  return 1;
}

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
  uint32_t regval;
  uint32_t i;

  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval |= CAN_MCR_SOFTRST;
  putreg32(regval, S32K1XX_CAN0_MCR);

  if (!s32k1xx_waitmcr_change(CAN_MCR_SOFTRST, 0))
    {
      nerr("Reset failed");
      return;
    }

  /* TODO calculate TASD */

  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval &= ~(CAN_MCR_SUPV);
  putreg32(regval, S32K1XX_CAN0_MCR);

  /* Initialize all MB rx and tx */

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      ninfo("MB %i %p\r\n", i, &priv->rx[i]);
      ninfo("MB %i %p\r\n", i, &priv->rx[i].id.w);
      priv->rx[i].cs.cs = 0x0;
      priv->rx[i].id.w = 0x0;
      priv->rx[i].data[0].w00 = 0x0;
      priv->rx[i].data[1].w00 = 0x0;
    }

  regval  = getreg32(S32K1XX_CAN0_MCR);
  regval |= CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_SRXDIS |
            CAN_MCR_IRMQ | CAN_MCR_AEN |
            (((TOTALMBCOUNT - 1) << CAN_MCR_MAXMB_SHIFT) & CAN_MCR_MAXMB_MASK);
  putreg32(regval, S32K1XX_CAN0_MCR);

  regval  = CAN_CTRL2_RRS | CAN_CTRL2_EACEN; /* FIXME TASD */
  putreg32(regval, S32K1XX_CAN0_CTRL2);

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      putreg32(0, S32K1XX_CAN0_RXIMR(i));
    }

  /* Filtering catchall */

  putreg32(0x3fffffff, S32K1XX_CAN0_RX14MASK);
  putreg32(0x3fffffff, S32K1XX_CAN0_RX15MASK);
  putreg32(0x3fffffff, S32K1XX_CAN0_RXMGMASK);
  putreg32(0x0, S32K1XX_CAN0_RXFGMASK);
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

  /* FIXME dynamic board config */

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
  priv->dev.d_ifup    = s32k1xx_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = s32k1xx_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = s32k1xx_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = s32k1xx_ioctl;     /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = (void *)g_flexcan; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->txpoll        = wd_create();       /* Create periodic poll timer */
  priv->txtimeout     = wd_create();       /* Create TX timeout timer */
  priv->rx            = (struct mb_s *)(S32K1XX_CAN0_MB);
  priv->tx            = (struct mb_s *)(S32K1XX_CAN0_MB +
                          (sizeof(struct mb_s) * RXMBCOUNT));

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

/* FIXME CONFIG_S32K1XX_FLEXCAN_NETHIFS == 1 && */

#if !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void)
{
  s32k1xx_netinitialize(0);
}
#endif

#endif /* CONFIG_S32K1XX_FLEXCAN */
